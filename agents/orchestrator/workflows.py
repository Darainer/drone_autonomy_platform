"""
Temporal workflow definitions for drone_autonomy_platform.

Workflows are the top-level orchestration — they define the sequence
of agent activities, human gates, and retry policies.
"""

from dataclasses import dataclass
from datetime import timedelta
from temporalio import workflow
from temporalio.common import RetryPolicy

with workflow.unsafe.imports_passed_through():
    from agents.orchestrator.activities import (
        create_feature_branch,
        analyze_intent,
        run_domain_agent,
        run_code_review,
        run_simulation,
        run_deploy,
        update_docs,
    )


retry_policy = RetryPolicy(
    maximum_attempts=5,
    initial_interval=timedelta(seconds=90),
    backoff_coefficient=1.5,
)


@dataclass
class FeatureRequest:
    description: str
    auto_approve: bool = False  # Skip human gates (for testing only)


@dataclass
class DeployRequest:
    version: str
    packages: list[str] | None = None  # None = all packages
    target: str = "orin"


# ─────────────────────────────────────────────────
# New Feature Workflow
# Intent → Plan → Implement → Test → Review → Docs
# ─────────────────────────────────────────────────

@workflow.defn
class NewFeatureWorkflow:
    """Full lifecycle for adding a new feature."""

    _human_approved: bool = False

    @workflow.run
    async def run(self, request: FeatureRequest) -> dict:
        # 1. Create a feature branch
        branch_info = await workflow.execute_activity(
            create_feature_branch,
            args=[request.description],
            start_to_close_timeout=timedelta(seconds=30),
            task_queue="orchestrator",
        )
        workflow.logger.info(f"Branch: {branch_info['branch']}")

        # 2. Orchestrator analyzes intent and creates a plan
        plan = await workflow.execute_activity(
            analyze_intent,
            args=[request.description],
            start_to_close_timeout=timedelta(minutes=5),
            retry_policy=retry_policy,
            task_queue="orchestrator",
        )

        workflow.logger.info(f"Plan: {plan['summary']} ({len(plan['steps'])} steps)")

        # 2. Execute each step in order
        results = []
        for step in plan["steps"]:
            result = await workflow.execute_activity(
                run_domain_agent,
                args=[step["agent"], step["action"], request.description],
                start_to_close_timeout=timedelta(minutes=15),
                retry_policy=retry_policy,
                task_queue=step.get("task_queue", "ros2-dev"),
            )
            results.append({"step": step, "result": result})

        # 3. Run simulation tests
        sim_results = await workflow.execute_activity(
            run_simulation,
            args=[plan],
            start_to_close_timeout=timedelta(minutes=30),
            retry_policy=retry_policy,
            task_queue="simulation",
        )

        # 4. Code review
        review = await workflow.execute_activity(
            run_code_review,
            args=[plan, results],
            start_to_close_timeout=timedelta(minutes=10),
            retry_policy=retry_policy,
            task_queue="orchestrator",
        )

        # 5. Human gate for safety-critical changes
        if plan.get("safety_critical") and not request.auto_approve:
            workflow.logger.info("⏸️  Waiting for human approval (safety-critical)")
            await workflow.wait_condition(
                lambda: self._human_approved,
                timeout=timedelta(hours=24),
            )

        # 6. Update docs
        await workflow.execute_activity(
            update_docs,
            args=[plan, results],
            start_to_close_timeout=timedelta(minutes=5),
            task_queue="orchestrator",
        )

        return {
            "plan": plan,
            "results": results,
            "sim_results": sim_results,
            "review": review,
            "status": "complete",
        }

    @workflow.signal
    async def approve(self):
        """Signal from human to approve and continue."""
        self._human_approved = True

    @workflow.query
    def get_status(self) -> str:
        return "awaiting_approval" if not self._human_approved else "approved"


# ─────────────────────────────────────────────────
# Deploy Workflow
# Review → Build → Human Gate → Deploy → Verify
# ─────────────────────────────────────────────────

@workflow.defn
class DeployWorkflow:
    """Build, review, and deploy to Orin Nano."""

    _deploy_approved: bool = False

    @workflow.run
    async def run(self, request: DeployRequest) -> dict:
        # 1. Full code review
        review = await workflow.execute_activity(
            run_code_review,
            args=[{"packages": request.packages}, []],
            start_to_close_timeout=timedelta(minutes=10),
            retry_policy=retry_policy,
            task_queue="orchestrator",
        )

        if not review.get("passed", False):
            return {"status": "blocked", "reason": "code_review_failed", "review": review}

        # 2. Run full regression
        sim_results = await workflow.execute_activity(
            run_simulation,
            args=[{"type": "regression", "packages": request.packages}],
            start_to_close_timeout=timedelta(minutes=45),
            retry_policy=retry_policy,
            task_queue="simulation",
        )

        # 3. Human gate — always for deploys
        workflow.logger.info(f"⏸️  Waiting for human approval to deploy {request.version}")
        await workflow.wait_condition(
            lambda: self._deploy_approved,
            timeout=timedelta(hours=24),
        )

        # 4. Cross-compile and deploy
        deploy_result = await workflow.execute_activity(
            run_deploy,
            args=[request.version, request.packages, request.target],
            start_to_close_timeout=timedelta(minutes=30),
            retry_policy=retry_policy,
            task_queue="deployment",
        )

        return {
            "version": request.version,
            "review": review,
            "sim_results": sim_results,
            "deploy": deploy_result,
            "status": "deployed",
        }

    @workflow.signal
    async def approve_deploy(self):
        self._deploy_approved = True


# ─────────────────────────────────────────────────
# ML Model Update Workflow
# Train → Export → Test → Deploy model
# ─────────────────────────────────────────────────

@workflow.defn
class ModelUpdateWorkflow:
    """Retrain, optimize, and deploy an ML model."""

    _deploy_approved: bool = False

    @workflow.run
    async def run(self, request: dict) -> dict:
        # 1. Train / export / optimize
        ml_result = await workflow.execute_activity(
            run_domain_agent,
            args=["ml-pipeline", request.get("action", "train and optimize"), request.get("description", "")],
            start_to_close_timeout=timedelta(hours=2),
            retry_policy=retry_policy,
            task_queue="ml-pipeline",
        )

        # 2. Integration test with perception pipeline
        sim_results = await workflow.execute_activity(
            run_simulation,
            args=[{"type": "ml_validation", "model": request.get("model_name", "")}],
            start_to_close_timeout=timedelta(minutes=30),
            retry_policy=retry_policy,
            task_queue="simulation",
        )

        # 3. Human gate
        workflow.logger.info("⏸️  Waiting for approval to deploy model")
        await workflow.wait_condition(
            lambda: self._deploy_approved,
            timeout=timedelta(hours=24),
        )

        # 4. Deploy model to Orin
        deploy_result = await workflow.execute_activity(
            run_deploy,
            args=[request.get("model_version", "latest"), ["perception"], "orin"],
            start_to_close_timeout=timedelta(minutes=20),
            retry_policy=retry_policy,
            task_queue="deployment",
        )

        return {
            "ml_result": ml_result,
            "sim_results": sim_results,
            "deploy": deploy_result,
            "status": "model_deployed",
        }

    @workflow.signal
    async def approve_deploy(self):
        self._deploy_approved = True
