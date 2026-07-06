#!/usr/bin/env python3
"""
Generate C4-model architecture views from the ROS2 source code.

Scans src/ for rclcpp / rclpy node definitions (create_publisher,
create_subscription, create_client, create_service, message_filters) and
emits C4-PlantUML diagram sources, SVGs rendered with Graphviz, and
interface tables into docs/architecture/c4/:

    level1_context.{puml,svg,md}         — L1 system context (operator, GCS, PX4, sensors)
    level2_container.{puml,svg,md}       — L2 container view (every ROS2 node, topic flows)
    level3_component_<pkg>.{puml,svg,md} — L3 component view per src/ package
    topics.md                            — full topic/service inventory with link status
    README.md                            — index

Rendering needs Java, Graphviz, and the PlantUML jar (>= 1.2023.x so the C4
stdlib is bundled). The jar is located via, in order: $PLANTUML_JAR, a
`plantuml` executable on PATH, ~/.local/share/plantuml/plantuml.jar,
/opt/plantuml/plantuml.jar (legacy/container fallback, no sudo needed for
the jar — only `apt-get install graphviz` requires root). Set up both with:

    bash scripts/setup_c4_tooling.sh

Usage:
    python scripts/generate_c4.py              # (re)generate all views + SVGs
    python scripts/generate_c4.py --check      # exit 1 if views are stale (CI/drift check)
    python scripts/generate_c4.py --no-render  # skip SVG rendering (no Java/Graphviz needed)

Topic matching rules:
  * remappings= entries in launch_ros Node(...) actions (src/**/launch/*.py
    and launch/*.py, matched by node name) are applied first
  * exact fully-qualified topic match           -> connected edge
  * private topics (~/x) expand to /<node>/x    -> only match if remapped;
    same-basename private topics on different nodes are flagged as
    "needs remap" candidate links
  * topics matching an EXTERNAL_SYSTEMS prefix  -> edge to/from external system
  * anything else                               -> listed as dangling
"""

from __future__ import annotations

import argparse
import os
import re
import shutil
import subprocess
import sys
import tempfile
from collections import defaultdict
from pathlib import Path

REPO = Path(__file__).resolve().parent.parent
SRC_DIR = REPO / "src"
OUT_DIR = REPO / "docs" / "architecture" / "c4"

GENERATED_HEADER = (
    "<!-- GENERATED FILE — do not edit by hand. "
    "Regenerate with: python scripts/generate_c4.py -->\n"
)
PUML_HEADER = (
    "' GENERATED FILE — do not edit by hand. "
    "Regenerate with: python scripts/generate_c4.py\n"
)

# External systems recognised by topic/service prefix.
# (alias, display name, description, prefix regex)
EXTERNAL_SYSTEMS = [
    (
        "mavros",
        "MAVROS / PX4 Autopilot",
        "MAVLink bridge to the PX4 flight controller",
        re.compile(r"^/mavros(/|$)"),
    ),
    (
        "oakd",
        "OAK-D Camera Driver",
        "depthai-ros driver publishing RGB + stereo depth",
        re.compile(r"^/oak(/|$)"),
    ),
    (
        "isaac_ros",
        "Isaac ROS (VSLAM / NITROS)",
        "NVIDIA Isaac ROS GPU-accelerated nodes",
        re.compile(r"^/visual_slam(/|$)"),
    ),
    (
        "viz",
        "Operator Visualization",
        "rqt_image_view / GCS video overlay consumers",
        re.compile(r"^/visualization(/|$)"),
    ),
]

# ── source parsing ──────────────────────────────────────────────────────────

CPP_NODE = re.compile(r'Node\(\s*"([^"]+)"')
CPP_PUB = re.compile(r'create_publisher<([^>]+)>\s*\(\s*"([^"]+)"')
CPP_SUB = re.compile(r'create_subscription<([^>]+)>\s*\(\s*"([^"]+)"')
CPP_CLIENT = re.compile(r'create_client<([^>]+)>\s*\(\s*"([^"]+)"')
CPP_SERVICE = re.compile(r'create_service<([^>]+)>\s*\(\s*"([^"]+)"')
CPP_MF_DECL = re.compile(r'message_filters::Subscriber<([^>]+)>\s+(\w+)\s*;')
CPP_MF_SUB = re.compile(r'\b(\w+)\.subscribe\(\s*this\s*,\s*"([^"]+)"')

PY_NODE = re.compile(r'super\(\)\.__init__\(\s*["\']([^"\']+)["\']')
STR_OR_NAME = r'("[^"]+"|\'[^\']+\'|[A-Za-z_]\w*)'
PY_PUB = re.compile(r'create_publisher\(\s*([A-Za-z_][\w.]*)\s*,\s*' + STR_OR_NAME)
PY_SUB = re.compile(r'create_subscription\(\s*([A-Za-z_][\w.]*)\s*,\s*' + STR_OR_NAME)
PY_MF_SUB = re.compile(
    r'message_filters\.Subscriber\(\s*self\s*,\s*([A-Za-z_][\w.]*)\s*,\s*' + STR_OR_NAME
)
PY_CLIENT = re.compile(r'create_client\(\s*([A-Za-z_][\w.]*)\s*,\s*' + STR_OR_NAME)
PY_PARAM = re.compile(r'declare_parameter\(\s*["\'](\w+)["\']\s*,\s*["\']([^"\']+)["\']')

LAUNCH_NODE = re.compile(r"\bNode\s*\(((?:[^()]|\([^()]*\))*)\)", re.S)
LAUNCH_NAME = re.compile(r"name\s*=\s*['\"]([^'\"]+)['\"]")
LAUNCH_REMAPS = re.compile(r"remappings\s*=\s*\[(.*?)\]", re.S)
LAUNCH_REMAP_TUPLE = re.compile(r"\(\s*['\"]([^'\"]+)['\"]\s*,\s*['\"]([^'\"]+)['\"]\s*\)")


def parse_launch_remappings() -> dict[str, dict[str, str]]:
    """Remappings per node name, from launch_ros Node(...) actions."""
    remaps: dict[str, dict[str, str]] = defaultdict(dict)
    launch_files = list(SRC_DIR.glob("*/launch/*.py")) + list((REPO / "launch").glob("*.py"))
    for path in sorted(launch_files):
        text = path.read_text(errors="replace")
        for block in LAUNCH_NODE.findall(text):
            name_m = LAUNCH_NAME.search(block)
            remaps_m = LAUNCH_REMAPS.search(block)
            if not name_m or not remaps_m:
                continue
            for src, dst in LAUNCH_REMAP_TUPLE.findall(remaps_m.group(1)):
                remaps[name_m.group(1)][src] = dst
    return remaps


def short_type(cpp_or_py_type: str) -> str:
    """drone_autonomy_msgs::msg::Mission -> drone_autonomy_msgs/Mission"""
    t = cpp_or_py_type.strip()
    parts = [p for p in t.split("::") if p not in ("msg", "srv")]
    return "/".join(parts) if "::" in t else t


def resolve_py_topic(token: str, params: dict[str, str]) -> tuple[str | None, bool]:
    """Return (topic, from_param). Resolves variables named after a declared parameter."""
    if token[0] in "\"'":
        return token[1:-1], False
    if token in params:
        return params[token], True
    return None, False


class RosNode:
    def __init__(self, name: str, package: str, lang: str, file: Path):
        self.name = name
        self.package = package
        self.lang = lang
        self.file = file
        self.pubs: list[tuple[str, str]] = []  # (topic as written, msg type)
        self.subs: list[tuple[str, str]] = []
        self.clients: list[tuple[str, str]] = []  # (service, srv type)
        self.services: list[tuple[str, str]] = []
        self.notes: list[str] = []
        self.remaps: dict[str, str] = {}

    def fq(self, topic: str) -> str:
        """Apply launch remappings, then expand ~/x to /<node_name>/x."""
        topic = self.remaps.get(topic, topic)
        if topic.startswith("~"):
            topic = f"/{self.name}{topic[1:]}"
        topic = self.remaps.get(topic, topic)
        if not topic.startswith("/"):
            return f"/{topic}"
        return topic


def parse_sources() -> list[RosNode]:
    nodes: list[RosNode] = []
    for path in sorted(SRC_DIR.rglob("*")):
        if path.suffix not in (".cpp", ".py") or "/launch/" in str(path):
            continue
        text = path.read_text(errors="replace")
        package = path.relative_to(SRC_DIR).parts[0]

        if path.suffix == ".cpp":
            m = CPP_NODE.search(text)
            if not m:
                continue
            node = RosNode(m.group(1), package, "C++", path)
            node.pubs = [(t, short_type(ty)) for ty, t in CPP_PUB.findall(text)]
            node.subs = [(t, short_type(ty)) for ty, t in CPP_SUB.findall(text)]
            node.clients = [(t, short_type(ty)) for ty, t in CPP_CLIENT.findall(text)]
            node.services = [(t, short_type(ty)) for ty, t in CPP_SERVICE.findall(text)]
            # C++ message_filters::Subscriber<TYPE> member -> name, then
            # name.subscribe(this, "topic", ...) calls; fall back to a
            # generic type only if the member declaration isn't found.
            mf_types = {name: ty for ty, name in CPP_MF_DECL.findall(text)}
            for name, t in CPP_MF_SUB.findall(text):
                ty = mf_types.get(name, "message_filters::Subscriber")
                node.subs.append((t, short_type(ty)))
            nodes.append(node)
        else:
            m = PY_NODE.search(text)
            if not m:
                continue
            node = RosNode(m.group(1), package, "Python", path)
            params = dict(PY_PARAM.findall(text))
            for regex, target in ((PY_PUB, node.pubs), (PY_SUB, node.subs),
                                  (PY_MF_SUB, node.subs), (PY_CLIENT, node.clients)):
                for ty, token in regex.findall(text):
                    topic, from_param = resolve_py_topic(token, params)
                    if topic is None:
                        node.notes.append(
                            f"unresolved topic variable `{token}` ({regex.pattern[:30]}…)"
                        )
                        continue
                    target.append((topic, short_type(ty)))
                    if from_param:
                        node.notes.append(f"topic `{topic}` comes from parameter default")
            nodes.append(node)

    remaps = parse_launch_remappings()
    for node in nodes:
        node.remaps = remaps.get(node.name, {})
    return nodes


# ── model building ──────────────────────────────────────────────────────────

def external_for(topic: str) -> tuple[str, str] | None:
    for alias, name, _desc, rx in EXTERNAL_SYSTEMS:
        if rx.search(topic):
            return alias, name
    return None


def build_edges(nodes: list[RosNode]):
    """Return (edges, topic_rows, remap_warnings, dangling).

    edges: (src_alias, dst_alias, label, tech, kind)  kind in topic|remap|service
    """
    topic_pubs: dict[str, list[tuple[RosNode, str, str]]] = defaultdict(list)
    topic_subs: dict[str, list[tuple[RosNode, str, str]]] = defaultdict(list)
    for n in nodes:
        for t, ty in n.pubs:
            topic_pubs[n.fq(t)].append((n, t, ty))
        for t, ty in n.subs:
            topic_subs[n.fq(t)].append((n, t, ty))

    edges: list[tuple[str, str, str, str, str]] = []
    topic_rows: list[dict] = []
    remap_warnings: list[str] = []
    dangling: list[str] = []

    all_topics = sorted(set(topic_pubs) | set(topic_subs))
    for topic in all_topics:
        pubs = topic_pubs.get(topic, [])
        subs = topic_subs.get(topic, [])
        ext = external_for(topic)
        ty = (pubs or subs)[0][2]
        status = ""

        if pubs and subs:
            status = "connected"
            for pn, _, pty in pubs:
                for sn, _, _ in subs:
                    edges.append((pn.name, sn.name, topic, pty, "topic"))
        elif ext:
            alias, name = ext
            status = f"external ({name})"
            for pn, _, pty in pubs:
                edges.append((pn.name, alias, topic, pty, "topic"))
            for sn, _, sty in subs:
                edges.append((alias, sn.name, topic, sty, "topic"))
        else:
            # try basename match between private topics on different nodes
            base = topic.rsplit("/", 1)[-1]
            matched = False
            if pubs and not subs:
                for other, osubs in topic_subs.items():
                    if other != topic and other.rsplit("/", 1)[-1] == base:
                        for pn, _, pty in pubs:
                            for sn, _, _ in osubs:
                                edges.append(
                                    (pn.name, sn.name, f"{base} (needs remap)", pty, "remap")
                                )
                                remap_warnings.append(
                                    f"`{pn.name}` publishes `{topic}` but `{sn.name}` "
                                    f"subscribes `{other}` — same basename `{base}`; "
                                    "these only connect if remapped in a launch file."
                                )
                        matched = True
                status = "needs remap" if matched else "dangling (no subscriber)"
            elif subs and not pubs:
                for other in topic_pubs:
                    if other != topic and other.rsplit("/", 1)[-1] == base:
                        matched = True  # edge added from the pub side
                status = "needs remap" if matched else "dangling (no publisher)"
            if not matched:
                dangling.append(f"`{topic}` — {status}")

        topic_rows.append({
            "topic": topic,
            "type": ty,
            "pubs": ", ".join(p[0].name for p in pubs) or "—",
            "subs": ", ".join(s[0].name for s in subs) or "—",
            "status": status,
        })

    # services
    for n in nodes:
        for svc, ty in n.clients:
            ext = external_for(svc)
            servers = [m for m in nodes if any(m.fq(s) == n.fq(svc) for s, _ in m.services)]
            if servers:
                for m in servers:
                    edges.append((n.name, m.name, f"srv: {svc}", ty, "service"))
                status = "connected"
            elif ext:
                edges.append((n.name, ext[0], f"srv: {svc}", ty, "service"))
                status = f"external ({ext[1]})"
            else:
                status = "dangling (no server)"
                dangling.append(f"`{svc}` (service) — {status}")
            topic_rows.append({
                "topic": f"{svc} (service)", "type": ty,
                "pubs": n.name + " (client)", "subs": "—", "status": status,
            })

    return edges, topic_rows, remap_warnings, dangling


# ── C4-PlantUML rendering ───────────────────────────────────────────────────

REL_TAGS = [
    'AddRelTag("needs_remap", $lineStyle=DashedLine(), '
    '$legendText="needs remap (basename match only)")',
    'AddRelTag("service", $lineStyle=DottedLine(), $legendText="service call")',
]


def sanitize(alias: str) -> str:
    return re.sub(r"\W", "_", alias)


def used_externals(edges) -> list[tuple[str, str, str]]:
    aliases = {e[0] for e in edges} | {e[1] for e in edges}
    return [(a, n, d) for a, n, d, _rx in EXTERNAL_SYSTEMS if a in aliases]


def rel_line(src: str, dst: str, label: str, tech: str, kind: str, indent: str = "") -> str:
    tag = {"remap": "needs_remap", "service": "service"}.get(kind)
    tags = f', $tags="{tag}"' if tag else ""
    return f'{indent}Rel({sanitize(src)}, {sanitize(dst)}, "{label}", "{tech}"{tags})'


def diagram_md(title: str, stem: str, intro: str = "", extra: str = "") -> str:
    """Markdown page embedding the rendered SVG and linking its .puml source."""
    parts = [GENERATED_HEADER + f"# {title}", ""]
    if intro:
        parts += [intro, ""]
    parts += [f"![{title}]({stem}.svg)", "",
              f"Diagram source: [`{stem}.puml`]({stem}.puml) "
              "(C4-PlantUML, rendered with Graphviz)."]
    if extra:
        parts += ["", extra]
    return "\n".join(parts) + "\n"


def render_context() -> tuple[str, str]:
    puml = f"""{PUML_HEADER}@startuml
!include <C4/C4_Context>

title System Context — Drone Autonomy Platform

Person(operator, "Drone Operator", "Plans missions, monitors telemetry, retains override authority")
System(platform, "Drone Autonomy Platform", "ROS2 autonomy stack on Jetson Orin (this repository)")
System_Ext(gcs, "Ground Control Station", "QGroundControl / custom GCS")
System_Ext(px4, "PX4 Flight Controller", "Autopilot firmware, reached via MAVROS/MAVLink")
System_Ext(oakd, "OAK-D Camera", "RGB + stereo depth sensor")

Rel(operator, gcs, "Operates")
BiRel(gcs, px4, "MAVLink C2 + telemetry", "radio datalink")
BiRel(platform, px4, "Setpoints, mode commands, vehicle state", "MAVROS / DDS")
Rel(oakd, platform, "RGB + stereo depth images", "ROS2 topics")

SHOW_LEGEND()
@enduml
"""
    md = diagram_md(
        "C4 Level 1 — System Context", "level1_context",
        extra=("The context view is maintained from the `EXTERNAL_SYSTEMS` table and the\n"
               "static context template in `scripts/generate_c4.py` — update those when an\n"
               "external actor or system changes."),
    )
    return puml, md


def render_container(nodes: list[RosNode], edges) -> tuple[str, str]:
    by_pkg: dict[str, list[RosNode]] = defaultdict(list)
    for n in nodes:
        by_pkg[n.package].append(n)

    lines = [PUML_HEADER + "@startuml",
             "!include <C4/C4_Container>", "",
             "title Container View — ROS2 nodes of the Drone Autonomy Platform", ""]
    lines += REL_TAGS
    lines.append("")
    for alias, name, desc in used_externals(edges):
        lines.append(f'System_Ext({alias}, "{name}", "{desc}")')
    lines.append("")
    lines.append('System_Boundary(platform, "Drone Autonomy Platform") {')
    for pkg in sorted(by_pkg):
        lines.append(f'    Container_Boundary({sanitize(pkg)}_pkg, "src/{pkg}") {{')
        for n in sorted(by_pkg[pkg], key=lambda x: x.name):
            rel = n.file.relative_to(REPO)
            lines.append(
                f'        Container({sanitize(n.name)}, "{n.name}", '
                f'"ROS2 / {n.lang}", "{rel}")'
            )
        lines.append("    }")
    lines.append("}")
    lines.append("")
    seen = set()
    for src, dst, label, tech, kind in edges:
        key = (src, dst, label)
        if key in seen:
            continue
        seen.add(key)
        lines.append(rel_line(src, dst, label, tech, kind))
    lines += ["", "SHOW_LEGEND()", "@enduml"]
    puml = "\n".join(lines) + "\n"

    md = diagram_md(
        "C4 Level 2 — Container View (ROS2 Nodes)", "level2_container",
        intro=("Every deployable ROS2 node in `src/`, grouped by package, with topic and\n"
               "service flows extracted from the source. Dashed edges are *needs remap*\n"
               "candidates matched by topic basename only — see [`topics.md`](topics.md)\n"
               "for details."),
    )
    return puml, md


def render_component(pkg: str, nodes: list[RosNode], all_nodes, edges) -> tuple[str, str]:
    pkg_node_names = {n.name for n in nodes}
    pkg_edges = [e for e in edges if e[0] in pkg_node_names or e[1] in pkg_node_names]
    neighbors = ({e[0] for e in pkg_edges} | {e[1] for e in pkg_edges}) - pkg_node_names
    node_by_name = {n.name: n for n in all_nodes}
    ext_by_alias = {a: (n, d) for a, n, d, _ in EXTERNAL_SYSTEMS}

    lines = [PUML_HEADER + "@startuml",
             "!include <C4/C4_Component>", "",
             f"title Component View — src/{pkg}", ""]
    lines += REL_TAGS
    lines.append("")
    for nb in sorted(neighbors):
        if nb in ext_by_alias:
            name, desc = ext_by_alias[nb]
            lines.append(f'System_Ext({nb}, "{name}", "{desc}")')
        elif nb in node_by_name:
            lines.append(
                f'Container({sanitize(nb)}, "{nb}", "ROS2 node", '
                f'"src/{node_by_name[nb].package}")'
            )
    lines.append("")
    lines.append(f'Container_Boundary({sanitize(pkg)}, "src/{pkg}") {{')
    for n in sorted(nodes, key=lambda x: x.name):
        iface = f"{len(n.pubs)} pub / {len(n.subs)} sub"
        if n.clients:
            iface += f" / {len(n.clients)} srv client"
        lines.append(
            f'    Component({sanitize(n.name)}, "{n.name}", '
            f'"ROS2 node ({n.lang})", "{iface}")'
        )
    lines.append("}")
    lines.append("")
    seen = set()
    for src, dst, label, tech, kind in pkg_edges:
        key = (src, dst, label)
        if key in seen:
            continue
        seen.add(key)
        lines.append(rel_line(src, dst, label, tech, kind))
    lines += ["", "SHOW_LEGEND()", "@enduml"]
    puml = "\n".join(lines) + "\n"

    table = ["## Interfaces", "",
             "| Node | Direction | Topic / Service | Type |", "|---|---|---|---|"]
    for n in sorted(nodes, key=lambda x: x.name):
        for t, ty in n.pubs:
            table.append(f"| `{n.name}` | publishes | `{n.fq(t)}` | `{ty}` |")
        for t, ty in n.subs:
            table.append(f"| `{n.name}` | subscribes | `{n.fq(t)}` | `{ty}` |")
        for t, ty in n.clients:
            table.append(f"| `{n.name}` | service client | `{t}` | `{ty}` |")
        for t, ty in n.services:
            table.append(f"| `{n.name}` | service server | `{t}` | `{ty}` |")
    notes = [note for n in nodes for note in n.notes]
    extra = "\n".join(table)
    if notes:
        extra += "\n\n## Parser notes\n\n" + "\n".join(f"- {x}" for x in notes)

    md = diagram_md(f"C4 Level 3 — Component View: `src/{pkg}`",
                    f"level3_component_{pkg}", extra=extra)
    return puml, md


def render_topics(topic_rows, remap_warnings, dangling) -> str:
    lines = [GENERATED_HEADER, "# Topic & Service Inventory", "",
             "| Topic | Type | Publishers | Subscribers | Status |",
             "|---|---|---|---|---|"]
    for r in topic_rows:
        lines.append(
            f"| `{r['topic']}` | `{r['type']}` | {r['pubs']} | {r['subs']} | {r['status']} |"
        )
    if remap_warnings:
        lines += ["", "## ⚠ Remap warnings", ""]
        lines += [f"- {w}" for w in sorted(set(remap_warnings))]
    if dangling:
        lines += ["", "## Dangling interfaces", ""]
        lines += [f"- {d}" for d in dangling]
    return "\n".join(lines) + "\n"


def render_index(pkgs: list[str]) -> str:
    comp_links = "\n".join(
        f"- [Level 3 — src/{p}](level3_component_{p}.md)" for p in sorted(pkgs)
    )
    return f"""{GENERATED_HEADER}# C4 Architecture Views (generated)

Generated from the ROS2 source by `scripts/generate_c4.py` as C4-PlantUML
(`.puml`) sources rendered to SVG with Graphviz and embedded in the Markdown
pages. Regenerate after any change to nodes, topics, services, or launch
remappings:

```bash
python scripts/generate_c4.py          # regenerate views + SVGs
python scripts/generate_c4.py --check  # verify views match the code (CI)
```

Rendering needs Java + Graphviz + the PlantUML jar — see the docstring of
`scripts/generate_c4.py` (or `.claude/skills/c4/SKILL.md`) for setup.

- [Level 1 — System Context](level1_context.md)
- [Level 2 — Container View (ROS2 nodes)](level2_container.md)
{comp_links}
- [Topic & Service Inventory](topics.md)
"""


# ── SVG rendering (PlantUML + Graphviz) ─────────────────────────────────────

def find_plantuml(required: bool) -> list[str] | None:
    """Return the PlantUML invocation, or None (only when not required)."""
    problems = []
    if shutil.which("dot") is None:
        problems.append("Graphviz `dot` not found — install with: sudo apt-get install graphviz")
    cmd = None
    xdg_data = os.environ.get("XDG_DATA_HOME") or str(Path.home() / ".local" / "share")
    default_jar = Path(xdg_data) / "plantuml" / "plantuml.jar"
    jar = os.environ.get("PLANTUML_JAR")
    if jar and Path(jar).exists():
        cmd = ["java", "-jar", jar]
    elif shutil.which("plantuml"):
        cmd = ["plantuml"]
    elif default_jar.exists():
        cmd = ["java", "-jar", str(default_jar)]
    elif Path("/opt/plantuml/plantuml.jar").exists():  # legacy / container images
        cmd = ["java", "-jar", "/opt/plantuml/plantuml.jar"]
    else:
        problems.append(
            "PlantUML not found — set $PLANTUML_JAR, or fetch the jar "
            "(see docstring of scripts/generate_c4.py)"
        )
    if problems:
        msg = "; ".join(problems)
        if required:
            sys.exit(f"error: {msg}\n(use --no-render to skip SVG rendering)")
        print(f"note: {msg}")
        return None
    return cmd


def render_svgs(cmd: list[str], puml_dir: Path) -> None:
    """Render every .puml in puml_dir to .svg alongside it."""
    pumls = sorted(str(p) for p in puml_dir.glob("*.puml"))
    if not pumls:
        return
    res = subprocess.run(
        cmd + ["-tsvg", "-nometadata", "-charset", "UTF-8", *pumls],
        capture_output=True, text=True,
    )
    if res.returncode != 0:
        sys.exit(f"error: PlantUML rendering failed:\n{res.stdout}\n{res.stderr}")


# ── main ────────────────────────────────────────────────────────────────────

def generate() -> dict[str, str]:
    """Return {filename: content} for all text outputs (.md and .puml)."""
    nodes = parse_sources()
    if not nodes:
        sys.exit("error: no ROS2 nodes found under src/")
    edges, topic_rows, remap_warnings, dangling = build_edges(nodes)
    by_pkg: dict[str, list[RosNode]] = defaultdict(list)
    for n in nodes:
        by_pkg[n.package].append(n)

    files: dict[str, str] = {}
    files["level1_context.puml"], files["level1_context.md"] = render_context()
    files["level2_container.puml"], files["level2_container.md"] = \
        render_container(nodes, edges)
    for pkg, pkg_nodes in by_pkg.items():
        stem = f"level3_component_{pkg}"
        files[f"{stem}.puml"], files[f"{stem}.md"] = \
            render_component(pkg, pkg_nodes, nodes, edges)
    files["topics.md"] = render_topics(topic_rows, remap_warnings, dangling)
    files["README.md"] = render_index(list(by_pkg))
    return files


def check(files: dict[str, str], renderer: list[str] | None) -> int:
    stale = []
    for name, content in files.items():
        path = OUT_DIR / name
        if not path.exists() or path.read_text() != content:
            stale.append(name)

    svg_names = {n[:-5] + ".svg" for n in files if n.endswith(".puml")}
    if renderer is None:
        print("note: SVG rendering skipped or unavailable — checking .md/.puml only")
        for svg in sorted(svg_names):
            if not (OUT_DIR / svg).exists():
                stale.append(f"{svg} (missing)")
    else:
        with tempfile.TemporaryDirectory() as tmp:
            tmp_dir = Path(tmp)
            for name, content in files.items():
                if name.endswith(".puml"):
                    (tmp_dir / name).write_text(content)
            render_svgs(renderer, tmp_dir)
            for svg in sorted(svg_names):
                committed = OUT_DIR / svg
                if not committed.exists() or \
                        committed.read_bytes() != (tmp_dir / svg).read_bytes():
                    stale.append(svg)

    # flag committed views that would no longer be generated
    if OUT_DIR.exists():
        expected = set(files) | svg_names
        for path in sorted(OUT_DIR.iterdir()):
            if path.suffix in (".md", ".puml", ".svg") and path.name not in expected:
                stale.append(f"{path.name} (orphaned)")

    if stale:
        print("C4 views are stale — run: python scripts/generate_c4.py")
        for s in stale:
            print(f"  - {s}")
        return 1
    print("C4 views are up to date.")
    return 0


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--check", action="store_true",
                    help="exit 1 if generated views differ from files on disk")
    ap.add_argument("--no-render", action="store_true",
                    help="skip SVG rendering (only regenerate/check .md and .puml)")
    args = ap.parse_args()

    files = generate()
    renderer = None if args.no_render else find_plantuml(required=not args.check)

    if args.check:
        return check(files, renderer)

    OUT_DIR.mkdir(parents=True, exist_ok=True)
    for name, content in files.items():
        (OUT_DIR / name).write_text(content)
        print(f"wrote {OUT_DIR.relative_to(REPO) / name}")
    if renderer is None:
        print("warning: SVGs NOT re-rendered — committed diagrams may be stale")
    else:
        render_svgs(renderer, OUT_DIR)
        for puml in sorted(OUT_DIR.glob("*.puml")):
            print(f"wrote {puml.relative_to(REPO).with_suffix('.svg')}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
