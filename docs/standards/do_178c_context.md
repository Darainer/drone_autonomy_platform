# DO-178C Context for Safety Review

This document provides context on DO-178C, "Software Considerations in Airborne Systems and Equipment Certification," as an applicable standard for safety-critical development within this project. This standard will guide the Safety Review agent in its analysis.

## DO-178C Overview:

*   **Purpose:** DO-178C is a formal standard used to certify software in airborne systems, ensuring it is safe, reliable, and operates as intended. It is the primary means by which authorities like the FAA (Federal Aviation Administration) and EASA (European Union Aviation Safety Agency) approve avionics software.
*   **Objective-Based:** The standard is not a rigid set of rules but rather a collection of objectives to be satisfied. This allows flexibility in development methods while ensuring safety goals are met.
*   **Design Assurance Levels (DALs):** A core concept of DO-178C is the DAL, which ranks the criticality of a software component based on the potential consequences of its failure. The levels range from DAL A (Catastrophic) to DAL E (No Safety Effect). A higher DAL requires a more rigorous development and verification process.

## Relevance to this Project:

Given that this is a drone autonomy platform, the principles of DO-178C are directly applicable to the safety-critical components. The Safety Review agent will apply these principles when reviewing any pull requests that modify the `src/control` and `src/safety` directories. Changes will be assessed for their impact on system safety and reliability, guided by the rigorous, objective-based approach of DO-178C.