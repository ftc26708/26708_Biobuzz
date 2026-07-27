# Team 26708 The Argonauts | 2026-2027 Software Repository

---

Welcome to the official code repository of **Team 26708, The Argonauts,** for the BIOBUZZ™ season. This codebase is built on top of the official FIRST Tech Challenge SDK, customized to leverage three main software pillars:

* **Pedro Pathing:** For precise localizer-driven trajectory following and drivetrain movement.
* **Ivy:** For asynchronous, clean, command-based control flow to run both parallel and sequential actions effortlessly.
* **FTControl Panels:** For real-time telemetry, graphs, diagnostics, and live constant tuning.

---

## Core Architecture Flow

This codebase is organized into a decoupled, unidirectional 5-package hierarchical architecture. To prevent architectural decay and circular dependencies, elements must follow a strict downward data flow; lateral communication between files is strictly prohibited across all packages, with explicit exceptions granted only to `.cmd` for macro composition, `.util` for helper execution, and `.opmode` for base class inheritance.

### 1. `.util`
* **Role:** Contains stateless math utilities such as coordinate transforms, sensor data filters, subsystem behavior models, and project-wide configurations and constants.
* **Data Flow:** Operates as the lowest level internal dependency layer. It handles algorithmic calculations and configurations on request but is prohibited from reading physical hardware or invoking subsystems directly.

### 2. `.subsys`
* **Role:** Contains dedicated wrapper classes for isolated mechanisms (e.g., drivetrain). This layer directly interfaces with the FTC SDK hardware maps, handling raw hardware reads and writes.
* **Data Flow:** Pulls kinematics and scaling values from `.util`. It is the only layer permitted to communicate directly with physical hardware. Subsystems remain completely blind to sister mechanisms.

### 3. `.robot`
* **Role:** Serves as the robot's composition layer. It constructs and owns every subsystem instance, providing a single access point through which the rest of the codebase references the robot's hardware abstractions.
* **Data Flow:** Depends on `.subsys` to initialize the robot's mechanisms. It contains no mechanism logic or cross-subsystem behaviors; those responsibilities belong to the `cmd` layer.

### 4. `.cmd`
* **Role:** Implements Ivy's command-based control framework, cleanly segregated into two distinct functional subpackages:
    * `.cmd.atom`: Discrete, isolated commands dedicated to a single subsystem's behavior.
    * `.cmd.macro`: Composite routines that leverage Ivy to sequence or parallelize multiple atomic actions into high-level macros.
* **Data Flow:** Utilizes the hardware endpoints exposed by `.robot`. Its commands query mechanism states and schedule hardware actions over time, keeping the active match loops clear of execution logic.

### 5. `.opmode`
* **Role:** The execution environment for match play, separated into three distinct execution packages:
    * `.opmode.auto`: Follows a pre-programmed set of instructions.
    * `.opmode.teleop`: Human-input interpretation mapping gamepads to active command states.
    * `.opmode.test`: Diagnostics, calibration, tuning, and other isolated testing routines.
* **Data Flow:** Sits at the highest level of the architecture. It instantiates the `.robot` mediator and triggers actions within `.cmd` based on match time, sensor data, and driver input.

---

## Contributing and Code Standards

To keep the repository stable throughout the competitive season, all developers must adhere to our repository workflows, code quality guidelines, and branching rules.

Before opening a pull request, please read our [Contributing Guidelines](.github/CONTRIBUTING.md) for information regarding the protocols this codebase follows.