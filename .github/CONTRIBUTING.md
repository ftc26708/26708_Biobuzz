# Team 26708 The Argonauts | Contributing Guidelines

---

## Branching Strategy

Development flows strictly upward: **Feature Branch ➔ dev ➔ main**.

### Branch Definitions
* **`main` (Production):** Verified competition code. Merges come only from `release/*` or `hotfix/*` with semantic version tags (e.g., `v1.2.0`). Direct commits blocked.
* **`dev` (Integration):** Central hub for ongoing work. Direct commits blocked; PR with passing CI required.
* **Feature Branches (`feature/`, `bugfix/`):** Isolated sandboxes off `dev` for scoped tasks.
* **`hotfix/v<Major>.<Minor>.<Patch>` (Emergency):** Fast-track tournament fixes off `main`. Requires patch increment, pit check, and dual merge into `main` and `dev`.

---

## PR & Peer Review Protocol

All merges into `dev` or `main` require a green CI build and human approval.

* **Feature Branch Reviews (`dev` target):** Requires 1 peer programmer approval. Please post subsystem test telemetry/results in the PR comments.
* **Release Branch Reviews (`main` target):** Requires approval from a lead programmer, lead driver, and lead tester.

---

## Codebase & Architectural Rules

### 1. Strict Downward Package Flow
Please follow the unidirectional import chain: `opmode` ➔ `cmd` ➔ `robot` ➔ `subsys` ➔ `util`. (For more information on the structure of the codebase, please refer [here](../README.md)).
* **Import Limits:** Please do not import higher-level packages into lower-level ones (e.g., `subsys` importing `cmb`).
* **Lateral Exceptions:** Allowed only within `.cmd` for macros, `.util` for helpers, and `.opmode` for base inheritance.

### 2. Explain the "Why" in Comments
Code structure explains what happens; comments explain why choices were made.
* **Tuning:** Document reasons for PID coefficients, encoder targets, and physical offsets.
* **Math:** Explain coordinate transforms, kinematics, and custom filter formulas.
* **State Machines:** Clarify why specific state changes or interrupts are scheduled.

### 3. Hardware Access Isolation
* **Direct Mapping:** Please restrict FTC SDK hardware map, motor, servo, and sensor calls strictly to the `subsys` package.
* **Subsystem Ownership:** The `.robot` layer owns every subsystem instance. Please interact with subsystems through these shared instances instead of constructing or accessing hardware independently.