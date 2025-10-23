
Act as a **senior software architect** and an **expert in Model Predictive Control (MPC) and vehicle dynamics**.
You are working on a **fully functional prototype** (current branch) that already implements:

* Deterministic occupancy grid generation and inflation
* RRT* global path planning
* MPC-based vehicle control (single-track model)
* Live path and vehicle visualization

The prototype **runs end-to-end correctly**.
Your task is **not to rewrite the functionality**, but to **modularize, document, and optimize** the system into a **maintainable, production-grade Python package**.

---

## **Objective**

Refactor the working prototype into a **modular motion planning and control framework** with:

1. Clean, testable module boundaries
2. Accurate vehicle-dynamics-aware MPC tracking
3. Deterministic RRT* planning with reproducible results
4. Full configuration control via YAML and CLI
5. Complete documentation and developer guides
6. Preservation of all existing functionality and behavior

---

## **Architectural Deliverables**

### **Final directory structure**

```
src/
  pyproject.toml
  README.md
  LICENSE
  src/
    __init__.py
    cli.py
    config.py
    logging_setup.py
    common/
      geometry.py
      types.py
    maps/
      generator.py
      inflate.py
      io.py
    planning/
      rrt_star.py
      plan_result.py
    control/
      vehicle_model.py
      mpc_controller.py
      ref_builder.py
    viz/
      vehicle_draw.py
      visualization.py
      record.py
  examples/
    generate_map.py
    plan_path.py
    run_pipeline.py
  tests/
    test_maps.py
    test_rrt_star.py
    test_vehicle_model.py
    test_mpc_controller.py
  docs/
    index.md
    architecture.md
    mpc_theory.md
    vehicle_dynamics.md
    planning_rrt_star.md
    usage_pipeline.md
    configuration.md
```

---

## **Implementation Rules**

1. **Do not alter algorithmic behavior.** Preserve all numeric parameters, solver settings, and plotting logic.
2. **Focus on structure and documentation**, not new features.
3. Each module must contain docstrings describing its purpose, math, and I/O signatures.
4. Provide a `PipelineConfig` dataclass combining `MapConfig`, `PlannerConfig`, `MPCConfig`, and `VizConfig`.
5. Implement a `click` CLI interface for map generation, inflation, planning, and full pipeline execution.
6. Use `yaml.safe_load` for config parsing.

---

## **Control System Design (from MPC Expert View)**

* **Model:**

  * Kinematic bicycle (single-track)
  * State: `[x, y, yaw, v]`, Input: `[accel, steer]`
  * Linearization via small-angle Jacobians
  * Discretization via forward Euler

* **MPC (OSQP-compatible QP):**

  * Stage cost: `(x - x_ref)^T Q (x - x_ref) + u^T R u`
  * Terminal cost: `(x_N - x_ref_N)^T QN (x_N - x_ref_N)`
  * Slack variables (quadratic) for soft constraints:

    * Velocity bounds
    * Input and rate limits
  * Solver: OSQP

    * `rho = 0.1`, `alpha = 1.6`, `adaptive_rho = True`, `max_iter = 60000`
  * Recovery: if infeasible, reduce reference velocity by 40%, relax rate limits once.

* **Constraints:**

  * Accel ∈ [−35, 35] px/s²
  * Steer ∈ [−0.6, 0.6] rad
  * Rate: accel ±12, steer ±0.15
  * Velocity ∈ [0, 90] px/s
  * Safety margins respected via inflated map.

---

## **Planner (RRT*) Requirements**

* Deterministic random seed for reproducibility.
* Configurable step, goal bias, iteration count, and rewire radius.
* Collision checking against inflated occupancy map.
* Return a structured `PlanResult` dataclass with path, nodes, and success flag.

---

## **Map System**

* White = free, black = occupied.
* Map generator deterministic with seed option.
* Inflation radius computed from vehicle half-width and margin.
* Pixel-to-meter conversion via `map_resolution`.

---

## **Visualization**

* Display RRT* tree, final path, MPC predicted trajectory, and vehicle pose.
* Animate MPC predictions frame-by-frame (`plot_prediction()`).
* Optional image sequence recording (`record.py`).
* Support headless rendering via backend `"Agg"`.

---

## **Documentation Requirements**

### **1. README.md**

Must include:

* Project overview
* Architecture diagram
* Theory of operation (RRT* + MPC)
* Installation instructions
* CLI usage examples
* Configuration reference
* Simulation demo instructions
* Contact and citation info

### **2. /docs folder**

Write a **comprehensive documentation set** including:

| File                   | Purpose                                       |
| ---------------------- | --------------------------------------------- |
| `index.md`             | Entry point overview                          |
| `architecture.md`      | Software architecture & module relationships  |
| `mpc_theory.md`        | MPC derivation, QP formulation, solver tuning |
| `vehicle_dynamics.md`  | Bicycle model equations & linearization       |
| `planning_rrt_star.md` | RRT* algorithm details & parameters           |
| `usage_pipeline.md`    | Step-by-step end-to-end workflow              |
| `configuration.md`     | YAML config schema and CLI flags              |

All `.md` files must be self-contained and include equations (Markdown + LaTeX syntax).

---

## **Testing**

* Unit tests for all modules using `pytest`.
* Validate:

  * Inflation kernel correct radius.
  * RRT* consistent path given fixed seed.
  * MPC feasibility and correct solver outputs.
  * Vehicle kinematics stable under nominal control.
  * Visualization runs under `Agg` backend.

---

## **Code Quality and Tooling**

* Type hints everywhere (`mypy --strict`)
* Formatting: `black`, linting: `ruff`
* Tests: `pytest`
* CI: run lint, type-check, and tests
* Documentation build check (Markdown links and images)

---

## **Final Deliverables**

1. **Refactored package** (`src/`)
2. **Comprehensive documentation** (`README.md` + `/docs`)
3. **Functional examples** (`examples/`)
4. **Config-driven CLI pipeline**
5. **Unit tests** under `tests/`
6. **Continuous Integration setup**
7. **Preserved functional behavior** from prototype

---

## **Implementation Roles**

You are acting as:

* **Software Architect:** Ensure modular, scalable design.
* **MPC Expert:** Maintain solver feasibility and robust vehicle tracking.
* **Vehicle Dynamics Engineer:** Enforce realistic dynamic limits and curvature constraints.
* **Optimization Specialist:** Guarantee numerical conditioning and QP stability.
* **Technical Writer:** Produce professional documentation and architecture guides.

---

## **Final Objective**

Deliver a **fully modular, documented, and production-ready version** of the **working prototype**, preserving end-to-end functionality while achieving architectural, numerical, and documentation excellence.
