# MPC RRT* Motion Planning Toolkit

This repository hosts the `mpc_rrt_star` Python package, a modular implementation of
deterministic RRT* path planning combined with a model-predictive controller for
kinematic bicycle models. All actively maintained source code, documentation, and tests
live under [`mpc_rrt_star/`](mpc_rrt_star/).

Legacy prototype scripts and generated artefacts have been removed in favour of the
package, which offers a clean command-line interface, reusable modules, and automated
test coverage.

## Getting started

```bash
cd mpc_rrt_star
python -m venv .venv
source .venv/bin/activate
pip install -e .[dev]
```

With the environment active you can execute the full planning and control pipeline:

```bash
python -m mpc_rrt_star.cli run
```

The CLI automatically generates a deterministic test map if none is present and writes
visualisation artefacts (inflated map, RRT* tree snapshot, optional frame dumps) to the
working directory. Additional commands such as `generate-map`, `inflate-map`, and
`plan-path` are available; consult [`mpc_rrt_star/README.md`](mpc_rrt_star/README.md) for
a complete overview.
