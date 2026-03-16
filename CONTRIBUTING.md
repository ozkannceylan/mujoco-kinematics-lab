# Contributing

Thanks for contributing to MuJoCo Robotics Lab.

## Scope

This repository is a robotics lab series focused on MuJoCo simulation,
Pinocchio-based analysis, and reproducible educational writeups. Contributions
should improve correctness, clarity, testing, or documentation quality.

## Before You Start

- Open an issue first for large changes, architectural shifts, or new lab ideas.
- Keep pull requests focused. Small, reviewable changes merge faster.
- Do not remove or relicense bundled third-party assets unless their upstream
  license explicitly allows it and the relevant notices are preserved.

## Development Standards

- Use Python 3.10+.
- Add type hints and docstrings to new functions.
- Use `pathlib.Path` for filesystem paths.
- Keep comments in English.
- Add or update tests in the relevant `tests/` directory for behavior changes.
- When a lab includes paired English and Turkish docs, update both when the
  change affects user-facing documentation.

## Robotics-Specific Expectations

- Use Pinocchio for analytical computations and MuJoCo for simulation/runtime.
- Prefer cross-validation when both engines can compute the same quantity.
- Avoid hardcoded absolute paths.
- Use explicit tolerances for numerical assertions.

## Pull Request Checklist

- The change is scoped to a clear problem.
- Tests pass locally, or the PR explains why tests could not be run.
- Documentation is updated when behavior or usage changes.
- Third-party notices remain intact.

## Licensing

By submitting a contribution, you agree that your work will be licensed under
the Apache License, Version 2.0, for the portions of the repository maintained
under the root [LICENSE](LICENSE).
