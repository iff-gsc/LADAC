# LADAC documentation

This directory contains cross-cutting documentation for LADAC. Documentation for individual models and software components remains close to the corresponding source code in module-level `README.md` files.

## User documentation

- [Getting started](Getting_Started.md)
- [Installation](Installation.md)
- [Architecture and workflows](Architecture.md)
- [Aircraft modeling](Aircraft_Modeling.md)
- [Flight control](Flight_Control.md)
- [ArduPilot integration](ArduPilot_Integration.md)
- [Testing and troubleshooting](Testing.md)

## Developer documentation

- [Development guide](Development.md)
- [Rules for commits](Rules_for_commits.md)

## Documentation principle

The documentation is split into two complementary levels:

1. **Workflow documentation in `docs/`** explains how multiple LADAC components are combined to perform a task.
2. **Module documentation beside the code** explains the purpose, assumptions, interfaces, examples, validation, and references of a particular implementation.

When adding a new component, update both levels only when necessary. A low-level implementation normally needs a local `README.md`; a new cross-cutting workflow may additionally require an update in `docs/`.
