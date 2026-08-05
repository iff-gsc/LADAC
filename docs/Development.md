# Development guide

Read this guide before contributing code to LADAC.

## General principles

1. Choose descriptive and unambiguous names.
2. Avoid names that shadow MATLAB functions.
3. Reserve names containing `Test` or `test` at the beginning or end for tests. Alternatives include `example`, `evaluation`, `verification`, `check`, `analysis`, and `trial`.
4. Balance simplicity, reuse, and readability. Avoid both unnecessary duplication and premature abstraction.
5. Code does not need to be final, but every committed state must work within its documented scope.
6. LADAC is scientific software. Cite the source of implemented scientific methods and document assumptions and equations.
7. Use `lowerCamelCase` for functions.
8. Use `UpperCamelCase` for classes.
9. Use `word` or `multiple_words` for directory names.
10. Use descriptive variable names or established mathematical notation such as `M_bg` or `omega_Kb`. Boolean variables should normally start with `is_`.
11. Script names should contain an underscore to distinguish them from functions, for example `myProject_example`.
12. Use structures for parameter-rich models and interfaces.
13. Design reusable numerical functions for MATLAB, Simulink, and code generation where practical.
14. Keep aircraft-, experiment-, and publication-specific parameters outside LADAC.

System objects may be appropriate for selected components, but consider their Simulink and code-generation limitations. Avoid unsupported language features and data structures in code-generation paths.

## Repository organization

A single reusable function should be placed in the most appropriate existing area.

A larger component should have its own directory. Related functions should share a clear prefix:

```text
myProjectCreate.m
myProjectLoadParams.m
myProjectGetForces.m
myProjectSetGeometry.m
myProject_example.m
myProject_lib.slx
README.md
```

Do not create a new top-level directory for a small implementation. Top-level directories represent major physical or functional domains.

Use:

- `modules` for separately maintained dependencies included as Git submodules,
- `external` for third-party code distributed as a copy or partial copy,
- `interfaces` for integration with external software,
- `utilities` for LADAC-owned general-purpose helper functions.

Document third-party origin, version, license, and local modifications.

## MATLAB function style

- Start from the function template in `modules/m-utils/templates`.
- Include a concise help section that documents purpose, syntax, inputs, outputs, units, dimensions, coordinate frames, assumptions, and references.
- Add the `%#codegen` directive where code generation is intended.
- Prefer deterministic behavior and explicit data types in code-generation paths.
- Avoid hidden dependencies on the base workspace.
- Validate dimensions and configuration at initialization rather than in every simulation step where possible.

## Simulink library style

- Follow MATLAB's guidance for linked [library blocks](https://www.mathworks.com/help/simulink/ug/creating-block-libraries.html).
- Simulink library files should normally use the directory name followed by `_lib`, for example `multiple_words_lib.slx`.
- Save maintained models in `.slx` format.
- Follow the visual style of existing LADAC blocks.
- Prefer one output port. Use a Simulink bus when a component has multiple related outputs.
- Pass parameters through the mask. Use one parameter structure when many related parameters are required.
- Keep sample times, data types, units, axes, and reset behavior explicit.
- Avoid project-specific constants inside reusable library blocks.

### Block documentation

For short documentation, use mask text.

For substantial documentation, create a Markdown file in the same directory and link it from the block mask and help callback:

```matlab
web('README.md','-browser')
```

Document:

- purpose,
- assumptions and validity range,
- parameters,
- inputs and outputs,
- states and initialization,
- units and dimensions,
- examples,
- tests and validation,
- references,
- known limitations.

## Parameter structures

For larger components, use a consistent lifecycle where applicable:

- `myProjectInit` defines or initializes the structure.
- `myProject_params_default` is a documented default parameter template.
- `myProjectLoadParams` loads user-supplied parameters and derives dependent values.
- `myProjectCreate` assembles the complete structure.
- `myProjectGet...` functions calculate values from the structure.
- `myProjectSet...` functions return a modified structure.

Descriptions of structure fields should have one authoritative location, normally the initialization or creation function.

Users should copy parameter templates into their application repositories rather than edit LADAC defaults.

## Tests

Use test-driven development where practical.

Each component should provide appropriate tests at one or more levels:

- MATLAB unit tests,
- component examples,
- Simulink regression examples,
- verification against analytic or independent reference results,
- validation against experimental data.

Naming conventions:

- unit-test functions end in `UnitTest`,
- multiple test files belong in a `test` directory,
- demonstrations should use `example`, not `test`, unless they assert expected behavior.

Run the repository-level check:

```matlab
check_ladac
```

Also run the component-specific tests and any affected LADAC-Examples simulations.

## Documentation

Each substantive source directory should contain a `README.md`, except for directories such as `private`, generated-code directories, or self-explanatory test-data directories.

A component README should normally contain:

1. purpose,
2. motivation,
3. method and assumptions,
4. requirements,
5. examples,
6. usage and interfaces,
7. tests and validation,
8. known limitations,
9. literature.

Cross-cutting workflows belong in `docs/`. Avoid duplicating detailed component documentation there.

## Generated files

Do not commit generated or temporary files unless they are intentionally distributed artifacts.

Typical files to ignore include:

```text
slprj/
*.slxc
*.autosave
*.asv
*~
```

Before committing, review `git status` and verify that each file is intentional.

## Git and GitHub

Contributors should understand:

- clone, fetch, pull, and push,
- commits and commit messages,
- branches and tags,
- merges and rebases,
- Git submodules,
- pull requests and issue references.

LADAC follows a short-lived feature-branch workflow:

- `feature/descriptive_name`
- `bugfix/descriptive_name`
- `docs/descriptive_name`

Commit often, keep changes focused, and follow [Rules for commits](Rules_for_commits.md).

Before opening a pull request:

1. update documentation,
2. run relevant tests,
3. inspect changes for generated files,
4. verify library links,
5. describe scope, motivation, limitations, and validation.
