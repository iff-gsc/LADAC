# Testing and troubleshooting

## Repository-level check

After installation or an update, run:

```matlab
check_ladac
```

This is the first broad check of paths, libraries, dependencies, and selected functions.
It does not replace component-specific tests.

## Simulink Library Browser

Verify that:

1. LADAC appears in the Simulink Library Browser.
2. the main sublibraries open,
3. linked blocks can be inserted into a model,
4. no unresolved links are reported.

The main library can be opened directly:

```matlab
open_system('ladac_lib')
```

## Component tests

Testing conventions differ between older and newer components.
Common forms include:

- functions ending in `UnitTest`,
- MATLAB unit tests run with `runtests`,
- example scripts,
- Simulink example models,
- verification scripts comparing theory or reference data.

Example:

```matlab
rt = table(runtests('rigidBodyUnitTest'));
```

Read the local component README before running tests.

## Test levels

For a model or controller used in research, distinguish between:

1. **execution test:** the code runs without errors;
2. **unit test:** a function returns expected results for defined cases;
3. **verification:** implementation agrees with equations, limiting cases, or independent software;
4. **validation:** model output agrees sufficiently with experimental or flight-test data for an intended use;
5. **system test:** assembled simulation, SITL environment, or generated controller behaves correctly.

A passing example is usually only an execution or regression check.

## Troubleshooting

### MATLAB finds the wrong function

Check for multiple LADAC versions:

```matlab
which functionName -all
```

Remove unintended paths and restart MATLAB if necessary.

### A submodule directory is empty

Run:

```bash
git submodule update --init --recursive
```

### Simulink cannot resolve library links

- verify the LADAC path,
- open `ladac_lib.slx`,
- refresh the Library Browser,
- check whether paths changed during a repository reorganization,
- inspect linked-block paths.

### A model worked with another LADAC commit

Record and compare:

- LADAC commit,
- submodule commits,
- MATLAB release,
- project initialization scripts,
- parameter files,
- ArduPilot branch and commit.

For ArduPilot controller deployment, incompatible interface versions are a likely cause.

## Reporting an issue

Before opening an issue:

1. search existing issues,
2. run `check_ladac`,
3. run the smallest relevant component example,
4. check `git status`,
5. restart MATLAB/Simulink,
6. verify submodules.

Include:

- concise problem description,
- exact steps to reproduce,
- full error message,
- MATLAB release and operating system,
- LADAC commit SHA,
- relevant submodule commits,
- required files or a minimal example,
- expected and observed behavior.
