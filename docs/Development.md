# Development

Please read carefully through this guide if you want to develop code in LADAC.


## General principles

1. Name everything well.
2. Make sure that nothing has the same name as existing Matlab functions.
3. Make sure that no other files have "Test" or "test" in the name at the beginning or end than those for unit test (see [Testing section](#Testing)). 
Useful alternatives: eval, probe, verify, check, try, analyse, trial
4. Strike a balance between simplicity and not-repeating code.
5. Know that there's always more than one way to do something and that code is never final - but it does have to work.
6. This is scientific software. Provide the source for each line of code that implements a scientific method.
7. Function names should be `lowerCamelCase`.
8. Class names should be `UpperCamelCase`.
9. Folder names should be `word` or `multiple_words`.
10. Variable names should be `word` or `multiple_words` but you can also use typical math nomenclature (e.g. `M_bg` or `omega_Kb`). Booleans should start with `is_...`.
11. Script names should always contain an `_`, e.g. `multiple_words` or `lowerCamelCase_word`.
12. If you work on bigger projects, use structs.
	You could also use [System objects](https://de.mathworks.com/help/matlab/matlab_prog/what-are-system-objects.html) (type of Matlab class) but no other classes since they are not supported by Simulink.
	Note that there are [limitations](https://de.mathworks.com/help/simulink/ug/comparison-of-custom-block-functionality.html) for System objects such as no continuous states.
	Cell arrays are usually also not supported by Simulink.
13. Simulink library files should have the same name as the folder plus the extension `_lib`, e.g. `multiple_words_lib`.
	Moreover, Simulink files should be saved in .slx format.

## Single function vs. project

If you only implement a single function, move it to an appropriate location and give it an appropriate name.

If you are working on a (bigger) project that consists of multiple functions, move them to the same location.
Make sure that the function names start with the same name (e.g. `myProjectNiceFunction`, `myProjectFancyStuff`).
An example script should then be named `myProject_example` and a Simulink test file should be named `myProject_lib_example`.

## Function style

- Use the template provided in _m-utils/templates_.
- Make yourself familiar with code that is compatible with [code generation](https://www.mathworks.com/help/phased/ug/about-code-generation.html) and make sure to use the [%#codegen directive](https://www.mathworks.com/help/simulink/ug/adding-the-compilation-directive-codegen.html).


## Simulink block style

- Make yourself familiar with [creating library blocks](https://www.mathworks.com/help/simulink/ug/creating-block-libraries.html).
- Consider the style used for existing blocks.
- Usually, library blocks should only have one output. If there are multiple output signals, you should assign them to a Simulink bus.
- Parameters should be passed by the Mask. If many parameters are required, you should think about replacing them by one parameter struct.
- Documentation:
  - If not much documentation is needed, it can be done by text in the Mask.
  - If the documentation is relatively long, you should think about to use a Markdown file that is located in the same folder as the Simulink block.
    - Provide the Link to the Markdown file in the Mask by adding a `Hyperlink` and specify the following under `Callback`: `web('Link_to_Markdown_file.md','-browser')`
    - Put the same command under Mask --> Edit Mask --> Documentation --> Help (you will get to the file by right-click the block --> Help)
  - The documentation should specify inputs, parameters and outputs.
  - If the input parameter is a struct, you should only provide the struct definition in the documentation.
	
## Working with structs

If you use structs in (bigger) projects, you should use the following style:
- provide a function that defines/initializes the struct called `myProjectInit`.
- the description of all variables inside the struct should be done in the `myProjectInit` file.
- provide a default parameters file called `myProject_params_default`.
- provide a function that loads the parameters from the parameters file called `myProjectLoadParams`.
- provide a function that creates the struct called `myProjectCreate` (inside this function the functions `myProjectInit` and `myProjectLoadParams` should be called).
- all other functions that compute stuff based on the struct, should have the struct as an input, e.g. `stuff = myProjectGetStuff(my_struct)` (where `my_struct = myProjectInit()`).
- all other functions that set variables inside the struct, should have the struct as an input and as an output, e.g. `my_struct = myProjectSetVariable(my_struct,...)`.

## Testing

You should use test-driven development (TDD).
In the best case, a MATLAB unit test for TDD exists for each function which can be used to check the expected values for correct functioning.
TDD in short: First write a test function with inputs and the corresponding outputs.
Then write a function that satisfies all tests.  
For more information, see [here](https://blogs.mathworks.com/loren/2013/10/15/function-is-as-functiontests/) and [here](https://de.mathworks.com/help/matlab/matlab_prog/write-function-based-unit-tests-.html).

To check if everything works, run `check_LADAC`.

The following style should be used:
- There should be a function and/or Simulink file to test the function(s).
- The function should end with `...UnitTest`, e.g. `myProjectUnitTest`.
- If there are multiple files for testing, move them to a `test` subdirectory.

## Documentation

Each subfolder (except of `test` folders or similar) should contain a `README.md` that provides the necessary information: usually motivation, test and how it works.

## Using Git and GitHub

Make yourself familiar with [Git](https://git-scm.com).  
There is also a good [documentation on GitHub](https://docs.gitlab.com/ee/topics/git/).  
You should know at least about the following concepts:
- commit, commit message
- branch, tag
- push, pull, fetch, clone
- submodule
- merge
- rebase


This repository follows the [GitHub flow](https://guides.github.com/introduction/flow/index.html).
Ensure that you understand it.

For your branches, please consider the following ([more information](https://docs.gitlab.com/ee/topics/gitlab_flow.html)):
- Commit often and push frequently (see [Rules for commits](Rules_for_commits.md)).
- Write good commit messages (see [Rules for commits](Rules_for_commits.md)).
- Keep your feature branches short-lived.
- If you add a feature, name your branch `feature/good_name`. If you add a bugfix, name your branch `bugfix/good_name`.




