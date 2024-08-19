% fuselageUnitTest test functions and library of fuselage
%
%   These functions define test cases with well known results.
%   This unit test should be run every time the file has been modified,
%   to prove that results are still as expected.
%   If any changes has been made to the function this test
%   script can be used to find unintended errors.
%
% Example call:
%   rt = table(runtests('fuselageUnitTest'))
%
% Literature: 
%   [1] https://blogs.mathworks.com/loren/2013/10/15/function-is-as-functiontests/
%
%   [2] https://de.mathworks.com/help/matlab/matlab_prog/write-function-based-unit-tests-.html
%

% Disclaimer:
%   SPDX-License-Identifier: GPL-3.0-only
% 
%   Copyright (C) 2020-2022 Yannic Beyer
%   Copyright (C) 2022 TU Braunschweig, Institute of Flight Guidance
% *************************************************************************

% Call all subfunction tests in this file
function tests = fuselageUnitTest
    tests = functiontests(localfunctions);
end


function fuselageExampleScriptTest(testCase)

run('fuselage_example');
close all;

end

function fuselageLibTest(testCase)

run('fuselage_lib_example_init')

end

