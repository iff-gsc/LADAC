% airfoilAnalytic0515UnitTest test functions and library of simple fuselage
%
%   These functions define test cases with well known results.
%   This unit test should be run every time the file has been modified,
%   to prove that results are still as expected.
%   If any changes has been made to the function this test
%   script can be used to find unintended errors.
%
% Example call:
%   rt = table(runtests('airfoilAnalytic0515UnitTest'))
%
% Literature: 
%   [1] https://blogs.mathworks.com/loren/2013/10/15/function-is-as-functiontests/
%
%   [2] https://de.mathworks.com/help/matlab/matlab_prog/write-function-based-unit-tests-.html
%

% Disclamer:
%   SPDX-License-Identifier: GPL-2.0-only
% 
%   Copyright (C) 2020-2022 Yannic Beyer
%   Copyright (C) 2022 TU Braunschweig, Institute of Flight Guidance
% *************************************************************************

% Call all subfunction tests in this file
function tests = airfoilAnalytic0515UnitTest
    tests = functiontests(localfunctions);
end


function airfoilAnalytic0515AlExampleScriptTest(testCase)

run('airfoilAnalytic0515Al_example');
close all;

end

