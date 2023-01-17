% batteryUnitTest test function
%
% Example call:
%   rt = table(runtests('batteryUnitTest'))

% Disclamer:
%   SPDX-License-Identifier: GPL-3.0-only
% *************************************************************************

% Call all subfunction tests in this file
function tests = batteryUnitTest
    tests = functiontests(localfunctions);
end


function batteryExampleScriptTest(testCase)

run('battery_example');
close all;

end
