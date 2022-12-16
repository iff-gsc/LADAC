% actuatorsUnitTest test function
%
% Example call:
%   rt = table(runtests('actuatorsUnitTest'))

% Disclamer:
%   SPDX-License-Identifier: GPL-2.0-only
% *************************************************************************

% Call all subfunction tests in this file
function tests = actuatorsUnitTest
    tests = functiontests(localfunctions);
end


function actuatorsLibExampleTest(testCase)

modelname = 'actuators_lib_example';
load_system(modelname);
set_param(modelname,'SimulationCommand','Update');
close_system(modelname, 0);

end
