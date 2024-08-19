% rigidBodyUnitTest test function
%
% Example call:
%   rt = table(runtests('rigidBodyUnitTest'))

% Disclaimer:
%   SPDX-License-Identifier: GPL-3.0-only
% *************************************************************************

% Call all subfunction tests in this file
function tests = rigidBodyUnitTest
    tests = functiontests(localfunctions);
end

function rigidBodyExampleTest(testCase)

q_bg = [1;0;0;0];
M_bg = quat2Dcm(q_bg);
rigidBodyKinematicsQuat(zeros(3,1),zeros(3,1),q_bg,M_bg);
rigidBodyKinetics(zeros(3,1),zeros(3,1),1,eye(3),9.81,M_bg,zeros(3,1),zeros(3,1));

end

function rigidBodyLibExampleTest(testCase)

modelname = 'rigidBody_lib_example';
load_system(modelname);
set_param(modelname,'SimulationCommand','Update');
close_system(modelname, 0);

end
