% Test_inducedVelocityMomentumTheory implements test functions for the calculation
% function inducedVelocityMomentumTheory in LADAC library
%
%   These functions define test cases with well known results.
%   This unit test should be run every time the file has been modified,
%   to prove that results are still as expected.
%   If any changes has been made to the function this test
%   script can be used to find unintended errors.
%
%   example call: rt = table(runtests('Test_inducedVelocityMomentumTheory'))
%
% Literature: 
%   [1] https://blogs.mathworks.com/loren/2013/10/15/function-is-as-functiontests/
%
%   [2] https://de.mathworks.com/help/matlab/matlab_prog/write-function-based-unit-tests-.html
%

% Disclaimer:
%   SPDX-License-Identifier: GPL-3.0-only
% 
%   Copyright (C) 2020-2022 Fabian Guecker
%   Copyright (C) 2022 TU Braunschweig, Institute of Flight Guidance
% *************************************************************************

% Call all subfunction tests in this file
function tests = inducedVelocityUnitTest
    tests = functiontests(localfunctions);
end


function Test_Vx_Vz_equal_Zero(testCase)

V_x = 0;
V_z = 0;

actSolution = inducedVelocityMomentumTheory(V_x, V_z);    
expSolution = 1;

verifyEqual(testCase, actSolution, expSolution, 'RelTol', 1e-6)

end

function Test_Vertical_Decent_Slow(testCase)

V_x = 0;
V_z = -1.5;

actSolution = inducedVelocityMomentumTheory(V_x, V_z);    
expSolution = 2;

verifyEqual(testCase, actSolution, expSolution, 'RelTol', 1e-6)

end

function Test_Vertical_Decent_Fast(testCase)

V_x = 0;
V_z = -2;

actSolution = inducedVelocityMomentumTheory(V_x, V_z);    
expSolution = 1;

verifyEqual(testCase, actSolution, expSolution, 'RelTol', 1e-6)

end

% function Test_Vortex_Ring_State_Vx_0_0(testCase)
% 
% v_h = 1;
% V_x = 0;
% V_z = -1.51;
% testCase.verifyError(@()inducedVelocityMomentumTheory(V_x, V_z, v_h), 'MomentumTheory:VortexRingState');
% 
% 
% V_z = -1.99;
% testCase.verifyError(@()inducedVelocityMomentumTheory(V_x, V_z, v_h), 'MomentumTheory:VortexRingState');
% 
% end
