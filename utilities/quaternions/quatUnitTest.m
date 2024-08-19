% quatUnitTest implements tests for the
% reduced state quaternion attitude controller in LADAC library
%
%   These functions define test cases with well known results.
%   This unit test should be run every time the file has been modified,
%   to prove that results are still as expected.
%   If any changes has been made to the function this test
%   script can be used to find unintended errors.
%
%   example calls:
%   rt = table(runtests('quatUnitTest'))
%   rt = table(runtests('quatUnitTest','OutputDetail',0))
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
function tests = quatUnitTest
tests = functiontests(localfunctions);
end

%% ------------------------------------------------------------------------
function Test_quatMultiply(testCase)

rng('default');

A = (2 * rand(4,50)) - 1;
B = (2 * rand(4,50)) - 1;

for i = 1:size(A,2)
    
    actResult = quatMultiply(A(:,i), B(:,i));
    expResult = quatmultiply(A(:,i)', B(:,i)')';
    
    verifyEqual(testCase, actResult, expResult, 'AbsTol', 4*eps);
    
end

end

%% ------------------------------------------------------------------------
function Test_quatConj(testCase)

rng('default');

A = (2 * rand(4,50)) - 1;
B = (2 * rand(4,50)) - 1;

for i = 1:size(A,2)
    
    actResult = quatConj(A(:,i));
    expResult = quatconj(A(:,i)')';
    
    verifyEqual(testCase, actResult, expResult, 'AbsTol', 4*eps);
    
end

end

%% ------------------------------------------------------------------------
function Test_quatInv(testCase)

rng('default');

A = (2 * rand(4,50)) - 1;
B = (2 * rand(4,50)) - 1;

for i = 1:size(A,2)
    
    actResult = quatInv(A(:,i));
    expResult = quatinv(A(:,i)')';
    
    verifyEqual(testCase, actResult, expResult, 'AbsTol', 4*eps);
    
end

end

%% ------------------------------------------------------------------------
function Test_quatDivide(testCase)

rng('default');

A = (2 * rand(4,50)) - 1;
B = (2 * rand(4,50)) - 1;

for i = 1:size(A,2)
    
    actResult = quatDivide(A(:,i), B(:,i));
    expResult = quatdivide(A(:,i)', B(:,i)')';
    
    verifyEqual(testCase, actResult, expResult, 'AbsTol', 4*eps);
    
end

end

%% ------------------------------------------------------------------------
function Test_quatNormalize(testCase)

rng('default');

A = (2 * rand(4,50)) - 1;

for i = 1:size(A,2)
    
    actResult = quatNormalize(A(:,i));
    expResult = quatnormalize(A(:,i)')';
    
    verifyEqual(testCase, actResult, expResult, 'AbsTol', 4*eps);
    
end

end

%% ------------------------------------------------------------------------
function Test_quatNorm(testCase)

rng('default');

A = (2 * rand(4,50)) - 1;

for i = 1:size(A,2)
    
    actResult = quatNorm(A(:,i));
    expResult = norm(quaternion(A(:,i)'));
    
    verifyEqual(testCase, actResult, expResult, 'AbsTol', 4*eps);
    
end

end

%% ------------------------------------------------------------------------
function Test_quatLog(testCase)

rng('default');

A = (2 * rand(4,50)) - 1;

for i = 1:size(A,2)
    
    q = A(:,i);
    
    actResult = quatLog(q);
    
    [w, x, y, z] = parts(log(quaternion(q')));
    expResult = [w; x; y; z];
    
    verifyEqual(testCase, actResult, expResult, 'AbsTol', 4*eps);
    
end

end

%% ------------------------------------------------------------------------
function Test_quatExp(testCase)

rng('default');

A = (2 * rand(4,50)) - 1;

for i = 1:size(A,2)
    
    q = A(:,i);
    
    actResult = quatExp(q);
    
    [w, x, y, z] = parts(exp(quaternion(q')));
    expResult = [w; x; y; z];
    
    verifyEqual(testCase, actResult, expResult, 'AbsTol', 4*eps);
    
end

end

%% ------------------------------------------------------------------------
function Test_quatIntegration(testCase)

for dt = 1:10
    
    q0 = [1; 0; 0; 0];
    
    actResult = quatIntegration(q0, [pi/2; 0; 0] , [0; 0; 0], dt);
    
    expResult = euler2Quat([pi/2*dt; 0; 0]);
    
    verifyEqual(testCase, actResult, expResult, 'AbsTol', 4*eps);
    
end

for dt = 1:10
    
    q0 = [1; 0; 0; 0];
    
    actResult = quatIntegration(q0, [0; 0; 0], [pi/2; 0; 0], dt);
    
    expResult = euler2Quat([pi/4*dt^2; 0; 0]);
    
    verifyEqual(testCase, actResult, expResult, 'AbsTol', 4*eps);
    
end

end

%% ------------------------------------------------------------------------
function Test_quatLogDivide(testCase)

rng('default');

A = (2 * rand(4,50)) - 1;
B = (2 * rand(4,50)) - 1;

for i = 1:size(A,2)
    
    q = A(:,i);
    qr = B(:,i);
    
    actResult = quatLogDivide(q, qr, false);
    
    [w, x, y, z] = parts(log(quaternion(quatdivide(q',qr'))));
    expResult = [0; x; y; z];
    
    verifyEqual(testCase, actResult, expResult, 'AbsTol', 4*eps);
    
end

end






