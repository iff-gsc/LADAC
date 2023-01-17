function [ y, x_dt ] = ssOutput( A, B, C, D, x, u ) %#codegen
% ssOutput computes the outputs y and x_dt of the state-space
%   representation.
% 
% Inputs:
%   A       system (n x n) matrix
%   B       input (n x m) matrix
%   C       output (q x n) matrix
%   D       feedtrhough (q x m) matrix
%   x       state (n x 1) vector
%   u       input (m x 1) vector
% 
% Outputs:
%   y       output (q x 1) vector
%   x_dt    time-derivative of x
% 

% Disclamer:
%   SPDX-License-Identifier: GPL-3.0-only
% 
%   Copyright (C) 2020-2022 Yannic Beyer
%   Copyright (C) 2022 TU Braunschweig, Institute of Flight Guidance
% *************************************************************************

n = size(A,1);
q = size(C,1);
num_vectors = size(x,2);

x_dt = zeros(n,num_vectors);
y = zeros(q,num_vectors);

if num_vectors == 1
    x = x.*ones(n,1);
    x_dt(:) = A*x + B*u;
    y(:) = C*x + D*u;
else
    for i = 1:num_vectors
        x_dt(:,i) = A(:,:,i)*x(:,i) + B(:,:,i)*u(:,i);
        y(:,i) = C(:,:,i)*x(:,i) + D(:,:,i)*u(:,i);
    end
end

end
