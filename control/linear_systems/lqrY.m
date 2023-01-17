function Ky = lqrY(A,B,C,Q,R,verbosity)
% lqrY returns the optimal output-feedback with state and input weighting.
%   Output feedback does not work in general. Moreover, this function uses
%   a rather rudimentary algorithm. Please refer to [1] on pp. 397 or other
%   literature for further details.
% 
% Inputs:
%   A           System matrix (nxn)
%   B           Input matrix (nxm)
%   C           Output matrix (pxn)
%   Q           State weighting matrix (nxn)
%   R           Input weighting matrix (mxm)
%   verbosity   0 if nothing should be printed, 1 if you want information
% 
% Outputs:
%   Ky          Optimal output-feedback matrix (mxp)
% 
% Literature:
% [1]   Stevens, B. L. et al. (2016): Aircraft Control and Simulation.
%       Dynamics, Control Design, and Autonomous Systems. 3rd ed. Wiley.
%

% Disclamer:
%   SPDX-License-Identifier: GPL-3.0-only
% 
%   Copyright (C) 2020-2022 Yannic Beyer
%   Copyright (C) 2022 TU Braunschweig, Institute of Flight Guidance
% *************************************************************************

% fsolve options (display nothing)
options = optimoptions('fsolve','Display','none');
% initial values for optimization
P = eye(length(A));
S = eye(length(A));

% X = E (Stevens, p. 402)
X = eye(length(A));

% 1. Initialize (see [1] on page 404)
[~,~,Kx_0] = care(A,B,eye(length(A)),eye(length(B(1,:))));
k = 0;
Ky = Kx_0*C';

% 2. kth iteration (see [1] on page 404)
Delta_J_max = 1e-6;     % value chosen empirically
converged = false;
while ~converged
    % Set A_k
    A_k = lqrY_Ak(A,B,Ky,C);
    % Solve for P_k and S_k in
    P = fsolve(@(P)lqrY_H(A_k,P,C,Ky,R,Q),P,options);
    S = fsolve(@(S)lqrY_S(A_k,S,X),S,options);
    % Set J_k
    J_k = lqrY_J(P,X);
    % Evaluate the gain update direction
    Delta_Ky = inv(R)*B'*P*S*C'*inv(C*S*C')-Ky;
    
    % Iteration that ensures that alpha is chosen correctly (own
    % development, not verified for general usage)
    j = 1;
    alpha = 0.1;        % value chosen empirically
    stable = false;
    while ~stable
        % Update the gain by
        Ky_test = Ky + alpha*Delta_Ky;
        % where alpha is chosen so that A_k ...
        A_k = lqrY_Ak(A,B,Ky_test,C);
        eigenVals = eig(A_k);
        P_kp1 = fsolve(@(P)lqrY_H(A_k,P,C,Ky_test,R,Q),P,options);
        J_kp1 = lqrY_J(P_kp1,X);
        if verbosity > 0
            fprintf('lqrY iteration status:  cost =  %e (k=%i,j=%i) \n',J_k,k,j)
        end
        % ... is asymptotically stable
        condition1 = all(real(eigenVals<0));
        % ... J decreases
        condition2 = J_kp1 <= J_k;
        if condition1 && condition2
            stable = true;
            Ky = Ky_test;
        else
            % alpha is reduced if the conditions are not fulfilled
            alpha = alpha / 2; 
            j = j+1;
        end 
    end
  
    % If J_k+1 and J_k are close enough to each other, go to 3.
    % Otherwise, set k=k+1 and go to 2.
    Delta_J = J_k - J_kp1;
    if Delta_J < Delta_J_max
        % 3. Terminate
        converged = true;
    else
        k = k+1;
    end
    
end

function H = lqrY_H(A,P,C,K,R,Q)
    H = A'*P + P*A + C'*K'*R*K*C + Q;
end

function G = lqrY_S(A,S,X)
    G = A*S + S*A' + X;
end

function J = lqrY_J(P,X)
    J = 0.5*trace(P*X);
end

function A_k = lqrY_Ak(A,B,K,C)
    A_k = A-B*K*C;
end

end