function [u,W,iter] = wls_alloc(B,v,umin,umax,Wv,Wu,ud,gam,u,W,imax)
  
% WLS_ALLOC - Control allocation using weighted least squares.
%
%  [u,W,iter] = wls_alloc(B,v,umin,umax,[Wv,Wu,ud,gamma,u0,W0,imax])
%
% Solves the weighted, bounded least-squares problem
%
%   min ||Wu(u-ud)||^2 + gamma ||Wv(Bu-v)||^2
%
%   subj. to  umin <= u <= umax
%
% using an active set method.
%
%  Inputs:
%  -------
% B     control effectiveness matrix (k x m)
% v     commanded virtual control (k x 1)
% umin  lower position limits (m x 1)
% umax  upper position limits (m x 1)
% Wv    virtual control weighting matrix (k x k) [I]
% Wu    control weighting matrix (m x m) [I]
% ud    desired control (m x 1) [0]
% gamma weight (scalar) [1e6]
% u0    initial point (m x 1)
% W0    initial working set (m x 1) [empty]
% imax  max no. of iterations [100]
% 
%  Outputs:
%  -------
% u     optimal control
% W     optimal active set
% iter  no. of iterations (= no. of changes in the working set + 1)
%
%                            0 if u_i not saturated
% Working set syntax: W_i = -1 if u_i = umin_i
%                           +1 if u_i = umax_i
%
% See also: WLSC_ALLOC, IP_ALLOC, FXP_ALLOC, QP_SIM.
  
% Number of variables
  m = length(umin);
  
  % Set default values of optional arguments
  if nargin < 11
    imax = 100; % Heuristic value
    [k,m] = size(B);
    if nargin < 10, u = (umin+umax)/2; W = zeros(m,1); end
    if nargin < 8,  gam = 1e6;       end
    if nargin < 7,  ud = zeros(m,1); end
    if nargin < 6,  Wu = eye(m);     end
    if nargin < 5,  Wv = eye(k);     end
  end
      
  gam_sq = sqrt(gam);
  A = [gam_sq*Wv*B ; Wu];
  b = [gam_sq*Wv*v ; Wu*ud];
  
  % Initial residual.
  d = b - A*u;
  % Determine indeces of free variables.
  i_free = W==0;
  
  % Iterate until optimum is found or maximum number of iterations
  % is reached.
  for iter = 1:imax
    % ----------------------------------------
    %  Compute optimal perturbation vector p.
    % ----------------------------------------
    
    % Eliminate saturated variables.
    A_free = A(:,i_free);
    % Solve the reduced optimization problem for free variables.
    p_free = A_free\d;
    % Zero all perturbations corresponding to active constraints.
    p = zeros(m,1);
    % Insert perturbations from p_free into free the variables.
    p(i_free) = p_free;
    
    % ----------------------------
    %  Is the new point feasible?
    % ----------------------------
    
    u_opt = u + p;
    infeasible = (u_opt < umin) | (u_opt > umax);

    if ~any(infeasible(i_free))

      % ----------------------------
      %  Yes, check for optimality.
      % ----------------------------
      
      % Update point and residual.
      u = u_opt;
      d = d - A_free*p_free;
      % Compute Lagrangian multipliers.
      lambda = W.*(A'*d);
      % Are all lambda non-negative?
      if lambda >= -eps
	% / ------------------------ \
	% | Optimum found, bail out. |
	% \ ------------------------ /
	return;
      end
      
      % --------------------------------------------------
      %  Optimum not found, remove one active constraint.
      % --------------------------------------------------
      
      % Remove constraint with most negative lambda from the
      % working set.
      [lambda_neg,i_neg] = min(lambda);
      W(i_neg) = 0;
      i_free(i_neg) = 1;
    
    else
      
      % ---------------------------------------
      %  No, find primary bounding constraint.
      % ---------------------------------------
      
      % Compute distances to the different boundaries. Since alpha < 1
      % is the maximum step length, initiate with ones.
      dist = ones(m,1);
      i_min = i_free & p<0;
      i_max = i_free & p>0;

      dist(i_min) = (umin(i_min) - u(i_min)) ./ p(i_min);
      dist(i_max) = (umax(i_max) - u(i_max)) ./ p(i_max);

      % Proportion of p to travel
      [alpha,i_alpha] = min(dist);
      % Update point and residual.
      u = u + alpha*p;
      d = d - A_free*alpha*p_free;
      
      % Add corresponding constraint to working set.
      W(i_alpha) = sign(p(i_alpha));
      i_free(i_alpha) = 0;
      
    end
  
  end
  