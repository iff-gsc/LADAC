function [ x, w, u, v, Anorm, alfa, rhobar, phibar ] = ...
    ladac_lsqr_iterate( A, x, w, u, v, Anorm, alfa, rhobar, phibar)

damp = 0;

explicitA = true;

if isa(A,'numeric')
  explicitA = true;
elseif isa(A,'function_handle')
  explicitA = false;
else
  error('SOL:lsqrSOL:Atype','%s','A must be numeric or a function handle');
end

%------------------------------------------------------------------
%     Main iteration loop.
%------------------------------------------------------------------

% Perform the next step of the bidiagonalization to obtain the
% next beta, u, alfa, v.  These satisfy the relations
%      beta*u  =  A*v  - alfa*u,
%      alfa*v  =  A'*u - beta*v.

if explicitA
    u = A*v    - alfa*u;
else
    u = A(v,1) - alfa*u;
end

beta = norm(u);
if beta > 0
    u     = (1/beta)*u;
    Anorm = norm([Anorm alfa beta damp]);
    
    if explicitA
        v = A'*u   - beta*v;
    else
        v = A(u,2) - beta*v;
    end
    
    alfa  = norm(v);
    
    if alfa > 0
        v = (1/alfa)*v;
    end
    
end

% Use a plane rotation to eliminate the damping parameter.
% This alters the diagonal (rhobar) of the lower-bidiagonal matrix.

rhobar1 = norm([rhobar damp]);
cs1     = rhobar/rhobar1;
phibar  = cs1*phibar;

rho     =   norm([rhobar1 beta]);
cs      =   rhobar1/rho;
sn      =   beta   /rho;
theta   =   sn*alfa;
rhobar  = - cs*alfa;
phi     =   cs*phibar;
phibar  =   sn*phibar;

t1      =   phi  /rho;
t2      = - theta/rho;

x       = x      + t1*w;
w       = v      + t2*w;

end
