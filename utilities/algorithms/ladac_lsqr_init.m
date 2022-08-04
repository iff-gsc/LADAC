function [ x, w, u, v, Anorm, alfa, rhobar, phibar ] = ... 
    ladac_lsqr_init( A, b)

explicitA = true;

if isa(A,'numeric')
  explicitA = true;
elseif isa(A,'function_handle')
  explicitA = false;
else
  error('SOL:lsqrSOL:Atype','%s','A must be numeric or a function handle');
end

m = length(b);
n = length(b);

Anorm  = 0;             

% Set up the first vectors u and v for the bidiagonalization.
% These satisfy  beta*u = b,  alfa*v = A'u.

x    = zeros(n,1);
u    = b(1:m);        
v    = zeros(n,1);
w    = zeros(n,1);
alfa = 0;             
beta = norm(u);

rhobar = 0;
phibar = 0;

if beta > 0
   u = (1/beta)*u;
   
   if explicitA
       v = A'*u;
   else
       v = A(u,2);
   end
   
   alfa = norm(v);
end
if alfa > 0
   v = (1/alfa)*v;      w = v;
end

Arnorm = alfa*beta;     

if Arnorm == 0
    return;
end

rhobar = alfa;          phibar = beta;  

%------------------------------------------------------------------
%     Main iteration loop.
%------------------------------------------------------------------
% while itn < itnlim
%   itn = itn + 1;
% 
% % Perform the next step of the bidiagonalization to obtain the
% % next beta, u, alfa, v.  These satisfy the relations
% %      beta*u  =  A*v  - alfa*u,
% %      alfa*v  =  A'*u - beta*v.
% 
%   u = A*v    - alfa*u;
% 
%   beta = norm(u);
%   if beta > 0
%     u     = (1/beta)*u;
%     Anorm = norm([Anorm alfa beta damp]);
% 
%     v = A'*u   - beta*v;
% 
%     alfa  = norm(v);
%     
%     if alfa > 0
%         v = (1/alfa)*v;
%     end
%     
%   end
% 
% % Use a plane rotation to eliminate the damping parameter.
% % This alters the diagonal (rhobar) of the lower-bidiagonal matrix.
% 
%   rhobar1 = norm([rhobar damp]);
%   cs1     = rhobar/rhobar1;
%   phibar  = cs1*phibar;
% 
%   rho     =   norm([rhobar1 beta]);
%   cs      =   rhobar1/rho;
%   sn      =   beta   /rho;
%   theta   =   sn*alfa;
%   rhobar  = - cs*alfa;
%   phi     =   cs*phibar;
%   phibar  =   sn*phibar;
% 
%   t1      =   phi  /rho;
%   t2      = - theta/rho;
% 
%   x       = x      + t1*w;
%   w       = v      + t2*w;

end