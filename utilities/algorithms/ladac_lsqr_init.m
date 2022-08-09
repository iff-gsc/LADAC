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
u    = b;        
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

end