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

Anorm  = zeros(1, superiorfloat(b));             

% Set up the first vectors u and v for the bidiagonalization.
% These satisfy  beta*u = b,  alfa*v = A'u.

x    = zeros(n,1, superiorfloat(b));
u    = b;        
v    = zeros(n,1, superiorfloat(b));
w    = zeros(n,1, superiorfloat(b));
alfa = zeros(1, superiorfloat(b));             
beta = norm(u);

rhobar = zeros(1, superiorfloat(b));
phibar = zeros(1, superiorfloat(b));

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