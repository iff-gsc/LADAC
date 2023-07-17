function K_y = ssPlaceY(A,B,C,p,w)
% ssPlaceY compute the output feedback gain of state-space model that 
% approximately yields the closed-loop desired eigenvalues (pole placement)
%   Use the weighting w to manipulate the feedback gain.
%   Note that the obtained system is not necessarily stable. In case of
%   undesired results, try to place the poles closer to the poles of the
%   open-loop system.
% 
% Syntax:
%   K_y = ssPlaceY( A, B, C, p )
%   K_y = ssPlaceY( A, B, C, p, w )
% 
% Inputs:
%   A               system matrix (NxN array)
%   B               input matrix (Nx1 array)
%   C               output matrix (MxN array)
%   p               desired poles (1xN array)
%   w               weighting of poles to be approximated (1xN array)   
% 
% Outputs:
%   K_y             output feedback gain (1xN array)
% 
% Example:
%   A = rand(4,4);
%   b = rand(4,1);
%   C = [eye(3),zeros(3,1)];
%   p = [-1,-2+2i,-2-2i,-3];
%   w = [1,1,1,1];
%   ky = ssPlaceY( A, b, C, p );
%   figure, hold on
%   h1=plot(real(eig(A)),imag(eig(A)),'x')
%   h2=plot(real(p),imag(p),'+')
%   h3=plot(real(eig(A-b*ky*C)),imag(eig(A-b*ky*C)),'o')
%   axis equal, sgrid
%   legend([h1,h2,h3],'open loop','desired','output feedback')
% 
% See also:
%   ssPlace
% 
% Literature:
%   [1] Lunze, J. (2016). Regelungstechnik 2. Mehrgroessensysteme, Digitale
%       Regelung. Springer-Verlag Berlin Heidelberg.
% 

% Disclamer:
%   SPDX-License-Identifier: GPL-3.0-only
% 
%   Copyright (C) 2023 Yannic Beyer
%   Copyright (C) 2023 TU Braunschweig, Institute of Flight Guidance
% *************************************************************************

% full state feedback gain
K = ssPlace(A,B,p);

% eigenvectors of system with full state feedback
[V,D] = eig(A-B*K);
p_v = diag(D);

% sort eigenvector so that they match the order of the specified poles p
idx = zeros(1,length(p));
for i = 1:length(p)
    [~,idx(i)] = min(vecnorm(p(i)-p_v,2,2));
    p_v(idx(i)) = max(p_v) + 1e10;
end
Vs = V(:,idx);

if nargin == 4
    W = eye(length(p));
elseif nargin == 5
    W = diag(w);
end

% [1], Eq. (6.48)
K_y_complex = (K*Vs*W) * pinv(C*Vs*W);

% If imaginary part is not zero, something went wrong.
if any( abs(imag(K_y_complex)) > 1e-8 )
    error(['Invalid specification of poles. ', ...
        'Check if the weighting of conjugate complex poles is equal.'])
end

K_y = real( K_y_complex );

end