function fuse = simpleFuselageInit( )
% simpleFuselageInit defines the simple fuselage struct. 
%   This is a simple generic model. The model is documented
%   in [1, p. 29f] and it is based on measured data presented in 
%   [2, p 248].
% 
% Outputs:
%   fuse            simple fuselage struct as defined by this function
% 
% Syntax:
%   fuse = sipleFuselageInit();
% 
% Literature:
%   [1] Beyer, Y. (2017): Flight Control Design and Simulation of a Tandem
%       Tilt Wing RPAS, master thesis, TU Braunschweig.
%   [2] Schlichting, H., & Truckenbrodt, E. A. (1969). Aerodynamik des
%       Flugzeuges: Zweiter Band: Aerodynamik des Tragfl�gels (Teil II),
%       des Rumpfes, der Fl�gel-Rumpf-Anordnungen und der Leitwerke. 2nd
%       edition. Springer-Verlag.
% 
% See also:
%   simpleFuselageLoadParams, simpleFuselageCreate
% 

% Disclamer:
%   SPDX-License-Identifier: GPL-2.0-only
% 
%   Copyright (C) 2020-2022 Yannic Beyer
%   Copyright (C) 2022 TU Braunschweig, Institute of Flight Guidance
% *************************************************************************

% yawing moment coefficient slope, 1/rad
fuse.dCn_dbeta = 0;
% lift curve slope, 1/rad
fuse.dCL_dalpha0 = 0;
% maximum lateral force coefficient, -
fuse.C_Qmax = 0;
% lateral force slope, 1/rad
fuse.dCQ_dbeta0 = 0;
% pitching moment coefficient slope, 1/rad
fuse.dCm_dalpha = 0;
% maximum lift coefficient, -
fuse.C_Lmax = 0;
% maximum drag coefficient (alpha=45deg), -
fuse.C_Dmax = 0;
% minimum drag coefficient (alpha=0), -
fuse.C_Dmin = 0;
% volume, m^3
fuse.V = 0;

end