function fuselage = simpleFuselageCreateFromCPACS( tiglHandle, fuse_UID )
% simpleFuselageCreateFromCPACS returns parameters for a simple fuselage
% aerodynamics model with typical values and the volume from CPACS.
% 
% Inputs:
%   tiglHandle      tiglHandle, see tiglOpenCPACSConfigurationTry
%   fuse_UID        UID of fuselage (string) as defined in CPACS
% 
% Outputs:
%   fuselage        parameters for the fuselage aerodynamics model (struct)
% 
% Literature:                   
%   [1] Schlichting, H., & Truckenbrodt, E. A. (1969). Aerodynamik des
%       Flugzeuges: Zweiter Band: Aerodynamik des Tragfluegels (Teil II),
%       des Rumpfes, der Fluegel-Rumpf-Anordnungen und der Leitwerke. 2nd
%       edition. Springer-Verlag.
% 
% See also: simpleFuselageCl, simpleFuselageCd, simpleFuselageCm
% 

% Disclaimer:
%   SPDX-License-Identifier: GPL-3.0-only
% 
%   Copyright (C) 2020-2022 Yannic Beyer
%   Copyright (C) 2022 TU Braunschweig, Institute of Flight Guidance
% *************************************************************************

% init
fuselage = simpleFuselageInit();

% get fuselage volume from CPACS
fuse_index = tiglFuselageGetIndex( tiglHandle, fuse_UID );
volume = tiglFuselageGetVolume( tiglHandle, fuse_index );

fuselage = simpleFuselageLoadParams( fuselage, 'simpleFuselage_params_default', volume );

% get fuselage length from CPACS and estimate reference point
fuse_num_sections = tiglFuselageGetSectionCount(tiglHandle,3);
points_right = zeros(3,fuse_num_sections);
points_left = zeros(3,fuse_num_sections);
for i=1:fuse_num_sections
    if i < fuse_num_sections
        eta = 0;
        idx = i;
    else
        eta = 1;
        idx = i-1;
    end
    [points_right(1,i),points_right(2,i),points_right(3,i)] = ...
        tiglFuselageGetPoint(tiglHandle,fuse_index,idx,eta,0);
    [points_left(1,i),points_left(2,i),points_left(3,i)] = ...
        tiglFuselageGetPoint(tiglHandle,fuse_index,idx,eta,0.5);
end
% fuselage center line (mean)
points = ( points_right + points_left ) / 2;
% reference point of fuselage (see [1], p. 248)
x_ref = 0.45*points(1,end);
fuselage.xyz_ref = [ ...
    - interp1(points(1,:),points(1,:),x_ref); ...
    interp1(points(1,:),points(2,:),x_ref); ...
    - interp1(points(1,:),points(3,:),x_ref) ...
    ];

end