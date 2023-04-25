function zeta = wingGetDimLessSpanwiseLengthVector( vortex )
% getDimLessSpanwiseLengthVector computes the dimensionless spanwise length
%   vector zeta. Note that zeta is differently defined than in [1],
%   nomenclature or above eq. 12. Since G is normalized by the span instead
%   of the chord, zeta is multiplied with the span instead of the chord.
% 
% Inputs:
%   vortex              struct containing vgeometric vortex parameters as
%                       defined by setGeometry.m
% 
% Outputs:
%   zeta                dimensionless spanwise length vector (3xn)
% 
% Literature:
%   [1] Phillips, W. F., & Snyder, D. O. (2000). Modern adaption of
%       Prandtl's classic lifting-line theory. Jounal of Aircraft, 37(4),
%       662-670.
%   [2] Schlichting, H., & Truckenbrodt, E. A. (1969). Aerodynamik des
%       Flugzeuges: Zweiter Band: Aerodynamik des Tragfl�gels (Teil II),
%       des Rumpfes, der Fl�gel-Rumpf-Anordnungen und der Leitwerke. 2nd
%       edition. Springer-Verlag.
% 

% Disclamer:
%   SPDX-License-Identifier: GPL-3.0-only
% 
%   Copyright (C) 2020-2022 Yannic Beyer
%   Copyright (C) 2022 TU Braunschweig, Institute of Flight Guidance
% *************************************************************************

% compute segment span db so that db(i)*c(i)=dA(i)
db = wingGetSegmentSpan( vortex );
% compute spatial vector along the bound segment dl
dl = wingGetSpatialVectorAlongBoundSegment( vortex );
% compute chord in the middle of two trailing vortices
c = vortex.c(1:end-1) + diff(vortex.c)/2;
% span is used for normalization, any other length could be used (span is
% used in [2], chord is used in [1])
span = sum( db(1,:,1) );
% Compute dimensionless spanwise length vector zeta.
zeta = span * dl ./ repmat((db.*c),3,1);

end