function [md2bm, span_load_station] = structureGetBendingMomentTrafoAt( structure_red, structure, eta )
% structureGetBendingMomentTrafoAt compute transformation matrix that maps
% modal dispacement vector to bending moment at specified locations
% 
% Inputs:
%   structure_red           reduced-order structure struct (see
%                           structureGetReduced)
%   structure               structure struct (see
%                           structureCreateFromNastran)
%   eta                     dimensionless spanwise locations where the
%                           bending moment should be evaluated (1xN array)
% 
% Outputs:
%   md2bm                   transformations matrix that maps the modal
%                           displacement vector to the bending moment at
%                           each eta (NxM array, where M is the number of
%                           modal displacements of structure_red); it is
%                           the matrix product of [1], eq. (11)
%   span_load_station       y coordinates in aircraft frame of the load
%                           stations defined by eta (1xN array)
% 
% See also:
%   structureGetReduced, structureCreateFromNastran
% 
% Literature:
%   [1] Moulin, B., & Karpel, M. (2007). Gust loads alleviation using 
%       special control surfaces. Journal of Aircraft, 44(1), 17-25.
%       https://arc.aiaa.org/doi/pdf/10.2514/1.19876
% 

% Disclaimer:
%   SPDX-License-Identifier: GPL-3.0-only
% 
%   Copyright (C) 2020-2022 Yannic Beyer
%   Copyright (C) 2022 TU Braunschweig, Institute of Flight Guidance
% *************************************************************************

% matrix product of [1], eq. (11)
[md2bm,span_load_station] = structureGetCutLoadTrafoAt( ...
                                structure_red, structure, eta, ...
                                'Mx', 'reverse_right' );
                                    
end
