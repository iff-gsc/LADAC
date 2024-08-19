function fuse = simpleFuselageLoadParams( fuse, filename, varargin )
% simpleFuselageLoadParams loads parameters from a parameters file
% to a simple fuselage struct.
% 
% Inputs:
%   fuse            simple fuselage struct, see simpleFuselageInit
%   filename        parameters filename (string), e.g.
%                   'simpleFuselage_params_default'
%   V               volume of the fuselage (scalar), in m^3
% 
% Outputs:
%   fuse            simple fuselage struct, see simpleFuselageInit
% 
% See also:
%   simpleFuselageInit, simpleFuselageCl
% 

% Disclaimer:
%   SPDX-License-Identifier: GPL-3.0-only
% 
%   Copyright (C) 2020-2022 Yannic Beyer
%   Copyright (C) 2022 TU Braunschweig, Institute of Flight Guidance
% *************************************************************************

if ~isempty(varargin)
    fuse.V(:) = varargin{1};
end

run( filename );

fuse.dCn_dbeta(:) 	= dCn_dbeta;
fuse.dCL_dalpha0(:)	= dCL_dalpha0;
fuse.C_Qmax(:)  	= C_Qmax;
fuse.dCQ_dbeta0(:) 	= dCQ_dbeta0;
fuse.dCm_dalpha(:)	= dCm_dalpha;
fuse.C_Lmax(:)    	= C_Lmax;
fuse.C_Dmax(:)    	= C_Dmax;
fuse.C_Dmin(:)    	= C_Dmin;

end