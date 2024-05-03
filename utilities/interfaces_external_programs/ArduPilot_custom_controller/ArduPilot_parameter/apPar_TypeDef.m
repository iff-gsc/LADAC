classdef apPar_TypeDef
% APPAR_TYPEDEF is a small data container to describe a type.
% 
% See also:
%   APPAR_GETSTRUCTSFROMHEADERFILE

% Disclaimer:
%   SPDX-License-Identifier: GPL-3.0-only
% 
%   Copyright (C) 2024 Jonas Withelm
%   Copyright (C) 2024 TU Braunschweig, Institute of Flight Guidance
% *************************************************************************

properties (Access = public)
    type
    size
end

methods
    function obj = apPar_TypeDef(type, size)
        obj.type = type;
        obj.size = size;
    end
end

end
