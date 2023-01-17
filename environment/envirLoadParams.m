function envir = envirLoadParams( filename, varargin )

% Disclamer:
%   SPDX-License-Identifier: GPL-3.0-only
% 
%   Copyright (C) 2020-2022 Yannic Beyer
%   Copyright (C) 2022 TU Braunschweig, Institute of Flight Guidance
% *************************************************************************

if ~isempty(varargin)
    envirBusName = varargin{1};
    if length(varargin) > 1
        alt = varargin{2};
    else
        alt = 0;
    end
else
    envirBusName = 'envir';
    alt = 0;
end

run(filename);

% init bus
struct2slbus( envir, envirBusName );

end