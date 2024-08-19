function airfoil_map = wingAirfoilMapLoadSection( section )
% wingAirfoilMapLoadSection loads multiple profile aerodynamics maps into
% a struct.
% 
% Inputs:
%   section             Array of chars that are the names of .mat files in
%                       which the profile map is stored
% 
% Outputs:
%   airfoil_map         Struct that contains multiple profile aerodynamics
%                       maps (structs)
% 

% Disclaimer:
%   SPDX-License-Identifier: GPL-3.0-only
% 
%   Copyright (C) 2020-2022 Yannic Beyer
%   Copyright (C) 2021 Lucas Schreer
%   Copyright (C) 2022 TU Braunschweig, Institute of Flight Guidance
% *************************************************************************

% determine how many different profiles a wing contains
profile_name = unique(section,'rows');

airfoil_map = struct;

for i = 1:length(profile_name(:,1))

    % load and delete head layer of profile-struct
    datacell = struct2cell(load(strtrim(profile_name(i,:))));
    dataarray = [datacell{:}];

    % save profile-struct under its name in the output struct
    airfoil_map.(strtrim(profile_name(i,:))) = dataarray;
end

end

