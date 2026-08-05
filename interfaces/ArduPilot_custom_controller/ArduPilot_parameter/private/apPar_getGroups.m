function [grps, idxs_bool, idxs, is_group] = apPar_getGroups(fldpaths, depth)
% APPAR_GETGROUPS

% Disclaimer:
%   SPDX-License-Identifier: GPL-3.0-only
% 
%   Copyright (C) 2024 Jonas Withelm
%   Copyright (C) 2024 TU Braunschweig, Institute of Flight Guidance
% *************************************************************************


delimiter = '.';


fldparts = cellfun(@(a) strsplit(a, delimiter), fldpaths, 'UniformOutput', false);

% extract tln up to depth
tlnparts = cell(size(fldparts));
for idx = 1:numel(fldparts)
    fldpart = fldparts{idx};
    if length(fldpart) >= depth
        tlnparts{idx} = fldpart(1:depth);
    else
        tlnparts{idx} = fldpart(1:end);
    end
end

tlns = cellfun(@(a) strjoin(a, delimiter), tlnparts, 'UniformOutput', false);

[C,~,ic] = unique(tlns, 'stable');

% struct output format
%{
idxs_bool = struct;
idxs      = struct;
is_group  = struct;
for idx = 1:numel(C)
    idxs_bool.(C{idx}) = ic'==idx;
    idxs.(C{idx})      = find(idxs_bool.(C{idx}));
    is_group.(C{idx})  = sum(idxs_bool.(C{idx})) > 1;
end
%}

% cell output format
grps      = cell(numel(C), 1);
idxs_bool = cell(numel(C), 1);
idxs      = cell(numel(C), 1);
is_group  = false(numel(C), 1);
for idx = 1:numel(C)
    grps(idx)      = C(idx);
    idxs_bool{idx} = ic'==idx;
    idxs{idx}      = find(idxs_bool{idx});
    is_group(idx)  = sum(idxs_bool{idx}) > 1;
end

end
