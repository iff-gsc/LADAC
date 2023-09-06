function [prop_name_match,ind,scaling] = propMapMatch(prop_name)
% propMapMatch find propeller in APC propeller database that matches well
%   the specified propeller.
%   The matching is performed on the diameter to pitch ratio.
% 
% Syntax:
%   [prop_name_match,ind,scaling] = propMapMatch(prop_name)
% 
% Inputs:
%    prop_name              The specific propeller name in DxP with
%                           propeller diameter D (in inch) and propeller
%                           pitch (in inch), e.g. '30x10.5' (string).
% 
% Outputs:
%    prop_name_match        The name of the matched propeller type within
%                           the first column of DATA_APC (string).
%                           The matching is performed by looking at the
%                           diameter to pitch ratio.
%                           Use the following command to get all available
%                           names:
%                           name_list = propMapGetNameList();
%   ind                     The index of the matched propeller within
%                           propMapGetNameList();
%   scaling                 The scaling factor: D_spec/D_match
%                           with specified propeller diameter D_spec and
%                           matched propeller diameter D_match.
% 
% See also:
%   propMapScatterCreate, propMapFitCreate, propMapGridCreate

% Disclamer:
%   SPDX-License-Identifier: GPL-3.0-only
% 
%   Copyright (C) 2023 Yannic Beyer
%   Copyright (C) 2023 TU Braunschweig, Institute of Flight Guidance
% *************************************************************************

load('DATA_APC');
prop_names_db = DATA_APC(:,1);
prop_name_split = strsplit(prop_name,'x');
d = str2num(prop_name_split{1});
p = str2num(prop_name_split{2});
d_p = d/p;
prop_name_list = propMapGetNameList();
prop_score = zeros(length(prop_name_list),1);
for i = 1:length(prop_score)
    name_db_split = strsplit(prop_name_list{i},'x');
    d_db = str2num(name_db_split{1});
    p_db = str2num(name_db_split{2});
    if isempty(d_db) || isempty(p_db)
        prop_score(i) = 0;
    else
        d_p_db = d_db/p_db;
        prop_score(i) = ...
            0.9 * min(d_p,d_p_db)/max(d_p,d_p_db) ...
            + 0.1 * min(d,d_db)/max(d,d_db);
    end
end
[score,idx] = sort(prop_score,'descend');
prop_name_match = prop_name_list{idx(1)};
prop_name_split = strsplit(prop_name_match,'x');
d_db = str2num(prop_name_split{1});
ind = find(strcmp(prop_names_db,prop_name_match));
scaling = d/d_db;
end