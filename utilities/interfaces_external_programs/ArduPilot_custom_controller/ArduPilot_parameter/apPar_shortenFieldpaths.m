function [fldpaths_ret, idxs_ret] = apPar_shortenFieldpaths(fldpaths, max_lengths)
% APPAR_SHORTENFIELDPATHS shortens the fieldpaths of an unpacked nested
%   struct to meet the provided maximum lengths.
%   Name groups are taken into account and it is ensured that each group
%   has the same name after shortening.
% 
% Inputs:
%   fldpaths        Cell array with fieldpaths of an unpacked struct
%   max_lengths     Maximum lengths for fieldpaths, array of doubles
% 
% Outputs:
%   fldpaths_ret    Cell array with shortened fieldpaths
%   idxs_ret        Indices of fieldpaths that were too long,
%                   array of doubles

% Disclaimer:
%   SPDX-License-Identifier: GPL-3.0-only
% 
%   Copyright (C) 2024 Jonas Withelm
%   Copyright (C) 2024 TU Braunschweig, Institute of Flight Guidance
% *************************************************************************

% ToDo:
% - idxs_ret is unclear, it contains indices of fields that were to long,
%   but doesn't contain indices of all fields that were changed!

delimiter = '.';


[grps, ~, idxs, is_group] = apPar_getGroups(fldpaths, 1);

fldpaths_ret = {};
idxs_ret     = [];

idxs_long_glob = [];    

for idx_grp = 1:numel(idxs)
    
    grpidxs = idxs{idx_grp};
    
    % Handle group
    if is_group(idx_grp)
        
        % Extract group fields
        grpfldpaths    = fldpaths(grpidxs);
        grpmax_lengths = max_lengths(grpidxs);
        grpfld_lengths = cellfun(@(a) length(a), grpfldpaths);
        
        % Find fields of group which are too long
        grpidxs_long = find(grpfld_lengths > grpmax_lengths);
        
        
        % Handle too long fields
        if ~isempty(grpidxs_long)
            idxs_long_glob = grpidxs(grpidxs_long);
            
            % Extract too long fields
            grpflds_long        = grpfldpaths(grpidxs_long);
            grpmax_lengths_long = grpmax_lengths(grpidxs_long);
            
            % Shorten all fields which are too long
            [grpflds_long_short, nl] = cellfun(@(a, b) shortenFieldpath(a, b), grpflds_long, num2cell(grpmax_lengths_long), 'UniformOutput', false);
            nl = cell2mat(nl);
            
            % Find the highest nesting level that has been shortened
            hnl = min(nl);
            
            % If hnl == 1, the top nesting level (tnl) has been
            % shortened and the shortest top nesting level name must be
            % found
            if hnl == 1
                grpflds_sel  = grpflds_long_short(nl == 1);
                
                grpfldsparts = cellfun(@(a) strsplit(a, delimiter), grpflds_sel,  'UniformOutput', false);
                tnl_names    = cellfun(@(a) a{hnl},                 grpfldsparts, 'UniformOutput', false);
                tnl_sizes    = cellfun(@(a) length(a),              tnl_names);
                
                % Find shortest top nesting level name
                [~, idx_min] = min(tnl_sizes);
                
                % Set the group specifier to the shortest nesting level name
                group_spec   = tnl_names{idx_min};
            else
                group_spec = grps{idx_grp};
            end
            
            % Select next nesting level
            grpfldsparts    = cellfun(@(a) strsplit(a, delimiter),       grpfldpaths,  'UniformOutput', false);
            grpfldpaths_nnl = cellfun(@(a) strjoin(a(2:end), delimiter), grpfldsparts, 'UniformOutput', false);
            
            % Recursive search for groups at next nesting level
            grpfldpaths_ret = apPar_shortenFieldpaths(grpfldpaths_nnl, grpmax_lengths - (length(group_spec) + 1));
            
            grpfldpaths_ret = cellfun(@(a) [group_spec delimiter a], grpfldpaths_ret, 'UniformOutput', false);
        
        else
            % This group does not contain too long fields, just copy data
            grpfldpaths_ret = grpfldpaths;
        end
        
    % Handle single param
    else
        fldpath    = fldpaths{grpidxs};
        max_length = max_lengths(grpidxs);
        fldlength  = length(fldpath);
        if fldlength > max_length
            fldpath = shortenFieldpath(fldpath, max_length);
            idxs_long_glob = grpidxs;
        end
        grpfldpaths_ret = {fldpath};
    end
    
    fldpaths_ret = [fldpaths_ret; grpfldpaths_ret];
    idxs_ret     = [idxs_ret, idxs_long_glob];
    
end

end





%% LOCAL FUNCTIONS
function [fldpath_short, nesting_level] = shortenFieldpath(fldpath, max_len)
    
    delimiter = '.';
    
    fldparts = strsplit(fldpath, delimiter);
    fldsizes = cellfun(@(a) length(a), fldparts);
    
    len = sum(fldsizes);
    max_len = max_len - (numel(fldparts) - 1);
    
    overhang = len - max_len;
    
    
    % Soft Shortening: Remove vowels starting from the end
    for idx = numel(fldparts):-1:1
        fldpart = fldparts{idx};
        vowel_idxs = regexp(fldpart, '[aeiou]');
        num_vowels = length(vowel_idxs);
        
        if overhang > 0 && overhang < num_vowels
            idx_a = num_vowels - overhang + 1;
            vowel_idxs = vowel_idxs(idx_a:end);
            num_vowels = length(vowel_idxs);
        end
        
        fldparts{idx} = fldpart(setdiff(1:end,vowel_idxs));
        overhang = overhang - num_vowels;
        
        if overhang == 0
            break;
        end
    end
    nesting_level = idx;
    
    
    % Hard Shortening: Shorten to 3 letters per fldpart starting from the end
    if overhang > 0
        for idx = numel(fldparts):-1:1
            fldpart = fldparts{idx};
            
            % Remove everything except the first 3 letters per fldpart
            n_letters = 3;
            bf_idxs = (n_letters+1):length(fldpart);
            
            num_bf = length(bf_idxs);
            
            if overhang > 0 && overhang < num_bf
                idx_a = num_bf - overhang + 1;
                bf_idxs = bf_idxs(idx_a:end);
                num_bf = length(bf_idxs);
            end
            
            fldparts{idx} = fldpart(setdiff(1:end,bf_idxs));
            overhang = overhang - num_bf;
            
            if overhang == 0
                break;
            end
        end
    end
    if idx < nesting_level
        nesting_level = idx;
    end

    % Check if shortening was successful
    %{
    if overhang > 0
        err_msg = sprintf('Auto shortening of ''%s'' is not possible, fieldpath is too long!\n', name);
        hint = sprintf('Please have a look at the README for further details.');
        
        error('MATLAB:apPar_detectAndShortenGroups:fieldTooLong', ...
        '\n%s\n%s', err_msg, hint);
    end
    %}
    
    fldpath_short = strjoin(fldparts, delimiter);
    
end
