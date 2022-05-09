function addPathTiGL(TiGL_version)
% addPathTiGL adds the Matlab functions from TiGL and TiXI to the Matlab
%   path.
%   This function only works if TiGL and TiXI are installed:
%       Install TiGL: https://github.com/DLR-SC/tigl
%       Install TiXI: https://github.com/DLR-SC/tixi
%   If there are multiple installations of TiXI, the newest version will be
%   added to the Matlab path (this should not cause problems).
% 
% Inputs:
%   TiGL_version        Version of the to be added TiGL (string). The
%                       version should be equal to the name of the TiGL
%                       directory, e.g. '2.3.3' if the name of the TiGL
%                       directory is TiGL 2.3.3
% 
% Example:
%   addPathTiGL('3.0.0')
% 

% Disclamer:
%   SPDX-License-Identifier: GPL-2.0-only
% 
%   Copyright (C) 2020-2022 Yannic Beyer
%   Copyright (C) 2022 TU Braunschweig, Institute of Flight Guidance
% *************************************************************************

%% Set pathes (this can be modified)

% potential parent directorys of the TiGL and TiXI directory
% --> FEEL FREE TO ADD OTHER POTENTIAL DIRECTORIES
possible_paths = { ...
    'C:/Program Files'; ...
    'C:/Program Files (x86)'; ...
    '/usr/local/lib'; ...
    '/usr/local/share' ...
    };

% possible path to the matlab directory from the TiGL directory
% (This works for version 2 and 3)
TiGL_subfolder = { ...
            '/share/tigl/matlab'; ...
            '/share/tigl3/matlab' ...
            };
        
% possible path to the matlab directory from the TiXI directory
% (This works for version 2 and 3)
TiXI_subfolder = { ...
            '/share/tixi/matlab'; ...
            '/share/tixi3/matlab'; ...
            '/matlab' ...
            };

%% find TiGL and TiXI and add to path (this should not be modified)

num_path = length( possible_paths );
TiGL_folder_names = { ...
            'tigl'; ...
            ['TIGL ', TiGL_version] ...
            };

% init variables
TiGL_path_found = false;
TiGL_matlab_path_found = false;
TiXI_path_found = false;
TiXI_matlab_path_found = false;

for i = 1:num_path
    
    % find the TIGL directory
    for ii = 1:length(TiGL_folder_names)
        TiGL_path = [ possible_paths{i}, '/', TiGL_folder_names{ii} ];
        if exist( TiGL_path, 'dir' )
            TiGL_path_found = true;
            for j = 1:length(TiGL_subfolder)
                if exist( [ TiGL_path, TiGL_subfolder{j} ], 'dir' ) 
                    % add to path TIGL subdirectory
                    addpath( [ TiGL_path, TiGL_subfolder{j} ] );
                    TiGL_matlab_path_found = true;
                end
            end
        end
    end
end

if TiGL_matlab_path_found
    % find TIXI directory
    list_dir = dir( possible_paths{i} );
    len_list_dir = length(list_dir);
    % init variable
    is_TiGL_dir = false(len_list_dir,1);
    for k = 1:len_list_dir
        if list_dir(k).isdir
            if contains( list_dir(k).name, 'TIXI' ) || contains( list_dir(k).name, 'tixi' )
                is_TiGL_dir(k) = true;
            end
        end
    end
    idx_TIXI_dir_vec = find(is_TiGL_dir == true);
    if length(idx_TIXI_dir_vec) >= 1
        % choose newest TiXI version ( --> end )
        idx_TIXI_dir = idx_TIXI_dir_vec(1);
        TiXI_folder_name = list_dir(idx_TIXI_dir).name;
        TiXI_path_found = true;
        TiXI_path = [ possible_paths{i}, '/', TiXI_folder_name ];
        for l = 1:length(TiXI_subfolder)
            if exist( [ TiXI_path, TiXI_subfolder{l} ], 'dir' ) 
                % add to path TIXI subdirectory
                addpath( [ TiXI_path, TiXI_subfolder{l} ] );
                TiXI_matlab_path_found = true;
            end 
        end
    end
end

% Throw error if the TiGL directory was not found.
if ~TiGL_path_found
    error( [ 'TiGL path was not found. ', ...
        'Please check that TiGL is installed', ...
        'and that you specified the correct version. ', ...
        'If everything is correct, consider to modify this function,', ...
        'e.g. by adding the parent directory to the cell array possible_paths.' ] );
% Throw error if the path to the matlab subdirectory of TiGL was not found.
elseif ~TiGL_matlab_path_found
    error( [ 'matlab subdirectory in TiGL installation folder was not found. ', ...
        'Please check your TiGL installation or consider adding the path '...
        'to the subdirectory to the variable TiGL_subfolder in this function.'] );
end
% Throw error if the TiXI directory was not found.
if ~TiXI_path_found
    error( [ 'TiXI path was not found.', ...
        'Please check that TiXI is installed.', ...
        'If everything is correct, consider to modify this function,', ...
        'e.g. by adding the parent directory to the cell array possible_paths' ] );
% Throw error if the path to the matlab subdirectory of TiXI was not found.
elseif ~TiXI_matlab_path_found
    error( [ 'matlab subdirectory in TiXI installation folder was not found. ', ...
        'Please check your TiXI installation or consider adding the path '...
        'to the subdirectory to the variable TiXI_subfolder in this function.'] );
end

end