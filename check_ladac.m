% Disclamer:
%   SPDX-License-Identifier: GPL-3.0-only
% 
%   Copyright (C) 2020-2022 Fabian Guecker
%   Copyright (C) 2022 TU Braunschweig, Institute of Flight Guidance
% *************************************************************************

% Change to this folder
this_file_path = fileparts(mfilename('fullpath'));
cd(this_file_path);

% Add subfolders to path
addpath(genpath(this_file_path));

% Run all LADAC tests
tic;
rt = table(runtests(pwd,'IncludeSubfolders',true))
toc;

