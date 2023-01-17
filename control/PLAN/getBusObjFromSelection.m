function busInfo = getBusObjFromSelection( varargin )
%getBusObjFromSelection   Creates a M-File and a workspace variable for the selected Bus
%   This function creates a M-File and a workspace variable for the
%   selected Bus. The M-File contains the bus object definition that creates an
%   Bus object given as a variable. If one of them exist, i.e. variable or file,
%   it will be deleted first and then recreated.
%
% Syntax:  getBusObjFromSelection( varargin )
%
% Required Inputs:
%      Simulink object:     You have to select a Simulkink Bus creator
%                           inside Simulink.
%
% Optional Inputs:
%      path:      Path given as a string. If this Input is given and the
%                 directory exists, the file will be created in the given
%                 directory.
%
% Output Arguments:
%      busInfo:   A structure array containing bus information for the
%                 specified blocks. Each element of the structure array
%                 corresponds to one of the specified blocks and contains
%                 the following fields:
%                 block:   Handle of the block
%                 busName: Name of the bus object associated with the block
%
% Example:
%    busInfo = createBusObjMFile()
%    busInfo = createBusObjMFile( 'Configurations/Simulation' )
%
% See also: getBusObjFromSelection,  createIntegratorOutBus

% Disclamer:
%   SPDX-License-Identifier: GPL-3.0-only
% 
%   Copyright (C) 2021 Alexander Kuzolap
%   Copyright (C) 2022 TU Braunschweig, Institute of Flight Guidance
% *************************************************************************

    BusBlkName = get_param( gcbh, 'Name');
    if ~strcmp( get_param( gcbh, 'BlockType' ), 'BusCreator' )
        error( ' Error selected Object %s is not a BusCreator.\n', BusBlkName );
    end
    if ~isequal( nargin, 0)
        busInfo = createBusObjMFile( BusBlkHandle, varargin{1} );
    else
        busInfo = createBusObjMFile( gcbh );
    end
end
