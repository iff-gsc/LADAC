function busInfo = createBusObjMFile( BusBlkHandle, varargin )
%createBusObjMFile   Creates a M-File and a workspace variable for the given Bus
%   This function creates a M-File and a workspace variable for the given
%   Bus. The M-File contains the bus object definition that creates an Bus
%   object given as a variable. If one of them exist, i.e. variable or file, it
%   will be deleted first and then recreated.
%
% Syntax:  busInfo = createBusObjMFile( BusBlkHandle, varargin )
%
% Required Inputs:
%      model:     Name or handle of model
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
%    busInfo = createBusObjMFile( BusBlkHandle )
%    busInfo = createBusObjMFile( BusBlkHandle, 'Configurations/Simulation' )
%
% See also: getBusObjFromSelection,  createIntegratorOutBus

% Disclamer:
%   SPDX-License-Identifier: GPL-3.0-only
% 
%   Copyright (C) 2021 Alexander Kuzolap
%   Copyright (C) 2022 TU Braunschweig, Institute of Flight Guidance
% *************************************************************************

    BusBlkName = get_param( BusBlkHandle, 'Name');
%     if exist( BusBlkName, 'var')
    fprintf(' Workspace variable %s found.\n clear %s\n', ...
        BusBlkName, BusBlkName );
    evalin( 'base', [ 'clear ', BusBlkName ] );
%     clear BusBlkName;
%     end
    BusFile = [ BusBlkName, '.m' ];
    if exist( BusFile, 'file') == 2
        fprintf(' Deleting Bus file: %s\n', BusFile );
        % File exists.
        BusFilePth = which( BusFile );
        delete(BusFilePth);
    end
    if ~isequal( nargin, 1)
        if isequal( exist(varargin{1}, 'dir' ), 7 )
            BusBlkName = fullfile( varargin{1}, BusBlkName );
        else
            error(' ERROR folder %s does not exist!\n', varargin{1} );
        end
    end

    busInfo = Simulink.Bus.createObject( bdroot, BusBlkHandle, BusBlkName );
    % If you only need to create the variable use
    %busInfo = Simulink.Bus.createObject(bdroot, gcbh)
    fprintf(' Creating Bus file: %s\n', BusFile );
end
