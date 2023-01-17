function BusBlkHandle = createIntegratorOutBus( simulinkMdl, ...
    outputBusName, blkToBeReplaced, setColor )
%createIntegratorOutBus   Creates an integrator measurement bus for a given block
%   This function creates an integrator measurement bus for the block specified as
%   simulinkMdl in the function input. This function searches for all discrete
%   integrators and produces an output signal up to the highest level of the given
%   model, where all signals are connected to a bus. The bus name is specified in
%   the parameter outputBusName. If a VOLKSWAGEN coloring of the output blocks is
%   required, set setColor to true or 1. The created busblock handle is
%   returned.
%
% Syntax:  BusBlkHandle = createIntegratorOutBus( simulinkMdl, ...
%   outputBusName, blkToBeReplaced, setColor )
%
% Inputs:
% 	simulinkMdl         Sets the model where a integrator measurement bus should
%                       be created.
% 	outputBusName       This string defines the name for the created bus name,
%                       the signal name that is connected to the created bus and
%                       the ouput bus name. For the ouput bus name an 'Out' is
%                       appended to avoid same name on one level.
%   blkToBeReplaced     Simulink 'BlockType' that shall be replaced,
%                       usually 'DiscreteIntegrator' or 'Integrator'.
% 	setColor            Sets the ouput block color to the VOLKSWAGEN required
%                       coloring.
%
% Outputs:
%    BusBlkHandle       Returns the handle for the created bus block.
%
% Example:
%    BusBlkHandle = createIntegratorOutBus(...
%       'RcPlt_Mini_Norbert/Referenz_Fzg_Modell', 'FahrzeugMesurementBus', 1 );
%    createIntegratorOutBus(...
%       'HIL/motorModel', 'IntegratorMesurementBus', 0 );
%
% See also: getBusObjFromSelection,  createBusObjMFile,  linerator

% Disclamer:
%   SPDX-License-Identifier: GPL-3.0-only
% 
%   Copyright (C) 2021 Alexander Kuzolap
%   Copyright (C) 2022 TU Braunschweig, Institute of Flight Guidance
% *************************************************************************

% get_param( BusBlkHandle, 'ObjectParameters')

    % Get all ingerators
    IntObjects = find_system( simulinkMdl, ...
        'BlockType', blkToBeReplaced );

    % Create a bus for the output
    BusBlkName = [ simulinkMdl, '/', outputBusName ];
    BusBlkHandle = add_block( 'simulink/Signal Routing/Bus Creator', BusBlkName, 'MAKENAMEUNIQUE', 'ON' );
    set_param( BusBlkHandle, 'Inputs', int2str( length( IntObjects ) ) );
    BusPortHandles = get_param( BusBlkHandle, 'PortHandles' );
    [ xMax, yMax ] = maxPosBlk( simulinkMdl );
    pos = getPosOfCoord( [ xMax, yMax ], BusBlkHandle, 10, 50 );
    set_param( BusBlkHandle, 'Position', pos );
    addBottomSize( BusBlkHandle, 20 * length( IntObjects ) );
%     i = 1;
    %setColor = true;
    %simulinkMdl = 'Norbert_ADTF_newPlnr_Portimao_2018_Controller_RAW/04 FahrdynamikRegler';

    for i = 1:length( IntObjects )
        % Reset view that all fit
        set_param( gcs, 'Zoomfactor','fit to view' );

        % Get the block path
        blkPath = get_param( IntObjects{i}, 'Parent' );
        % Create block names
        outPortName = [ '_', get_param(IntObjects{i}, 'Name'),'_in' ];
        % Convert potential output and later bus (field name) into one that is valid
        outPortName = regexprep( outPortName, '\/', '|');
        outPortName = matlab.lang.makeValidName( outPortName );
        outPortBlk = [ blkPath,'/', outPortName ];

        % Get the integrator port handles
        intPortHandles = get_param(IntObjects{i}, 'PortHandles');

        % Add the blocks with the correct position
        outPortBlkHandle = add_block( 'simulink/Sinks/Out1', outPortBlk, 'MAKENAMEUNIQUE', 'ON' );

        % Give some warnings
        outPortNameNew = get_param( outPortBlkHandle, 'Name');
        if not( isequal( outPortName, outPortNameNew ) )
            warning( [ sprintf('A output block %s already exists.\n', outPortName), ...
                 sprintf('The output port was renamed to %s.', outPortNameNew) ] );
             outPortName = outPortNameNew;
        end

        % Set the block position
        outPortBlkHandlePos = getPosOf( intPortHandles.Outport, outPortBlkHandle, 10, -20 );

        set_param( outPortBlkHandle, 'Position', outPortBlkHandlePos );

        if setColor
            set_param(  outPortBlkHandle, 'BackgroundColor', 'red' );
        end

        % Get port handles of added in out ports
        outPortHandle = get_param( outPortBlkHandle, 'PortHandles' );

        % Reconnect
        add_line(blkPath, intPortHandles.Outport, outPortHandle.Inport, 'autorouting','on');

        blkPathParent = get_param( blkPath, 'Parent');

        % If it is the last path connect the outgoing bus
        if isequal( blkPathParent, simulinkMdl )

            % Get the port numbers of the ports
            outPortBlkHandlePortNum = str2num( get_param( outPortBlkHandle, 'Port') );

            % Get the destination and source port handles
            blkPathPortHandle = get_param( blkPath, 'PortHandles' );
            srcPort = blkPathPortHandle.Outport( outPortBlkHandlePortNum );
            lineHandle = add_line(blkPathParent, srcPort, BusPortHandles.Inport( i ), 'autorouting','on');
            set_param( lineHandle, 'Name', outPortName );
            addBottomSize( blkPath, 42 );
        end

        while ~isequal( blkPathParent, simulinkMdl )

            % Switch to the system and fit zoom
            open_system( blkPathParent );
            set_param(gcs,'Zoomfactor','fit to view');

            % Resize the subblock
            addBottomSize( blkPath, 42 );

            % Get the port numbers of the ports
            outPortBlkHandlePortNum = str2num( get_param( outPortBlkHandle, 'Port') );

            % Get the destination and source port handles
            blkPathPortHandle = get_param( blkPath, 'PortHandles' );
            srcPort = blkPathPortHandle.Outport( outPortBlkHandlePortNum );

            % Create the parent in out blocks
            outPortBlkParent = [ blkPathParent, '/', outPortName ];
            outPortBlkHandleParent = add_block( 'simulink/Sinks/Out1', outPortBlkParent, 'MAKENAMEUNIQUE', 'ON' );

            % Give some warnings
            outPortNameNew = get_param( outPortBlkHandleParent, 'Name');
            if not( isequal( outPortName, outPortNameNew ) )
                warning( [ sprintf('A output block %s already exists.\n', outPortName), ...
                     sprintf('The output port was renamed to %s.', outPortNameNew) ] );
                 outPortName = outPortNameNew;
            end

            if setColor
                set_param(  outPortBlkHandleParent, 'BackgroundColor', 'red' );
            end

            % Obtain the position for the in and out blocks
            outPortParentBlkPos = getHorizPosOf(srcPort, outPortBlkHandleParent, 10);

            % Set the obtained position of the in and out block
            set_param( outPortBlkHandleParent, 'Position', outPortParentBlkPos);

            % Get the in and output handles
            outPortHandle = get_param( outPortBlkHandleParent, 'PortHandles' );

            % Connect in and output to the subsystem
            add_line(blkPathParent, srcPort, outPortHandle.Inport, 'autorouting','on');

            % Get the new elements
            blkPath = blkPathParent;
            blkPathParent = get_param( blkPath, 'Parent');
            outPortBlkHandle = outPortBlkHandleParent;

            % If it is the last path connect the outgoing bus
            if isequal( blkPathParent, simulinkMdl )

                % Get the port numbers of the ports
                outPortBlkHandlePortNum = str2num( get_param( outPortBlkHandle, 'Port') );

                % Get the destination and source port handles
                blkPathPortHandle = get_param( blkPath, 'PortHandles' );
                srcPort = blkPathPortHandle.Outport( outPortBlkHandlePortNum );
                lineHandle = add_line(blkPathParent, srcPort, BusPortHandles.Inport( i ), 'autorouting','on');
                set_param( lineHandle, 'Name', outPortName );
                addBottomSize( blkPath, 42 );
            end
        end
    end

    % Create ouput bus block
    outPortBusBlk = [ simulinkMdl, '/', outputBusName, 'Out' ];
    outPortBusBlkHandle = add_block( 'simulink/Sinks/Out1', outPortBusBlk, 'MAKENAMEUNIQUE', 'ON' );
    pos = getHorizPosOf( BusPortHandles.Outport, outPortBusBlkHandle, 50 );
    set_param( outPortBusBlkHandle, 'Position', pos );
    outPortBusPortHandle = get_param( outPortBusBlkHandle, 'PortHandles' );
    lineHandle = add_line( simulinkMdl, BusPortHandles.Outport, outPortBusPortHandle.Inport );
    set_param( lineHandle, 'Name', outputBusName );
    if setColor
        set_param(  outPortBusBlkHandle, 'BackgroundColor', 'red' );
    end
end
