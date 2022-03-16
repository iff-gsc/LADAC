function SubsysHandle = createRateLimiterWOSat( simulinkPth, RLBlkName, xyPos,...
    gainStrEntry, risingSlewRate, fallingSlewRate, setColor )
%CREATERATELIMITER   This function creates a rate limiter replacement using a
%   PT-1 elemrnt with a discrete time integrator as an additional state.
%   This rate limiter replacement was created as an approximation for model
%   linearizations. Furthermore, it uses a continuous replacement for the
%   saturation block.
%
% Syntax:  SubsysHandle = createRateLimiter( simulinkPth, RLBlkName, xyPos,...
%               gainStrEntry, risingSlewRate, fallingSlewRate, setColor )
%
% Required Inputs:
%      simulinkPth:         String that specifies in which path the rate limiter
%                           must be created.
%      RLBlkName:           Rate limiter replacement blockname.
%      xyPos:               Array with two entries which sets the position of
%                           the block i.e. [ xPos, yPos ].
%      gainStrEntry:        String that spacifies the gain of the PT-1. Use
%                           something like 10 times the samplerate. The string
%                           could also contain a number like '50'.
%      risingSlewRate:      double specifies the rising slew rate.
%      fallingSlewRate:     double specifies the falling slew rate.
%      setColor:            Boolean value which colors the new in- and output
%                           block backrounds with green for input and red for
%                           outputs, respectively, if true is given.
%
% Output Arguments:
%      SubsysHandle:       The handle of the created rate limiter replacement
%                           block.
%
% Example:
%    rateLimiterRplce = createRateLimiter( 'RateLimiterTest/OriginalRateLimiter',...
%       'RLReplacement', [ 420,   190 ], '1 / ( 10 * sampletime )', .1, -8., true )
%
% See also: RateLimiterReplacer,  createSaturation

% Disclamer:
%   SPDX-License-Identifier: GPL-2.0-only
% 
%   Copyright (C) 2021 Alexander Kuzolap
%   Copyright (C) 2022 TU Braunschweig, Institute of Flight Guidance
% *************************************************************************

    %% Debug settings
%     simulinkPth = 'RateLimiterTest/OriginalRateLimiter';
%     RLBlkName = 'Bla';
%     xyPos = [ 420,   190 ];
%     setColor = true;
%     gainStrEntry = '1 / ( 10 * dt.Regler )';
%     risingSlewRate = .1;
%     fallingSlewRate = -3.;
    % Rate limiter replacement size
    xWidth = 170;
    yWidth = 30;

    %% Subsystem creation
    % Create the subsystem block
    SubsysFullPth = [ simulinkPth, '/', RLBlkName ];
    SubsysHandle = add_block( 'built-in/Subsystem', SubsysFullPth, ...
        'MAKENAMEUNIQUE', 'ON' );
    % Get the chosen unique name
    SubsysFullPth = [ simulinkPth, '/', get_param( SubsysHandle, 'Name' ) ];

    % Set the block position
    SubsysBlkPos = [ xyPos(1),   xyPos(2),   xyPos(1) + xWidth,   xyPos(2 ) +  yWidth ];
    set_param( SubsysHandle, 'Position', SubsysBlkPos );

    %% Create elements inside Subsystem
    % Create the in and output ports
    inPortName = 'RateLimiter Input';
    outPortName = 'RateLimiter Output';
    inPortNamePth = [ SubsysFullPth, '/', inPortName ];
    outPortNamePth = [ SubsysFullPth, '/', outPortName ];

    % Add the blocks with the correct position
    inputHandle = add_block( 'simulink/Sources/In1', inPortNamePth,...
        'MAKENAMEUNIQUE', 'ON' );
    outputHandle = add_block( 'simulink/Sinks/Out1', outPortNamePth,...
        'MAKENAMEUNIQUE', 'ON' );

    % Set the color
    if setColor
        set_param( inputHandle, 'BackgroundColor', 'green' );
        set_param( outputHandle, 'BackgroundColor', 'red' );
    end

    % Get in- and outputport handles of the in and outputs blocks
    % e.g. subblock input got an output port handle
    inPortHandle = get_param( inputHandle, 'PortHandles' );
    inPortHandleOutport = inPortHandle.Outport;

    outPortHandle = get_param( outputHandle, 'PortHandles' );
    %outPortHandleInport = outPortHandle.Inport;

    % Set the in and output block positions
    %inPortPos = get_param( inputHandle, 'Position' );

    % Create a sum block
    sumNamePth = [ SubsysFullPth, '/', 'Sum' ];
    sumHandle = add_block( 'built-in/Sum', sumNamePth,...
        'MAKENAMEUNIQUE', 'ON', 'IconShape', 'round', 'Inputs', '|+-' );
    sumPos = get_param( sumHandle, 'Position' );
    sumPos = [ sumPos(1), sumPos(2), sumPos(1) + 20 sumPos(2) + 20 ];
    set_param( sumHandle, 'Position', sumPos );
    % Set the sum block position
    sumPos = getHorizPosOf( inPortHandleOutport, sumHandle, +30 );
    set_param( sumHandle, 'Position', sumPos );
    % Get the sum porthandles and in-/outports
    sumPortHandle = get_param( sumHandle, 'PortHandles' );

    % Create the integrator, saturation and the gain
    gainNamePth = [ SubsysFullPth, '/', 'Gain' ];
    gainHandle = add_block( 'built-in/Gain', gainNamePth,...
        'MAKENAMEUNIQUE', 'ON' );
    % Set the gain position
    gainPos = getHorizPosOf( sumPortHandle.Outport, sumHandle, +30 );
    set_param( gainHandle, 'Position', gainPos );
    set_param( gainHandle, 'Gain', gainStrEntry );
    % Get the gain port handles
    gainPortHandle = get_param( gainHandle, 'PortHandles' );

    % Create the discrete time integrator
    integratorNamePth = [ SubsysFullPth, '/', 'Integrator' ];
    integratorHandle = add_block( 'built-in/DiscreteIntegrator', integratorNamePth,...
        'MAKENAMEUNIQUE', 'ON' );
    % Set the position of the integrator
    integratorPos = getHorizPosOf( gainPortHandle.Outport, integratorHandle, ...
        +30 + 100 + 30 );
    set_param( integratorHandle, 'Position', integratorPos );
    % Get the integrator port handles
    integratorPortHandle = get_param( integratorHandle, 'PortHandles' );
    % inherit the sampletime
    set_param( integratorHandle, 'SampleTime', '-1' );
    % Create the saturation block
    xyPos = get_param( gainPortHandle.Outport, 'Position' ) + [ 30, -20 ];

    saturationHandle = createSaturation( SubsysFullPth, 'Saturation', xyPos,...
        gainPortHandle.Outport, integratorPortHandle.Inport, ...
        fallingSlewRate, risingSlewRate );
    % Get the integrator port handles
%     saturationPortHandle = get_param( saturationHandle, 'PortHandles' );

    % Set the position of the output port
    outpuPos = getHorizPosOf( integratorPortHandle.Outport, outputHandle, 30 );
    set_param( outputHandle, 'Position', outpuPos );

    %% Connect the subsystem
    % Input -- sum
    add_line( SubsysFullPth, inPortHandle.Outport, sumPortHandle.Inport(1),...
            'autorouting','on');
    % Integrator -- sum
    add_line( SubsysFullPth, integratorPortHandle.Outport, sumPortHandle.Inport(2),...
            'autorouting','on');
    % sum -- gain
    add_line( SubsysFullPth, sumPortHandle.Outport, gainPortHandle.Inport,...
            'autorouting','on');
    % Integrator -- Output
    add_line( SubsysFullPth, integratorPortHandle.Outport, outPortHandle.Inport,...
            'autorouting','on');
end
