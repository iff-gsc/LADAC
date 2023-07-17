function joystickCalibrate( joystickname )
% joystickCalibrate detects the required configuration parameters of your
%   joystick and saves them as a configuration file in the
%   LADAC/control/joystick/params folder.
%   Please follow the requests of the command window.
%   Try to continuously move the requested stick/pot/switch to assure that
%   it is detected.
%   Note that each step of the calibration routine takes a fixed amount of
%   time regardless of what you are doing.
%   If you want to stop the calibration, just press "Cntrl+C" in the
%   command window.
%   Information about the calibration file:
%   The order of channels (order of array elements) is defined similar to
%   Taranis Q X7 [1].
%   The index vector (ch_idx) assign the sticks to the defined channels.
% 
% Inputs:
%   joystickname            give your joystick a name (e.g. 'PS4'): the
%                           specified name will be used as the
%                           configuration file name (string)
% 
% Example:
%   joystickGetConfig( 'PS4' )
% 
% Literature:
%   [1] https://www.frsky-rc.com/wp-content/uploads/Downloads/Manual/x7x7s%20access/X7%20X7S%20ACCESS%20-manual-20191226.pdf
% 
% See also:
%   joystickLoadParams
%

% Disclamer:
%   SPDX-License-Identifier: GPL-3.0-only
% 
%   Copyright (C) 2020-2022 Yannic Beyer
%   Copyright (C) 2022 TU Braunschweig, Institute of Flight Guidance
% *************************************************************************

% set accuracy of trim, max, min and dead zone values
accuracy = 0.01;

% set how much time the user has to react, in seconds
time_move = 2;
time_centered = 2;

% list of all actions during the calibration (in between a stick centered
% request will be added)
action = { ...
    'Wiggle all sticks very carefully to define the dead zone.'; ...
    'Move left stick up.'; ...
    'Move left stick down.'; ...
    'Move left stick left.'; ...
    'Move left stick right.'; ...
    'Move right stick up.'; ...
    'Move right stick down.'; ...
    'Move right stick left.'; ...
    'Move right stick right.'; ...
    'Move/switch channel 5 to maximum position.'; ...
    'Move/switch channel 6 to maximum position.'; ...
    'Move/switch channel 7 to maximum position.'; ...
    'Move/switch channel 8 to maximum position.'; ...
    };


% compile model and print to console so that the user knows that something
% happens
fprintf('Compiling the Simulink model...\n');
sim('joystick_setup',0.1);


% set number of channels
num_channels = 16;

% init
ch_trim = zeros(num_channels,1);
ch_reversed = false(num_channels,1);
ch_max = ones(num_channels,1);
ch_min = -ones(num_channels,1);

left_stick_down_up = 0;
left_stick_left_right = 0;
right_stick_down_up = 0;
right_stick_left_right = 0;
function_5 = 0;
function_6 = 0;
function_7 = 0;
function_8 = 0;

% init mean, max and min stick positions
js_mean = zeros(num_channels,1);
js_max = zeros(num_channels,1);
js_min = zeros(num_channels,1);

joystickOut = {};

% start calibration
for i = 1:2*length(action)
    i_even = i/2;
    i_isodd = rem( i, 2 );
    if i_isodd
        printAction('Hold all sticks centered and move/switch all pots/switches to minimum position.'); 
        if i == 1
            pause(time_centered);
            sim('joystick_setup',time_move);
            for j = 1:num_channels
                js_mean(j) = mean(joystickOut{1}.Values.Data(:,j));
            end
            ch_trim(:) = js_mean;
        else
            pause(time_centered);
        end
    else
        printAction(action{i_even});
        sim('joystick_setup',time_move);
        for j = 1:num_channels
            js_max(j) = max(joystickOut{1}.Values.Data(:,j));
            js_min(j) = min(joystickOut{1}.Values.Data(:,j));
        end
        diff_max = max( js_max - ch_trim );
        diff_min = min( js_min - ch_trim );
        [~,idx_max] = max( js_max - ch_trim );
        [~,idx_min] = min( js_min - ch_trim );
        if diff_max == 0 && diff_min == 0 && ~contains(action{i_even},'Wiggle')
            if i_even < 5
                error('No moving stick was detected. Try to move it continuously to ensure recognition.');
            else
                current_idx = i_even;
            end
        elseif diff_max > abs(diff_min)
            current_idx = idx_max;
            ch_max(current_idx) = js_max(current_idx);
        else
            current_idx = idx_min;
            ch_min(current_idx) = js_min(current_idx);
        end
        
        if contains(action{i_even},'Wiggle')
            ch_dead_max = js_max - ch_trim;
            ch_dead_min = js_min - ch_trim;
        elseif contains(action{i_even},'left') && contains(action{i_even},'up')
            left_stick_down_up = setStickIdx( left_stick_down_up, current_idx );
            if diff_max < abs(diff_min)
                ch_reversed(current_idx) = true;
            end
        elseif contains(action{i_even},'left') && contains(action{i_even},'down')
            left_stick_down_up = setStickIdx( left_stick_down_up, current_idx );
        elseif count(action{i_even},'left') == 2
            left_stick_left_right = setStickIdx( left_stick_left_right, current_idx );
        elseif contains(action{i_even},'left') && contains(action{i_even},'right')
            action_split = strsplit(action{i_even},'left');
            if contains(action_split{2},'right')
                left_stick_left_right = setStickIdx( left_stick_left_right, current_idx );
                if diff_max < abs(diff_min)
                    ch_reversed(current_idx) = true;
                end
            elseif contains(action_split{1},'right')
                right_stick_left_right = setStickIdx( right_stick_left_right, current_idx );
            end
        elseif contains(action{i_even},'right') && contains(action{i_even},'up')
            right_stick_down_up = setStickIdx( right_stick_down_up, current_idx );
            if diff_max < abs(diff_min)
                ch_reversed(current_idx) = true;
            end
        elseif contains(action{i_even},'right') && contains(action{i_even},'down')
            right_stick_down_up = setStickIdx( right_stick_down_up, current_idx );
        elseif count(action{i_even},'right') == 2
            right_stick_left_right = setStickIdx( right_stick_left_right, current_idx );
            if diff_max < abs(diff_min)
                ch_reversed(current_idx) = true;
            end
        elseif contains(action{i_even},'Move/switch')
            if diff_max < abs(diff_min)
                ch_min(current_idx) = js_min(current_idx);
                ch_reversed(current_idx) = true;
                ch_max(current_idx) = ch_trim(current_idx);
            else
                ch_max(current_idx) = js_max(current_idx);
                ch_min(current_idx) = ch_trim(current_idx);
            end
            if contains(action{i_even},'5')
                function_5 = current_idx;
            elseif contains(action{i_even},'6')
                function_6 = current_idx;
            elseif contains(action{i_even},'7')
                function_7 = current_idx;
            elseif contains(action{i_even},'8')
                function_8 = current_idx;
            end
        end
    end
end

% post processing
fprintf('Configuration finished. Results:\n')

% define channels similar to Taranis Q X7 [1]
ch_idx_sub = [ ...
    right_stick_left_right; ...
    right_stick_down_up; ...
    left_stick_down_up; ...
    left_stick_left_right; ...
    function_5; ...
    function_6; ...
    function_7; ...
    function_8 ...
    ];


% create joystick struct
% round to .5 values
jystck.ch_trim = round(1/accuracy*ch_trim)*accuracy;
jystck.ch_idx = joystickAssignChannels(ch_idx_sub);
jystck.ch_reversed = ch_reversed;
% round to .1 values
jystck.ch_max = round(1/accuracy*ch_max)*accuracy;
jystck.ch_min = round(1/accuracy*ch_min)*accuracy;
jystck.ch_dead_max = round(1/accuracy*ch_dead_max)*accuracy;
jystck.ch_dead_min = round(1/accuracy*ch_dead_min)*accuracy;

% print results to console
disp(['- trim values: ',arr2str(jystck.ch_trim(:)')]);
disp(['- indices: ',arr2str(jystck.ch_idx(:)')]);
disp(['- is reversed: ',arr2str(jystck.ch_reversed(:)')]);
disp(['- max values: ',arr2str(jystck.ch_max(:)')]);
disp(['- min values: ',arr2str(jystck.ch_min(:)')]);
disp(['- max dead zone: ',arr2str(jystck.ch_dead_max(:)')]);
disp(['- min dead zone: ',arr2str(jystck.ch_dead_min(:)')]);

% write configuration file
joystickWriteConfig( joystickname, jystck );

% final console output
disp(['Configuration file "joystick_params_', joystickname,'" successfully created in "LADAC/control/joystick/params".'])
disp('Finished successfully.')


    % sub functions
    
    function stickIdx = setStickIdx( stickIdx, idx )
        % Set the stick index and throw and error if the same stick should
        % have been moved in different directions but a second stick was
        % moved instead (index ambiguous).
        if stickIdx == 0 || stickIdx == idx
            stickIdx = idx;
        else
            error('Different sticks were detected for one stick up/down or left/right.');
        end
    end

    function [] = printAction( actionText )
        % Stick movement request animation in the console.
        % The signs are selected automatically depending on key words in
        % the "actionText" string (key words are: all, center, left, right,
        % up, down).
        center = 9675;
        left = 8592;
        right = 8594;
        up = 8593;
        down = 8595;
        thin_space = 8239;
        if contains(actionText,'all') && contains(actionText,'center')
            fprintf( [actionText,'\n   ____|___ \n  |  ....  |\n  | %s   %s |\n  |________|\n'], center, center )
        elseif contains(actionText,'Wiggle')
            fprintf( [actionText,'\n   ____|___ \n  |  ....  |\n  | +    + |\n  |________|\n'] )
        elseif contains(actionText,'left') && contains(actionText,'up')
            fprintf( [actionText,'\n   ____|___ \n  |  ....  |\n  | %s   %s%s |\n  |________|\n'], up, thin_space, center )
        elseif contains(actionText,'left') && contains(actionText,'down')
            fprintf( [actionText,'\n   ____|___ \n  |  ....  |\n  | %s   %s%s |\n  |________|\n'], down, thin_space, center )
        elseif count(actionText,'left') == 2
            fprintf( [actionText,'\n   ____|___ \n  |  ....  |\n  | %s   %s |\n  |________|\n'], left, center )
        elseif contains(actionText,'left') && contains(actionText,'right')
            actionSplit = strsplit(actionText,'left');
            if contains(actionSplit{2},'right')
                fprintf( [actionText,'\n   ____|___ \n  |  ....  |\n  | %s   %s |\n  |________|\n'], right, center )
            elseif contains(actionSplit{1},'right')
                fprintf( [actionText,'\n   ____|___ \n  |  ....  |\n  | %s   %s |\n  |________|\n'], center, left )
            end
        elseif contains(actionText,'right') && contains(actionText,'up')
            fprintf( [actionText,'\n   ____|___ \n  |  ....  |\n  | %s   %s%s |\n  |________|\n'], center, thin_space, up )
        elseif contains(actionText,'right') && contains(actionText,'down')
            fprintf( [actionText,'\n   ____|___ \n  |  ....  |\n  | %s   %s%s |\n  |________|\n'], center, thin_space, down )
        elseif count( actionText,'right') == 2
            fprintf( [actionText,'\n   ____|___ \n  |  ....  |\n  | %s   %s |\n  |________|\n'], center, right )
        elseif contains( actionText,'5')
            fprintf( [actionText,'\n   ____|___ \n  |  |...  |\n  | %s   %s |\n  |________|\n'], center, center )
        elseif contains( actionText,'6')
            fprintf( [actionText,'\n   ____|___ \n  |  .|..  |\n  | %s   %s |\n  |________|\n'], center, center )
        elseif contains( actionText,'7')
            fprintf( [actionText,'\n   ____|___ \n  |  ..|.  |\n  | %s   %s |\n  |________|\n'], center, center )
        elseif contains( actionText,'8')
            fprintf( [actionText,'\n   ____|___ \n  |  ...|  |\n  | %s   %s |\n  |________|\n'], center, center )
        end   
    end

    function joystickWriteConfig( joystickname, jystck )
        % write jystck struct to configuration file
        
        % add prefix joystick_param to scriptname
        scriptname = ['joystick_params_',joystickname];
        % get full path of this file to save config file in the right
        % folder
        mfilepath = fileparts(mfilename('fullpath'));
        % add .m extension
        scriptnamem = sprintf('%s.m', scriptname);
        % create config file
        fid = fopen([mfilepath,'/params/',scriptnamem], 'wt');
        % write into config file
        fprintf(fid, [ ...
            ' %% *** configuration script for the ', joystickname, ' joystick ***\n\n' ...
            ' %% values for all channels when the sticks are centered\n' ...
            'jystck.ch_trim = [', arr2str(jystck.ch_trim), ']'';\n' ...
            ' %% index of all channels so that they are compatible with the Simulink block\n' ...
            'jystck.ch_idx = uint8([', arr2str(jystck.ch_idx), ']'');\n' ...
            ' %% logical array that is true if a channel is reversed\n' ...
            'jystck.ch_reversed = logical([', arr2str(jystck.ch_reversed), ']'');\n' ...
            ' %% maximum channel values\n' ...
            'jystck.ch_max = [', arr2str(jystck.ch_max), ']'';\n' ...
            ' %% minimum channel values\n' ...
            'jystck.ch_min = [', arr2str(jystck.ch_min), ']'';\n' ...
            ' %% maximum dead zone values of the channels\n' ...
            'jystck.ch_dead_max = [', arr2str(jystck.ch_dead_max), ']'';\n' ...
            ' %% minimum dead zone values of the channels\n' ...
            'jystck.ch_dead_min = [', arr2str(jystck.ch_dead_min), ']'';\n' ...
            ] );
        % close config file
        fclose(fid);
    end

    function ch_idx = joystickAssignChannels(ch_idx_sub)
        % If the ch_idx_sub array is shorter than ch_idx, avoid duplicate
        % indices.
        if any(ch_idx_sub==0)
            jj = 1;
            for kk = 1:length(ch_idx_sub)
                if ch_idx_sub(kk) == 0
                    while any(ch_idx_sub==jj)
                        jj = jj + 1;
                    end
                    ch_idx_sub(kk) = jj;
                end
            end
        else
            ch_idx = ch_idx_sub;
        end
    end

    function txt = arr2str( double_array )
        % array to string without duplicate spaces
        txt_with_duplicate_spaces = num2str(double_array(:)');
        txt = regexprep(txt_with_duplicate_spaces,' +',' ');
    end

end
