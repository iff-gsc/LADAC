function [prm] = wingGetParamsFromCPACS( tiglHandle, wing_idx )
% Description of wingGetParamsFromCPACS

% Disclamer:
%   SPDX-License-Identifier: GPL-2.0-only
% 
%   Copyright (C) 2020-2022 Yannic Beyer
%   Copyright (C) 2022 TU Braunschweig, Institute of Flight Guidance
% *************************************************************************

%% Get number of segments

num_segments = tiglWingGetSegmentCount( tiglHandle, wing_idx );

wingUID = tiglWingGetUID( tiglHandle, wing_idx );

prm.is_symmetrical = tiglWingGetSymmetry(tiglHandle,wing_idx)==2;

prm.b = tiglWingGetSpan( tiglHandle, wingUID );

% init variable
prm.eta_segments_wing = zeros( 1, num_segments + 1 );

prm.lambda  = zeros( 1, num_segments );
prm.nu      = zeros( 1, num_segments );
prm.epsilon = zeros( 1, num_segments );

pos_matrix_lead = zeros( 3, num_segments+1 );
pos_matrix_25 = zeros( 3, num_segments+1 );
pos_matrix_75 = zeros( 3, num_segments+1 );
pos_matrix_trail = zeros( 3, num_segments+1 );

camber_rel = zeros( 1, num_segments+1 );
camber_pos = zeros( 1, num_segments+1 );

% prm.norm_vec = zeros( 3, num_segments+1 );

for i = 0:num_segments
    if i == 0
        [x_lead,y_lead,z_lead] = tiglWingGetChordPoint_frd( tiglHandle, wing_idx, i+1, 0, 0 );
        [a,b,c]=tiglWingGetUpperPoint(tiglHandle,wing_idx,i+1,0,0);
        [x_25,y_25,z_25] = tiglWingGetChordPoint_frd( tiglHandle, wing_idx, i+1, 0, 0.25 );
        [x_75,y_75,z_75] = tiglWingGetChordPoint_frd( tiglHandle, wing_idx, i+1, 0, 0.75 );
        [x_trail,y_trail,z_trail] = tiglWingGetChordPoint_frd( tiglHandle, wing_idx, i+1, 0, 1 );
        [cam_rel,cam_pos] = tiglWingGetCamber( tiglHandle, wing_idx, i+1, 0 );
        % The norm vector seems to be not correct. It seems that the
        % "chord" in the tiglWingGetChordNormal function is not a straight
        % line from the leading edge to the trailing edge.
%         [ prm.norm_vec(1,i+1), prm.norm_vec(2,i+1), prm.norm_vec(3,i+1) ] = ...
%             tiglWingGetChordNormal_frd(tiglHandle,wing_idx,i+1,0,0.25);
    else        
        [x_lead,y_lead,z_lead] = tiglWingGetChordPoint_frd( tiglHandle, wing_idx, i, 1, 0 );
        [x_25,y_25,z_25] = tiglWingGetChordPoint_frd( tiglHandle, wing_idx, i, 1, 0.25 );
        [x_75,y_75,z_75] = tiglWingGetChordPoint_frd( tiglHandle, wing_idx, i, 1, 0.75 );
        [x_trail,y_trail,z_trail] = tiglWingGetChordPoint_frd( tiglHandle, wing_idx, i, 1, 1 );
        [cam_rel,cam_pos] = tiglWingGetCamber( tiglHandle, wing_idx, i, 1 );
%         [ prm.norm_vec(1,i+1), prm.norm_vec(2,i+1), prm.norm_vec(3,i+1) ] = ...
%             tiglWingGetChordNormal_frd(tiglHandle,wing_idx,i,1,0.25);
    end
    
    pos_matrix_lead(:,i+1) = [ x_lead; y_lead; z_lead ];
    pos_matrix_25(:,i+1) = [ x_25; y_25; z_25 ];
    pos_matrix_75(:,i+1) = [ x_75; y_75; z_75 ];
    pos_matrix_trail(:,i+1) = [ x_trail; y_trail; z_trail ];
    
    camber_rel(i+1) = cam_rel;
    camber_pos(i+1) = cam_pos;

    if prm.is_symmetrical
        sym_factor = 2;
    else
        sym_factor = 1;
    end
    prm.eta_segments_wing(i+1) = sqrt((y_25-pos_matrix_25(2,1)).^2+(z_25-pos_matrix_25(3,1))^2) / prm.b * sym_factor;
    prm.c(i+1) = x_lead - x_trail;
    
    if i > 0
        prm.lambda(i) = atan( -(pos_matrix_25(1,i+1)-pos_matrix_25(1,i)) / (pos_matrix_25(2,i+1)-pos_matrix_25(2,i)) );
        prm.nu(i) = atan( -(pos_matrix_25(3,i+1)-pos_matrix_25(3,i)) / (pos_matrix_25(2,i+1)-pos_matrix_25(2,i)) );
        prm.epsilon(i) = atan( -( pos_matrix_lead(3,i+1) - pos_matrix_trail(3,i+1) ) / ( pos_matrix_lead(1,i+1) - pos_matrix_trail(1,i+1) ) ) - ...
            atan( -( pos_matrix_lead(3,1) - pos_matrix_trail(3,1) ) / ( pos_matrix_lead(1,1) - pos_matrix_trail(1,1) ) );
    end
    
    % not used yet
    section{i+1,:} = tiglWingGetProfileName(tiglHandle,wing_idx,i+1,1);
    


end

prm.i = atan( -( pos_matrix_lead(3,1) - pos_matrix_trail(3,1) ) / ( pos_matrix_lead(1,1) - pos_matrix_trail(1,1) ) );

prm.rot_x = atan( (pos_matrix_25(3,end)-pos_matrix_25(3,1)) / (pos_matrix_25(2,end)-pos_matrix_25(2,1)) );

prm.nu = prm.nu + prm.rot_x;

alpha_0_camber = camber_rel ./ ( 1 - camber_pos );
alpha_0_camber_50 = interp1(pos_matrix_25(2,:),alpha_0_camber,0.5*pos_matrix_25(2,end));
prm.epsilon = prm.epsilon + (alpha_0_camber(2:end)-alpha_0_camber_50);


    function [x_frd,y_frd,z_frd] = tiglWingGetChordPoint_frd( handle, wing_idx, segment_idx, eta, xsi )
        [x_bru,y_bru,z_bru] = tiglWingGetChordPoint( handle, wing_idx, segment_idx, eta, xsi );
        x_frd = -x_bru;
        y_frd = y_bru;
        z_frd = -z_bru;
    end

    function [x_frd,y_frd,z_frd] = tiglWingGetChordNormal_frd( handle, wing_idx, segment_idx, eta, xsi )
        [x_bru,y_bru,z_bru] = tiglWingGetChordNormal( handle, wing_idx, segment_idx, eta, xsi );
        x_frd = -x_bru;
        y_frd = y_bru;
        z_frd = -z_bru;
    end

    function [camber_rel,camber_pos] = tiglWingGetCamber( tiglHandle, wing_idx, segment_idx, eta )
        len_cam = 101;
        xu = zeros(1,len_cam);
        zu = zeros(1,len_cam);
        xl = zeros(1,len_cam);
        zl = zeros(1,len_cam);
        xsi_vec = linspace(0,1,len_cam);
        for ii = 1:len_cam
            xsi = xsi_vec(ii);
            [xu(ii),~,zu(ii)] = tiglWingGetUpperPoint( tiglHandle, wing_idx, segment_idx, eta, xsi );
            [xl(ii),~,zl(ii)] = tiglWingGetLowerPoint( tiglHandle, wing_idx, segment_idx, eta, xsi );
        end
        chord_length = norm( [xu(1);zu(1)] - [xu(end);zu(end)], 2 );
        xm = xu(1) + xsi_vec * (xu(end)-xu(1));
        zm = interp1([xu(1),xu(end)],[zu(1),zu(end)],xm);
        camber_u = vecnorm( [xu;zu] - [xm;zm], 2, 1 );
        camber_l = vecnorm( [xl;zl] - [xm;zm], 2, 1 );
        [camber_u_max,camber_u_idx] = max(camber_u);
        [camber_l_max,camber_l_idx] = max(camber_l);
        if camber_u_max >= camber_l_max
            camber_max = camber_u_max;
            camber_idx = camber_u_idx;
        else
            camber_max = camber_l_max;
            camber_idx = camber_l_idx;
        end
        camber_rel = camber_max / chord_length;
        camber_pos = xsi_vec(camber_idx);       
    end


    % add data that is not available in the CPACS file yet (hard coded)
    if wing_idx == 1
        prm.S                   = 121.2;
        prm.eta_segments_device = [ 0, 0.101, 0.189, 0.304, 0.662, 0.696, 0.953, 1 ];
        prm.flap_depth          = [ 0, 0.2, 0, 0.2, 0, 0.2, 0 ];
        prm.control_input_index(1,:) = [ 0 1 0 2  0  3  0  0  4  0  5  0  6  0 ];
        prm.control_input_index(2,:) = [ 7 8 9 10 11 12 13 14 15 16 17 18 19 20 ];
        prm.x                   = pos_matrix_lead(1,1);
        prm.z                   = pos_matrix_lead(3,1);
        % hard code section name (aua)
        section                 = cell(length(prm.eta_segments_device),1);
        section(:)              = {'airfoilAnalytic0515_params_F15'};
        actuator_2_type      	= cell(length(prm.eta_segments_device),1);
        actuator_2_type(:)      = {'airfoilMicroTab_params_F15_90'};
    elseif wing_idx == 2
        prm.S                   = prm.b*mean(prm.c);
        prm.eta_segments_device = [ 0, 0.05, 0.95, 1];
        prm.flap_depth          = [ 0, 0.225, 0 ];
        prm.control_input_index(1,:) = [ 0, 1, 0, 0, 1, 0 ];
        prm.control_input_index(2,:) = [ 0, 0, 0, 0, 0, 0 ];
        prm.x                   = pos_matrix_lead(1,1);
        prm.z                   = pos_matrix_lead(3,1);
        % hard code section name (aua)
        section                 = cell(length(prm.eta_segments_device),1);
        section(:)              = {'airfoilAnalyticSimple_params_default'};
        actuator_2_type      	= cell(length(prm.eta_segments_device),1);
        actuator_2_type(:)      = {'none'};
    elseif wing_idx == 3
        prm.S                   = prm.b*mean(prm.c);
        prm.eta_segments_device = [ 0 0.2, 0.9 1 ];
        prm.flap_depth          = [ 0, 0.225, 0 ];
        prm.control_input_index(1,:) = [ 0, 1, 0 ];
        prm.control_input_index(2,:) = [ 0, 0, 0 ];
        prm.x                   = pos_matrix_lead(1,1);
        prm.z                   = pos_matrix_lead(3,1);
        % hard code section name (aua)
        section                 = cell(length(prm.eta_segments_device),1);
        section(:)              = {'airfoilAnalyticSimple_params_default'};
        actuator_2_type      	= cell(length(prm.eta_segments_device),1);
        actuator_2_type(:)      = {'none'};
    end

    prm.section = char(section(1:end-1));
    prm.actuator_2_type = char(actuator_2_type(1:end-1));
    prm.xyz_25 = pos_matrix_25;
    prm.xyz_75 = pos_matrix_75;

end
