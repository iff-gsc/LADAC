function suspPayloadGetFrequencies(uav, susp_payload)
    
% Disclaimer:
%   SPDX-License-Identifier: GPL-3.0-only
% 
%   Copyright (C) 2023 Jonas Withelm
%   Copyright (C) 2023 TU Braunschweig, Institute of Flight Guidance
% *************************************************************************

% Read loaded configuration
m_uav = uav.body.m;
m_pld = susp_payload.body.m;

Ixx_uav = uav.body.I(1,1);
Iyy_uav = uav.body.I(2,2);
Izz_uav = uav.body.I(3,3);

Ixx_pld = susp_payload.body.I(1,1);
Iyy_pld = susp_payload.body.I(2,2);
Izz_pld = susp_payload.body.I(3,3);

kt_s = susp_payload.joint.trans_stiffness;
kt_d = susp_payload.joint.trans_damping;

kr_s = susp_payload.joint.rot_stiffness;
kr_d = susp_payload.joint.rot_damping;

joint_uav_c = susp_payload.joint.joint_pos_uav_c;
cog_uav_c   = uav.config.CoG_Pos_c;
l_z_uav     = abs(joint_uav_c(3) - cog_uav_c(3));

joint_pld = [0;0;0];
cog_pld   = susp_payload.config.CoG_Pos_c;
l_z_pld   = abs(joint_pld(3) - cog_pld(3));



%% Translatoric modes
fprintf('\n<strong>Translatoric coupling:</strong>\n\n')


% Double mass oscillator (point masses)
k_s = kt_s;
k_d = kt_d;
A = [ 0,          1,          0,          0;
     -k_s/m_uav, -k_d/m_uav,  k_s/m_uav,  k_d/m_uav;
      0,          0,          0,          1;
      k_s/m_pld,  k_d/m_pld, -k_s/m_pld, -k_d/m_pld];
evals = eig(A);

fprintf('Double mass oscillator (point mass (UAV) + point mass (payload)):\n')
printEigs(evals);


% Double mass oscillator (rigid body (UAV) + point mass (payload))
k_s = kt_s;
k_d = kt_d;
l = l_z_uav;
I_c = min([Ixx_uav, Iyy_uav]);
A = [ 0,           1,            0,            0,            0,              0;
     -k_s/m_pld,  -k_d/m_pld,    k_s/m_pld,    k_d/m_pld,   (k_s*l)/m_pld,  (k_d*l)/m_pld;
      0,           0,            0,            1,            0,              0;
      k_s/m_uav,   k_d/m_uav,   -k_s/m_uav,   -k_d/m_uav,  -(k_s*l)/m_uav, -(k_d*l)/m_uav;
      0,           0,            0,            0,            0,              1;
     (k_s*l)/I_c, (k_d*l)/I_c, -(k_s*l)/I_c, -(k_d*l)/I_c, -(k_s*l^2)/I_c, -(k_d*l^2)/I_c];
evals = eig(A);

fprintf('\nDouble mass oscillator (rigid body (UAV) + point mass (payload)):\n')
printEigs(evals);


% Double mass oscillator (point mass (UAV) + rigid body (payload))
k_s = kt_s;
k_d = kt_d;
l = l_z_pld;
I_l = min([Ixx_pld, Iyy_pld]);

A = [ 0,           1,            0,            0,            0,              0;
     -k_s/m_uav,  -k_d/m_uav,    k_s/m_uav,    k_d/m_uav,   (k_s*l)/m_uav,  (k_d*l)/m_uav;
      0,           0,            0,            1,            0,              0;
      k_s/m_pld,   k_d/m_pld,   -k_s/m_pld,   -k_d/m_pld,  -(k_s*l)/m_pld, -(k_d*l)/m_pld;
      0,           0,            0,            0,            0,              1;
     (k_s*l)/I_l, (k_d*l)/I_l, -(k_s*l)/I_l, -(k_d*l)/I_l, -(k_s*l^2)/I_l, -(k_d*l^2)/I_l];
evals = eig(A);

fprintf('\nDouble mass oscillator (point mass (UAV) + rigid body (payload)):\n')
printEigs(evals);


% Double mass oscillator (rigid body (UAV) + rigid body (payload)), worst case (rotatoric oscillation of both bodies))
k_s = kt_s;
k_d = kt_d;
I_c = min([Ixx_uav, Iyy_uav]);
I_l = min([Ixx_pld, Iyy_pld]);
l_c = l_z_uav;
l_l = l_z_pld;

A = [ 0,                1,                0,               0,                0,                    0,                    0,                    0;
     -k_s/m_uav,       -k_d/m_uav,        k_s/m_uav,       k_d/m_uav,      -(k_s*l_c)/m_uav,     -(k_d*l_c)/m_uav,     -(k_s*l_l)/m_uav,     -(k_d*l_l)/m_uav;
      0,                0,                0,               1,                0,                    0,                    0,                    0;
      k_s/m_pld,        k_d/m_pld,       -k_s/m_pld,      -k_d/m_pld,       (k_s*l_c)/m_pld,      (k_d*l_c)/m_pld,      (k_s*l_l)/m_pld,      (k_d*l_l)/m_pld;
      0,                0,                0,               0,                0,                    1,                    0,                    0;
    -(l_c*k_s)/I_c,   -(l_c*k_d)/I_c,    (l_c*k_s)/I_c,   (l_c*k_d)/I_c,   -(l_c^2*k_s)/I_c,     -(l_c^2*k_d)/I_c,     -(l_c*l_l*k_s)/I_c,   -(l_c*l_l*k_d)/I_c;
      0,                0,                0,               0,                0,                    0,                    0,                    1;
    -(l_c*k_s)/I_l,   -(l_c*k_d)/I_l,    (l_c*k_s)/I_l,   (l_c*k_d)/I_l,   -(l_c*l_l*k_s)/I_l,   -(l_c*l_l*k_d)/I_l,   -(l_l^2*k_s)/I_l,     -(l_l^2*k_d)/I_l];
evals = eig(A);

fprintf('\nDouble mass oscillator (rigid body (UAV) + rigid body (payload)), worst case:\n')
printEigs(evals);



%% Rotatoric modes
fprintf('\n\n<strong>Rotatoric coupling:</strong>\n')


% Torsional oscillator
k_ts = kr_s;
k_td = kr_d;
I_c = Izz_uav;
I_l = Izz_pld;

A = [ 0,         1,         0,         0;
     -k_ts/I_c, -k_td/I_c,  k_ts/I_c,  k_td/I_c;
      0,         0,         0,         1;
      k_ts/I_l,  k_td/I_l, -k_ts/I_l, -k_td/I_l];
evals = eig(A);

fprintf('\nTorsional oscillator (rigid body (UAV) + rigid body (payload)):\n')
printEigs(evals);

end





%% LOCAL FUNCTIONS
function printEigs(eigs)
    
    epsilon = 1e-4;
    
    data_header = {'Eigenvalue:', 'omega_0 (Hz):', 'omega_d (Hz):', 'Phase (deg):'};
    
    data = cell.empty(0,4);
    d_idx = 1;
    
    for idx=1:numel(eigs)
        if abs(eigs(idx)) > epsilon
            val = eigs(idx);
            
            im = imag(val);
            re = real(val);
            if abs(im) < epsilon
                im = 0;
            end
            if abs(re) < epsilon
                re = 0;
            end
            
            im_str = sprintf('%.5g', im);
            re_str = sprintf('%.5g', re);
            
            data{d_idx,1} = [phantom_s(re_str) realtom_s(im_str)];

            data{d_idx,2} = num2str(abs(eigs(idx))/(2*pi));

            omega_d = imag(eigs(idx))/(2*pi);
            data{d_idx,3} = phantom_n(omega_d);

            phase_deg = rad2deg(phase(eigs(idx)));
            data{d_idx,4} = phantom_n(phase_deg);
            
            d_idx = d_idx + 1;
        end
    end
    printData(data_header, data, 'indent', 2, 'column_spacing', 4);
end
    

function expr = phantom_n(number)
    if number > 0
        expr = [' ' num2str(number)];
    else
        expr = num2str(number);
    end
end

function expr = phantom_s(number)
    if strcmp(number(1), '-')
        expr = number;
    else
        expr = [' ' number];
    end
end

function expr = realtom_s(number)
    if strcmp(number(1), '-')
        expr = number;
    else
        expr = ['+' number];
    end
end
