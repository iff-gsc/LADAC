function [Phi_dAcc,Theta_dAcc] = eulerAngles_dAcc(xy_gPsi_ddt_cmd,acc_vertical_measure, Theta) %#codegen
% eulerAngles_dAcc computes the derivative of the roll and pitch angle of a
%   multicopter with respect to the corresponding forward and lateral
%   acceleration. This function is based on a kinematic inversion.
%   To do:  - Add documentation with understandable derivation.
%           - The function gives only an approximation of the correct
%             result by now (see below).
% 
% Inputs:
%   xy_gPsi_ddt_cmd         commanded acceleration in a frame that is
%                           defined such that x is in heading direction and
%                           the xy-plane is parallel to the earth surface
%                           (g frame rotated about Psi).
%                           in m/s^2
%   acc_vertical_measure    measured vertical acceleration parallel to the
%                           gravity vector (including gravity), in m/s^2
% 
% Outputs:
%   Phi_dAcc                commanded roll angle, in rad
%   Theta_dAcc              commanded pitch angle, in rad
% 

% Disclamer:
%   SPDX-License-Identifier: GPL-2.0-only
% 
%   Copyright (C) 2020-2022 Yannic Beyer
%   Copyright (C) 2022 TU Braunschweig, Institute of Flight Guidance
% *************************************************************************

% avoid devision by zero
acc_vertical_wo_zero = sign(acc_vertical_measure)*max(abs(acc_vertical_measure),0.5);
if abs(acc_vertical_wo_zero) < 0.5
    acc_vertical_wo_zero = acc_vertical_wo_zero*0 + 0.5;
end

Theta_dAcc = [ 1/(1+(xy_gPsi_ddt_cmd(1)/acc_vertical_wo_zero)^2)*1/acc_vertical_wo_zero, 0 ];
% This is only correct if Theta=0.
% In the future, it must be changed. Therefore derive the function
% Phi = atan( y_d2t/( acc_gPsi_x*sin(Theta) + acc_vertical*cos(Theta) ) ),
% with
% Theta = atan( x_d2t/acc_vertical ).
% Phi_dAcc = [ 0, -1/(1+(xy_gPsi_ddt_cmd(2)/acc_vertical_wo_zero)^2)*1/acc_vertical_wo_zero ];
Phi_dAcc = [ xy_gPsi_ddt_cmd(2)/(xy_gPsi_ddt_cmd(2)^2+(xy_gPsi_ddt_cmd(1)*sin(Theta)+acc_vertical_wo_zero*cos(Theta))^2), ...
    -1/(1+(xy_gPsi_ddt_cmd(2)/(xy_gPsi_ddt_cmd(1)*sin(Theta)+acc_vertical_wo_zero*cos(Theta))^2))*1/acc_vertical_wo_zero ];



end
