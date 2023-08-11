function [c_L,c_m,c_D,z2_dt] = wingCustomActuator(wing)
c_L = zeros(size(wing.state.aero.unsteady.c_L_act2));
c_m = zeros(size(wing.state.aero.unsteady.c_L_act2));
c_D = zeros(size(wing.state.aero.unsteady.c_L_act2));
z2_dt = zeros(size(wing.state.aero.unsteady.z2_dt));
end