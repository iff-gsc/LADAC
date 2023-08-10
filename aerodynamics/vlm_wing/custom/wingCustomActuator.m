function [c_L,c_m,c_D,z2_dt] = wingCustomActuator(wing)
c_L = zeros(1,wing.n_panel);
c_m = zeros(1,wing.n_panel);
c_D = zeros(1,wing.n_panel);
z2_dt = zeros(size(wing.state.aero.unsteady.z2_dt));
end