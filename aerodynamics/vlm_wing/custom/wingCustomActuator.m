function [c_L,c_m,c_D,z2_dt] = wingCustomActuator(wing)
custom_path = which('wingCustomActuator','-all');
if length(custom_path) > 1
    for i = 1:length(custom_path)
        if contains(custom_path{i},'/ladac/') || contains(custom_path{i},'/LADAC/')
            rmpath(fileparts(custom_path{i}));
        end
    end
end
c_L = zeros(1,wing.n_panel);
c_m = zeros(1,wing.n_panel);
c_D = zeros(1,wing.n_panel);
z2_dt = zeros(size(wing.state.aero.unsteady.z2_dt));
end