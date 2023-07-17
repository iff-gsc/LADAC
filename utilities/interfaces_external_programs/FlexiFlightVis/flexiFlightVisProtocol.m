function [ffv,ffv_double_array,ffv_double_idx] = flexiFlightVisProtocol( wing_cell, fuse_cell )
% flexiFlightVisProtocol custom protocol for sending airplane state to
% FlexiFlightVis

num_wing = length(wing_cell);
num_fuse = length(fuse_cell);
ffv.rigid_body = rigidBodyCreate();
ffv.config.xyz_ref_c = zeros(3,1);
for i = 1:num_wing
    ffv.(['wing_',num2str(i),'_state']) = wing_cell{i};
end
for i = 1:num_fuse
    ffv.(['fuselage_',num2str(i),'_state']) = fuse_cell{i};
end

struct2bus(ffv,'ffv_bus');

% convert struct to double array with index array
[ffv_double_array,ffv_double_idx] = struct2double(ffv,999999);

end