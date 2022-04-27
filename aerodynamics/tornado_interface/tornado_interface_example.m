%% define wing
nx = 3;
geo = tornadoCreateGeo( 'wing_params_default',zeros(3,1),zeros(3,1),30,nx,1);

%% define wing state
state=tornadoCreateState(deg2rad(2),deg2rad(3)*0,20,zeros(3,1),10667,true);
lattice_type = 0;
[lattice,ref]=fLattice_setup2(geo,state,lattice_type);

%% plot wing geometry
% geometryplot(lattice,geo,ref);

%% run VLM
clear results
results.a=0;
results=solver9(results,state,geo,lattice,ref);
[results]=coeff_create3(results,lattice,state,ref,geo);

%% plot spanwise lift distribution
figure
cntrl_y = sort(lattice.COLLOC(1:nx:end,2));
plot( cntrl_y/max(cntrl_y), results.CL_local )
