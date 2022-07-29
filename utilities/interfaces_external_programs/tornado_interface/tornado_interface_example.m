%% define wing
geo = tornadoCreateGeo( 'wing_params_default', zeros(3,1), zeros(3,1), ...
    30, 3, 3, deg2rad([10,-10])*0, [true,true] );

%% define wing state
state=tornadoCreateState(deg2rad(2),deg2rad(3)*0,20,zeros(3,1),10667,true);
lattice_type = 1;
[lattice,ref]=fLattice_setup2(geo,state,lattice_type);

%% plot wing geometry
% geometryplot(lattice,geo,ref);

%% run VLM
clear results
results.a=0;
results=solver9(results,state,geo,lattice,ref);
[results]=coeff_create3(results,lattice,state,ref,geo);

%% plot spanwise lift distribution
% figure
plot( results.ystation/max(results.ystation), results.CL_local )
grid on
xlabel('Dimensionless span')
ylabel('Local lift coefficient')
