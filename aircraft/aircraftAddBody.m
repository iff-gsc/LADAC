function aircraft = aircraftAddBody(aircraft, component)

% New CoG in c frame
CoG_Pos_c = (aircraft.body.m * aircraft.config.CoG_Pos_c + component.body.m * component.config.CoG_Pos_c)./(aircraft.body.m + component.body.m);

% New total mass
m = aircraft.body.m + component.body.m;

% New inertia at new CoG
pos_offset_aircraft = aircraft.config.CoG_Pos_c - CoG_Pos_c;
I_aircraft = inertiaWithOffset(pos_offset_aircraft, aircraft.body.I, aircraft.body.m);

pos_offset_component = component.config.CoG_Pos_c - CoG_Pos_c;
I_component = inertiaWithOffset(pos_offset_component, component.body.I, component.body.m);

aircraft.config.CoG_Pos_c = CoG_Pos_c;
aircraft.body.m = m;
aircraft.body.I = I_aircraft + I_component;

end





%% LOCAL FUNCTIONS
function I = inertiaWithOffset(pos_offset, I_CoG, m)
    
    % ToDo: Implement rotation!
    
    a = [ 0, -pos_offset(3), pos_offset(2)
          pos_offset(3), 0, -pos_offset(1)
         -pos_offset(2), pos_offset(1), 0 ];
    
    I = I_CoG + m * (a' * a);
end
