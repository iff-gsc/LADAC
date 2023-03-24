function [] = lindiCopterStateLogicBus()
% lindiCopterStateLogicBus create Simulink Bus Object for state logic
    state_logic = lindiCopterStateLogic(0);
    struct2bus(state_logic,'lindiStateLogicBus');
end