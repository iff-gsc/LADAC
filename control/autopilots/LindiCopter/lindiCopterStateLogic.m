function state = lindiCopterStateLogic(mode_number,is_traj_valid)
% lindiCopterStateLogic define logical signals depending on flight mode

% init
state.isPscEnabled      = true;
state.isPosRmEnabled	= true;
state.isVertPscEnabled	= true;
state.isGdnceEnabled	= true;
state.isAttiCmdEnabled  = true;
state.isManThrEnabled	= true;
state.isAutoTuneEnabled = true;


% set variables

switch mode_number
    case 0 % loiter
        state.isPscEnabled      = true;
        state.isPosRmEnabled	= true;
        state.isVertPscEnabled	= false;
        state.isGdnceEnabled	= false;
        state.isAttiCmdEnabled  = false;
        state.isManThrEnabled	= false;
        state.isAutoTuneEnabled = false;
    case 1 % guided
        state.isPscEnabled      = true;
        if ~is_traj_valid
            state.isPosRmEnabled = true;
        else
            state.isPosRmEnabled = false;
        end
        state.isVertPscEnabled	= false;
        state.isGdnceEnabled	= true;
        state.isAttiCmdEnabled  = false;
        state.isManThrEnabled	= false;
        state.isAutoTuneEnabled = false;
    case 2 % stabilized
        state.isPscEnabled      = false;
        state.isPosRmEnabled	= false;
        state.isVertPscEnabled	= false;
        state.isGdnceEnabled	= false;
        state.isAttiCmdEnabled  = true;
        state.isManThrEnabled	= true;
        state.isAutoTuneEnabled = false;
    case 3 % altitude hold
        state.isPscEnabled      = true;
        state.isPosRmEnabled	= true;
        state.isVertPscEnabled	= true;
        state.isGdnceEnabled	= false;
        state.isAttiCmdEnabled  = true;
        state.isManThrEnabled	= false;
        state.isAutoTuneEnabled = false;
    case 4 % autotune
        state.isPscEnabled      = true;
        state.isPosRmEnabled	= true;
        state.isVertPscEnabled	= false;
        state.isGdnceEnabled	= false;
        state.isAttiCmdEnabled  = false;
        state.isManThrEnabled	= false;
        state.isAutoTuneEnabled = true;
end

end