function state = lindiCopterStateLogic(mode_number)
% lindiCopterStateLogic define logical signals depending on flight mode

% init
state.isPscEnabled      = true;
state.isPosRmEnabled	= true;
state.isGdnceEnabled	= true;
state.isAttiCmdEnabled  = true;
state.isManThrEnabled	= true;


% set variables

switch mode_number
    case {0,1}
        state.isPscEnabled(:) 	= true;
    otherwise
        state.isPscEnabled(:)	= false;
end

switch mode_number
    case {0}
        state.isPosRmEnabled(:) = true;
    otherwise
        state.isPosRmEnabled(:) = false;
end

switch mode_number
    case {1}
        state.isGdnceEnabled(:) = true;
    otherwise
        state.isGdnceEnabled(:) = false;
end

switch mode_number
    case {2}
        state.isAttiCmdEnabled(:)   = true;
    otherwise
        state.isAttiCmdEnabled(:)   = false;
end

switch mode_number
    case {2}
        state.isManThrEnabled(:)    = true;
    otherwise
        state.isManThrEnabled(:)    = false;
end
      
switch mode_number
    case {2}
        state.isManThrEnabled(:)    = true;
    otherwise
        state.isManThrEnabled(:)    = false;
end

end