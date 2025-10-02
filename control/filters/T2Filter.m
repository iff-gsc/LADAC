classdef T2Filter < matlab.System
    properties
        % Cutoff frequency (rad/s)
        omega_0	= 2*pi*80;
        % Damping ratio
        d = 1;
        % Sample time (s)
        Ts      = 1/400;
    end
    properties(Nontunable)
        % Enable velocity output
        is_vel (1,1) logical = false;
        % Enable acceleration output
        is_acc (1,1) logical = false;
    end
    properties(DiscreteState)
        x
    end
    properties(Access = private)
        Ad
        Bd
        Cd
        Dd
        T = 1/400;
        last_omega_0;
        last_d;
        u0              = 0;
        is_init         = true;
    end
    methods(Access = protected)
        function setupImpl(obj)
            setAd(obj);
            setBd(obj);
            setCd(obj);
            setDd(obj);
            obj.x = zeros(2,1);
            obj.last_omega_0 = obj.omega_0;
            obj.last_d = obj.d;
        end

        function varargout = stepImpl(obj,u)
            if obj.is_init
                init(obj,u);
            end
            updateParams(obj);
            
            y = obj.Cd*obj.x + obj.Dd*u;
            obj.x = obj.Ad*obj.x + obj.Bd*u;
            
            if ~obj.is_vel && ~obj.is_acc
                varargout{1} = y;
            elseif (obj.is_vel && ~obj.is_acc) || (~obj.is_vel && obj.is_acc)
                varargout{1} = y(1);
                varargout{2} = y(2);
            else
                varargout{1} = y(1);
                varargout{2} = y(2);
                varargout{3} = y(3);
            end
            
            obj.is_init = false;
        end

        function resetImpl(obj)
            obj.x(:) = 0;
            obj.is_init = true;
        end
        
        function num = getNumOutputsImpl(obj)
            if ~obj.is_vel && ~obj.is_acc
                num = 1;
            elseif (obj.is_vel && ~obj.is_acc) || (~obj.is_vel && obj.is_acc)
                num = 2;
            else
                num = 3;
            end
        end
        
        function inputName = getOutputNamesImpl(obj)
            if ~obj.is_vel && ~obj.is_acc
                inputName = "y";
            elseif obj.is_vel && ~obj.is_acc
                inputName = ["y","y_dt"];
            elseif ~obj.is_vel && obj.is_acc
                inputName = ["y","y_dt2"];
            else
                inputName = ["y","y_dt","y_dt2"];
            end
        end
        
        function updateParams(obj)
            if (obj.omega_0~=obj.last_omega_0 || obj.d~=obj.last_d)
                setAd(obj);
                setBd(obj);
                setCd(obj);
                setDd(obj);
                obj.last_omega_0 = obj.omega_0;
                obj.last_d = obj.d;
            end
        end
        
        function setAd(obj)
            A = [0 1;
                -obj.omega_0^2  -2*obj.d*obj.omega_0];
            obj.Ad = expm(A*obj.T);
        end
        
        function setBd(obj)
            A = [0 1;
                -obj.omega_0^2  -2*obj.d*obj.omega_0];
            B = [0; obj.omega_0^2];
            obj.Bd = A \ (obj.Ad-eye(2)) * B;
        end
        
        function setCd(obj)
            if ~obj.is_vel && ~obj.is_acc
                obj.Cd = [1 0];
            elseif obj.is_vel && ~obj.is_acc
                obj.Cd = [1 0; 0 1];
            elseif ~obj.is_vel && obj.is_acc
                obj.Cd = [1 0; 0 0];
            else
                obj.Cd = [1 0; 0 1; -obj.omega_0^2, -2*obj.d*obj.omega_0];
            end
        end
        
        function setDd(obj)
            if ~obj.is_vel && ~obj.is_acc
                obj.Dd = 0;
            elseif obj.is_vel && ~obj.is_acc
                obj.Dd = [0; 0];
            elseif ~obj.is_vel && obj.is_acc
                obj.Dd = [0; obj.omega_0^2];
            else
                obj.Dd = [0; 0; obj.omega_0^2];
            end
        end
        
        function init(obj,u)
            obj.x(1) = u;
            obj.x(2) = 0;
        end
        
    end
end