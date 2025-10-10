classdef T2Filter < matlab.System
    properties
        % Cutoff frequency (rad/s)
        omega_0	= 100;
        % Damping ratio
        d = 1;
    end
    properties(Nontunable)
        % Sample time (s)
        Ts      = 1/400;
        % Enable velocity output
        is_vel = false;
        % Enable acceleration output
        is_acc = false;
    end
    properties(DiscreteState)
        x
    end
    properties(Access = private)
        Ad
        Bd
        T
        last_omega_0
        last_d
        is_init = true;
    end
    methods(Access = protected)
        function setupImpl(obj,u)
            if obj.Ts <= 0
                error('Sample time must be positive.')
            else
                obj.T = obj.Ts*ones(1,class(obj.omega_0));
            end
            setAd(obj);
            setBd(obj);
            size_in = propagatedInputSize(obj,1);
            n_i = prod(size_in);
            obj.x = zeros(2,n_i,class(u));
            obj.last_omega_0 = obj.omega_0;
            obj.last_d = obj.d;
        end

        function varargout = stepImpl(obj,u)
            if obj.is_init
                init(obj,u);
            end
            updateParams(obj);
            
            size_in = propagatedInputSize(obj,1);
            n_i = prod(size_in);
            n_o = getNumOutputsImpl(obj);
            y = zeros(n_o,n_i,class(u));
            
            Cd = getCd(obj);
            Dd = getDd(obj);
            for i = 1:size(obj.x,2)
                y(:,i) = Cd*obj.x(:,i) + Dd*u(i);
                obj.x(:,i) = obj.Ad*obj.x(:,i) + obj.Bd*u(i);
                if any(isnan(obj.x(:,i))) || any(isinf(obj.x(:,i)))
                    init(obj,u);
                end 
            end
            
            % reshape output to match input size
            if ~obj.is_vel && ~obj.is_acc
                varargout{1} = reshape(y,size_in);
            elseif (obj.is_vel && ~obj.is_acc) || (~obj.is_vel && obj.is_acc)
                varargout{1} = reshape(y(1,:),size_in);
                varargout{2} = reshape(y(2,:),size_in);
            else
                varargout{1} = reshape(y(1,:),size_in);
                varargout{2} = reshape(y(2,:),size_in);
                varargout{3} = reshape(y(3,:),size_in);
            end
            
        end
        
        % resetImpl does not allow input u
        function init(obj,u)
            obj.x(1,:) = u(:);
            obj.x(2,:) = 0;
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
                obj.last_omega_0 = obj.omega_0;
                obj.last_d = obj.d;
            end
        end
        
        function setAd(obj)
            A = [ 0, 1; -obj.omega_0^2, -2*obj.d*obj.omega_0 ];
            obj.Ad = expm2(A*obj.T);
        end
        
        function setBd(obj)
            B = [0; obj.omega_0^2];
            A = [ 0, 1; -obj.omega_0^2, -2*obj.d*obj.omega_0 ];
            A_inv = inv2(A);
            obj.Bd = A_inv * (obj.Ad-eye(2)) * B;
        end
        
        function Cd = getCd(obj)
            if ~obj.is_vel && ~obj.is_acc
                Cd = [1 0]*ones(1,class(obj.omega_0));
            elseif obj.is_vel && ~obj.is_acc
                Cd = [1 0; 0 1]*ones(1,class(obj.omega_0));
            elseif ~obj.is_vel && obj.is_acc
                Cd = [1 0; 0 0]*ones(1,class(obj.omega_0));
            else
                Cd = [1 0; 0 1; -obj.omega_0^2, -2*obj.d*obj.omega_0]...
                    *ones(1,class(obj.omega_0));
            end
        end
        
        function Dd = getDd(obj)
            if ~obj.is_vel && ~obj.is_acc
                Dd = zeros(1,class(obj.omega_0));
            elseif obj.is_vel && ~obj.is_acc
                Dd = zeros(2,1,class(obj.omega_0));
            elseif ~obj.is_vel && obj.is_acc
                Dd = [0; obj.omega_0^2]*ones(1,class(obj.omega_0));
            else
                Dd = [0; 0; obj.omega_0^2]*ones(1,class(obj.omega_0));
            end
        end
        
    end
end