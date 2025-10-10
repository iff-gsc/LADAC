classdef NotchFilter < matlab.System
    properties
        % Central rejected frequency (rad/s)
        omega_0 = 2*pi*80;
        % Width of rejected frequency (rad/s)
        omega_c = 2*pi*40;
        % Amplitude reduction at rejected frequency (dB)
        A0_dB = 40;
    end
    properties(Nontunable)
        % Sample time (s)
        Ts = 1/400;
        % Discretization ('zoh', 'tustin')
        discretization_method = 'tustin';
        % Enable varying central frequency?
        external_omega_0 = false;
        % Enable varying frequency width?
        external_omega_c = false;
    end
    properties(DiscreteState)
        x
    end
    properties(Access = private)
        Ad
        Bd
        A0
        omega0
        omegac
        T
        last_omega_0;
        last_omega_c;
        last_A0_dB;
        is_init         = true;
    end
    methods(Access = protected)
        
        function num = getNumInputsImpl(obj)
            if obj.external_omega_0 && obj.external_omega_c
                num = 3;
            elseif obj.external_omega_0 || obj.external_omega_c
                num = 2;
            else
                num = 1;
            end
        end
        
        function setupImpl(obj,u)
            type1 = ones(class(u));
            obj.omega0 = obj.omega_0*type1;
            obj.omegac = obj.omega_c*type1;
            obj.omega0 = max(obj.omega0,2*obj.omegac);
            obj.A0 = 10^(-obj.A0_dB*type1/20);
            if obj.Ts <= 0
                error('Sample time must be positive.')
            else
                obj.T = obj.Ts*ones(1,class(obj.omega0));
            end
            setAdBd(obj);
            obj.x = zeros(2,1,class(u));
            obj.last_omega_0 = obj.omega0;
            obj.last_omega_c = obj.omegac;
            obj.last_A0_dB = obj.A0_dB;
        end

        function y = stepImpl(obj,u,w0,wc)
            if obj.external_omega_0
                obj.omega0 = w0;
            end
            if obj.external_omega_c
                obj.omegac = wc;
            end
            obj.omega0 = max(obj.omega0,2*obj.omegac);
            if obj.is_init
                init(obj,u);
            end
            updateParams(obj);
            
            Cd = getCd(obj);
            Dd = 1;
            y = Cd*obj.x + Dd*u;
            obj.x = obj.Ad*obj.x + obj.Bd*u;
        end

        function resetImpl(obj)
            obj.x(:) = 0;
            obj.is_init = true;
        end
        
        function updateParams(obj)
            if obj.A0_dB~=obj.last_A0_dB
                setA0(obj);
                obj.last_A0_dB = obj.A0_dB;
            end
            if (obj.omega0~=obj.last_omega_0 || obj.omegac~=obj.last_omega_c)
                setAdBd(obj);
                obj.last_omega_0 = obj.omega0;
                obj.last_omega_c = obj.omegac;
            end
        end
        
        function setAdBd(obj)
            A = [0 1;
                -obj.omega0^2  -obj.omegac];
            B = [0; 1];
            disc_method = erase(obj.discretization_method,'''');
            switch disc_method
                % ZOH
                case 'zoh'
                    obj.Ad = expm2(A*obj.T);
                    obj.Bd = inv2(A) * (obj.Ad-eye(2)) * B;
                % Tustin
                case 'tustin'
                    [obj.Ad,obj.Bd] = tustin2(A,B,obj.T);
                otherwise
                    [obj.Ad,obj.Bd] = tustin2(A,B,obj.T);
                    error('Discretization method not specified correctly.')
            end
        end
        
        function Cd = getCd(obj)
            Cd = [0 obj.omegac*(obj.A0-1)];
        end
        
        function setA0(obj)
            obj.A0 = 10^(-obj.A0_dB/20);
        end
        
        function init(obj,u)
            Det = (1-obj.Ad(1,1))*(1-obj.Ad(2,2))-obj.Ad(1,2)*obj.Ad(2,1);
            obj.x(1) = ((1-obj.Ad(2,2))*obj.Bd(1)+obj.Ad(1,2)*obj.Bd(2))/Det*u;
            obj.x(2) = (obj.Ad(2,1)*obj.Bd(1)+(1-obj.Ad(1,1))*obj.Bd(2))/Det*u;
            obj.x(isnan(obj.x)) = 0;
            obj.is_init = false;
        end
        
    end
end