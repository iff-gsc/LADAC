classdef NotchFilter < matlab.System
    properties
        % Central rejected frequency (rad/s)
        omega_0	= 2*pi*80;
        % Width of rejected frequency (rad/s)
        omega_c	= 2*pi*40;
        % Amplitude reduction at rejected frequency (dB)
        A0_dB	= 40;
        % Sample time (s)
        Ts      = 1/400;
    end
    properties(DiscreteState)
        x
    end
    properties(Access = private)
        Ad
        Bd
        T = 1/400;
        A0
        last_omega_0;
        last_omega_c;
        last_A0_dB;
        is_init         = true;
    end
    methods(Access = protected)
        function setupImpl(obj)
            obj.A0 = 10^(-obj.A0_dB/20);
            setAd(obj);
            setBd(obj);
            obj.x = zeros(2,1);
            obj.last_omega_0 = obj.omega_0;
            obj.last_omega_c = obj.omega_c;
            obj.last_A0_dB = obj.A0_dB;
        end

        function y = stepImpl(obj,u)
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
            if (obj.omega_0~=obj.last_omega_0 || obj.omega_c~=obj.last_omega_c)
                setAd(obj);
                setBd(obj);
                obj.last_omega_0 = obj.omega_0;
                obj.last_omega_c = obj.omega_c;
            end
        end
        
        function setAd(obj)
            A = [0 1;
                -obj.omega_0^2  -obj.omega_c];
            obj.Ad = expm2(A*obj.T);
        end
        
        function setBd(obj)
            A = [0 1;
                -obj.omega_0^2  -obj.omega_c];
            B = [0; 1];
            obj.Bd = inv2(A) * (obj.Ad-eye(2)) * B;
        end
        
        function Cd = getCd(obj)
            Cd = [0 obj.omega_c*(obj.A0-1)];
        end
        
        function setA0(obj)
            obj.A0 = 10^(-obj.A0_dB/20);
        end
        
        function init(obj,u)
            Det = (1-obj.Ad(1,1))*(1-obj.Ad(2,2))-obj.Ad(1,2)*obj.Ad(2,1);
            obj.x(1) = ((1-obj.Ad(2,2))*obj.Bd(1)+obj.Ad(1,2)*obj.Bd(2))/Det*u;
            obj.x(2) = (obj.Ad(2,1)*obj.Bd(1)+(1-obj.Ad(1,1))*obj.Bd(2))/Det*u;
            obj.is_init = false;
        end
        
    end
end