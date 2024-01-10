classdef RLink < handle & matlab.mixin.Copyable
    properties
        % DH parameters
        a   % link length
        alpha % link twist
        d   % link offset
        q % joint angle
        
        % joint parameters
        qlim % 2x1 limits of joint angle
        jointType %2 for spherical, 1 for revolute, 0 for prismatic
        axis % 3x1 axis of rotation
        jointMass % mass of the joint
        
        % link parameters
        linkMass     % link mass
        
        
        cm % 3x1 CoM position
        I % Inertia matrix
        Tc % Coulomb friction
        G % Gear ratio
        
        dcmotor

        q_off  % Offset from 0 --- new 0
        
    end
    
    methods
        % Constructor, initializes only the DH parameters
        function obj = RLink(a, alpha, d, theta)
            % MDH parameters
            obj.a = a;
            obj.alpha = alpha;
            obj.d = d;
            obj.q = theta;
        end
        
      
        % Method to get the transformation matrix of the link
        function T = T(obj, theta_t)
            % Method to get the transformation matrix of the link - MDH formulation
            theta = theta_t + obj.q_off; % Add the offset position to the joint angle
            T=[cos(theta),-sin(theta), 0 ,obj.a;
                sin(theta)*cos(obj.alpha), cos(theta)*cos(obj.alpha), -sin(obj.alpha), -sin(obj.alpha)*obj.d;
                sin(theta)*sin(obj.alpha),cos(theta)*sin(obj.alpha),cos(obj.alpha),cos(obj.alpha)*obj.d;
                0,0,0,1];
        end

        function [qdd_max] = qdd_lim(obj)
            % This function returns the maximum joint acceleration given:
            % - the maximum motor acceleration
            % - the gear ratio
            
            % !!! Assuming high gear ratio (G >> 1) !!!
            
            qdd_m_max = obj.dcmotor.get_qdd_max();
            qdd_max = qdd_m_max/obj.G;
        end

        function tau = friction(obj, qd) 
            % This function returns the Coulomb friction force seen from the output
            % of the gearbox (scaled by gear ratio) 
            tau = 0;
            if qd < 0
                % Negative friction
                tau = obj.Tc(1);
            elseif qd > 0
                % Positive friction
                tau = obj.Tc(2);
            else
                tau = 0;
            end
            tau = -obj.G*tau;
        end
        
        function displayInfo(obj) 
            % Method to display the link parameters
            disp('Link parameters:');
            disp(['a: ', num2str(obj.a)]);
            disp(['alpha: ', num2str(obj.alpha)]);
            disp(['d: ', num2str(obj.d)]);
            disp(['theta: ', num2str(obj.q)]);
            disp('Joint parameters:');
            disp(['qlim: ', num2str(obj.qlim)]);
            disp(['jointType: ', num2str(obj.jointType)]);
            disp(['axis: ', num2str(obj.axis)]);
            disp(['jointMass: ', num2str(obj.jointMass)]);
            disp('Link parameters:');
            disp(['linkMass: ', num2str(obj.linkMass)]);
            disp(['cm: ', num2str(obj.cm)]);
            disp(['I: ', num2str(obj.I)]);
            disp(['Tc: ', num2str(obj.Tc)]);
            disp(['G: ', num2str(obj.G)]);
        
        end

    end
end