% Equivalent of SerialLink object - collection of links
classdef RSLink < handle & matlab.mixin.Copyable
    properties
        gravity = [0; 0; 3.37];
        link
        n_links
        closed_ikine_fun
        j_fun
        q0sl % New [0,0,0,0] for this robot in deployed arm frame
    end
    methods
        function obj  = RSLink(links)
            obj.link = links;
            obj.n_links = length(links);
            for i = 1:obj.n_links
                obj.q0sl(i) = links(i).q_off;
            end
        end

        function [q_t] = translate_ref(obj, q)
            % Transform q from Genova's [0,0,0,0] pose to my [0,0,0,0] pose
            assert(length(q) == length(obj.q0sl) && numrows(q) == numrows(obj.q0sl), "q and ref1 should have the same dimension");
            q_t = q - obj.q0sl;
        end

        function T = fkine(obj, q)
            T = eye(4);
            for i = 1:obj.n_links
                T = T * obj.link(i).T(q(i));
            end
        end
        
        function obj = setIkine(obj, ifun)
            obj.closed_ikine_fun = ifun;
        end
        
        function q = ikine(obj, target, varargin)
            % X cartesian coordinates of end effector
            % fun is a function that returns thetas for a given X
            % q is a vector of joint angles in radians
            
            % returns [0] vector if joint limits are exceeded
            
            if nargin > 2
                q = obj.closed_ikine_fun(target, varargin{:});
            else 
                q = obj.closed_ikine_fun(target);
            end
        end
        
        function obj = setGravity(obj, g)
            assert(length(g) == 3);
            if (numcols(g) == 3) 
                g = g';
            end
            obj.gravity = g;
        end
        
        function robot = nofriction(obj)
            % Copy the robot and set no friction values
           l = [];
            for i = 1:obj.n_links
                link_iter = copy(obj.link(i));
                link_iter.Tc = [0 0];
                l = [l link_iter];
            end
            robot = RSLink(l);
        end

        function obj = setJhandle(obj, jh)
            % Set the joint handles for the robot
            obj.j_fun = jh;
        end
        
        function J = jacobian(obj, q)
            J = obj.j_fun(q);
        end
        
        function T = base_transform(obj, i, q)
            % Get transform from link i to base
            assert(i <= obj.n_links, "i should be less or equal to the number of links")
            T = eye(4);
            for idx = 1:i
                T = T * obj.link(idx).T(q(idx));
            end
        end
        
        function T = transform(obj, i, q_i)
            % Get transform from i to i-1 
            T = obj.link(i).T(q_i);
        end
        
        function axis = axis(obj, i)
            axis = obj.link(i).axis';
        end
        
        
        function tau = idyne(obj, q, qd, qdd)
            % This function computes the inverse dynamics of the manipulator
            % tau = idyne(q, qd, qdd)
            % q: joint angles
            % qd: joint velocities
            % qdd: joint accelerations
            % tau: joint torques
            
            tau = MNE(obj, q, qd, qdd);
        end
        
        function M = get_JSIM(obj, q)
            n = obj.n_links;
            M = zeros(n, n);            
            % Set qdd_j = 1 and qdd_i = 0 for i ~= j
            for j = 1:n
                qdd_j = zeros(n, 1);
                qdd_j(j) = 1;
                tau = MNE_wg(obj, q, zeros(n, 1), qdd_j);
                M(:, j) = tau;
            end
        end
        
        function C = get_C(obj, q, qd)
            % This function computes the coriolis matrix 
            n = obj.n_links;
            % We need to leave friction aside
            robot_copy = obj.nofriction();

            C = zeros(n, n);
            C_cen = zeros(n, n); % Centripetal terms

            for i = 1:n
                % Compute centripetal terms
                qd_fake = zeros(1, n);
                qd_fake(1) = 1;

                tau = MNE_wg(robot_copy, q, qd_fake, zeros(n, 1));
                C_cen(:, i) = C_cen(:,i) + tau';  
            end

            % Coriolis terms 
            for i = 1:n
                for j = i+1:n
                qd_fake = zeros(1, n);
                qd_fake(i) = 1;
                qd_fake(j) = 1;

                tau = MNE_wg(robot_copy, q, qd_fake, zeros(n, 1))';
                C(:, j) = C(:,j) + (tau - C_cen(:, j) - C_cen(:, i))*qd(i)/2;
                C(:, i) = C(:,i) + (tau - C_cen(:, j) - C_cen(:, i))*qd(j)/2;
                
                end
            end

            C = C + C_cen*diag(qd);
        end

        function G = get_G(obj, q)
            % This function computes the gravity matrix
            % G = gravity(q)
            % q: joint angles
            % G: gravity vector
            G = MNE(obj, q, zeros(obj.n_links, 1), zeros(obj.n_links, 1));
            
        end
        
        function F  = get_F(obj, qd)
            % This function computes the friction torque of the manipulator
            % F = friction(qd)
            % qd: joint velocities
            % F: friction torque
            F = zeros(obj.n_links, 1);
            for i = 1:obj.n_links
                F(i) = obj.link(i).friction(qd(i));
            end
        end

 
        function [qdd_max] = get_qdd_max(obj)
            % This function computes the maximum joint acceleration of the manipulator
            % qdd_max = get_qdd_max(q, qd, tau_max)
            % q: joint angles
            % qd: joint velocities
            % tau_max: maximum joint torques
            % qdd_max: maximum joint acceleration
            
            qdd_max = zeros(obj.n_links, 1);
            n = obj.n_links;
            for i = 1:n
                qdd_max(i) = obj.link(i).qdd_lim();
            end
        end
        
        function tau = pd_control(obj, q, qd, q_des, qd_des, qdd_des, Kp, Kd)
            % This function computes the joint torques for a PD controller
            % tau = pd_control(q, qd, q_des, qd_des, qdd_des, Kp, Kd)
            % q: joint angles
            % qd: joint velocities
            % q_des: desired joint angles
            % qd_des: desired joint velocities
            % qdd_des: desired joint accelerations
            % Kp: proportional gain
            % Kd: derivative gain
            % tau: joint torques
            
           
        end
        
        function [q_out,q_dout,q_ddout,t_traj_out] = ftraj_generator_viapoints(obj, q0,q1,q2,q3,acc,tf,freq)
        
        %--------------------------------------------------------------------------
        % Function to compute the trajecotry of the manipulator joints using 4 via
        % points. Cannot use more or less.
        %--------------------------------------------------------------------------
        % Inputs:
        % q0,...,q3     :   Joint configurations, this mean that q0 contail all the
        %                   pose of the first joint and so on. [rad]!!
        % acc           :   Acceleration for each joint. Must be a vector
        %                   [rad/s]!!!
        % tf            :   Time of movement from q0(1)-->q0(2) and so on
        % freq          :   time step taken to generate trajecotry
        %--------------------------------------------------------------------------
        % Outputs:
        % q_out         : Matrix [length(q0)*1/freq ]x length(q0) of position
        % q_dout        : Matrix [length(q0)*1/freq ]x length(q0) of velocity
        % q_ddout       : Matrix [length(q0)*1/freq ]x length(q0) of acceleration
        % t_traj        : Vector ot times for each joint
        % All plots are in degree
        %--------------------------------------------------------------------------
        
        q_mat=[q0',q1',q2',q3']; % Each colum is a joint, each row is a pose of the joint
        
        
        for i=1:length(q_mat(1,:))
            x=[];
            x_d=[];
            x_dd=[];
            
        
            %t_app=[];
        
            for j=1:length(q_mat(:,1))-1
                
                
                [q,q_d,q_dd,t_traj] = ftraj_generator(q_mat(j,i),q_mat(j+1,i),tf(j),acc(i),freq);
                x=[x;q];
                x_d=[x_d;q_d];
                x_dd=[x_dd;q_dd];
        
               
               
            end
            q_out(:,i)=x;
            q_dout(:,i)=x_d;
            q_ddout(:,i)=x_dd;
        
            t_traj_out(:,i)=linspace(0,sum(tf),length(q_out)); %length(tf)*1/freq
            figure(i)
    
            subplot(3,1,1)
            plot(t_traj_out,rad2deg(q_out(:,i)),'LineWidth',1.5)
            title('Joint position'), xlabel('time [s]'), ylabel('[deg]')
            subplot(3,1,2)
            plot(t_traj_out,rad2deg(q_dout(:,i)),'LineWidth',1.5)
            title('Joint velocity'), xlabel('time [s]'), ylabel('[deg/s]')
            subplot(3,1,3)
            plot(t_traj_out,rad2deg(q_ddout(:,i)),'LineWidth',1.5)
            title('Joint acceleration'), xlabel('time [s]'), ylabel('[deg/s^2]')
                
        end
        
        
        end
        
                
        
        
        
        
    end % methods end
end % class end