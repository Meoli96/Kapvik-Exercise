function [tau] = MNE(slink, q, qd, qdd, varargin)
    % Function implementing the Newton-Euler algorithm for forward dynamics
    
    % Inputs:
    % slink: structure containing the link parameters
    % q_dd: joint accelerations
    % q_d: joint velocities
    % q: joint positions
    
    % Outputs:
    % tau: joint torques
    
    % DYNAMIC  Inverse Newton-Euler for tree-structured manipulator dynamics to compute
    % joint torques for given joint positions, velocities, and accelerations.
    %
    % This function is used to retrieve the matrices actors in the forward dynamic,
    % i.e. the inertia matrix, the coriolis and centrifugal forces, the gravity
    % forces and the friction forces.
    
    % To compute M
    % tau: joint torques
    
    % number of links
    n_link = slink.n_links;
    
  
    

    % Initialize the force and moment vectors
    F = zeros(3,n_link);
    N = zeros(3,n_link);
    % Initialize the joint torques
    tau = zeros(1,n_link);
    % Initialize the external force and moment vectors acting on each joint

    
    Z_ip1 = [0; 0; 1]; % Rotation axis of link i+1 in i+1 frame, always [0;0;1]
    
    
    Rs = zeros(3,3,n_link); % Rotation matrix from frame i to frame i-1
    Ps = zeros(3,n_link); % Position of joint i in frame i-1


    omega = zeros(3,1);
    omega_d = zeros(3,1);
    vd = slink.gravity;

    if nargin == 6
        % Argument is the external force on the end effector
        fn_ext = varargin{1};
    else 
        fn_ext = zeros(1,6);
    end

    % Loop to compute rotation matrices, joint position and so on
    for i = 1:n_link
        Rs(:,:,i) = getRot(slink.transform(i,q(i)));
        Ps(:,i) = slink.link(i).a*[1,0, 0]';
    end
    
    % compute the velocity and acceleration of link i - Outward iteration
    for i = 1:n_link
        link_t = slink.link(i);
        Pcm = link_t.cm;
        R = Rs(:,:,i)'; % Rotation matrix from frame i-1 to frame i 
        P_ip1 = Ps(:,i); % Position of joint i in frame i-1
        
        
        % compute the angular velocity of link i+1
        omega_tmp = R*omega + qd(i)*Z_ip1;
        
        % compute the acceleration of link i
        omegad_tmp = R*omega_d + R*cross(omega, qd(i)*Z_ip1)+ qdd(i)*Z_ip1;
        
        % compute the linear acceleration of joint and com of link i
        vd_tmp = R*(cross(omega_d, P_ip1) + cross(omega, cross(omega, P_ip1)) + vd);
        
        % Update the angular velocity and acceleration of link i
        omega = omega_tmp;
        omega_d = omegad_tmp;
        vd = vd_tmp;
        
        vd_cm = cross(omega_d, Pcm) + cross(omega, cross(omega, Pcm)) + vd;
        
        
        mm = link_t.linkMass + link_t.jointMass;
        % compute the inertial force and moment acting on link i
        F(:,i) = mm.*vd_cm;
        N(:,i) = link_t.I*omega_d + cross(omega, link_t.I*omega);
    end
    
    % compute the force and moment acting on each joint - Inward iteration
    
    % Assign external forces to end effector
    f = fn_ext(1:3)';
    nn = fn_ext(4:6)';
    for i = n_link:-1:1
        
        link_t = slink.link(i);
        if i == n_link
            R = eye(3,3);
            P_ip1 = [0;0;0];
        else
            R = Rs(:,:,i+1);
            P_ip1 = Ps(:,i+1);
        end
        Pcm = link_t.cm;
  


        f_tmp = R*f + F(:,i);
        n_tmp = N(:, i) + R*nn + cross(Pcm, F(:, i)) + cross(P_ip1, R*f);
        
        f = f_tmp;
        nn = n_tmp;

        tau(i) = nn'*Z_ip1 + link_t.G^2*link_t.dcmotor.Jm*qdd(i) - link_t.friction(qd(i));
    end
    
    
    end
    
    
    


    