function [q] = ikine(target, varargin)
% This script computes the inverse kinematics for the manipulator
% assigned. We discard roll and yaw as not relevant for the task, in
% this way we have 4 DOF in both joint and cartesian space
% target is a 4x1 vector containing the x, y, z and pitch of the end
% effector
% lLengths is a 2x1 vector containing the length of the links
% theta1 off plane angle
% theta2 theta3 theta4 in plane angles
% varargin containts elbow up (eu) or elbow down (ed)

% af keeps the azimuth direction in the opposite semiplane of the end
% effector

% ab keeps the azimuth direction in the same semiplane of the end effector

x = target(1);
y = target(2);
z = target(3);
pitch = deg2rad(target(4));




conf_elbow = "eu"; % Default is elbow up
conf_azimut = "af"; % Default is azimuth forward
fact_elbow = 1; % Dont ask

if nargin == 3
    out_varargin1 = varargin{1};
    conf_elbow = string(out_varargin1(1));
    out_varargin2 = varargin{2};
    conf_azimut = string(out_varargin2(1));
end

if nargin == 2
    out_varargin = varargin{1};
    conf_elbow = string(out_varargin(1));
end

if strcmp(conf_azimut, "af")
  
    fact_elbow = 1;
    z = -z;
elseif strcmp(conf_azimut, "ab")
    
    fact_elbow = -1;
end
q = zeros(4,1);

L1 = 0.44;
L2 = 0.46;


%% Azimuthal plane (x-y)

% q1
q(1) = atan2(y, x);
if strcmp(conf_azimut, "af")
    % keep the azimuth between the first and second quadrant
    q(1) = q(1) + pi;
elseif strcmp(conf_azimut, "ab")
    % do nothing
else
    error("Invalid option %s", conf_azimut)
    
end
    


r_xy = sqrt(x^2 + y^2);



% Define x_prime for the in-plane equations
x_prime = r_xy;

%% In plane kine
r_x_prime_z2 = x_prime^2 + z^2;

% q3
cos_q3 = (r_x_prime_z2 - (L1^2 + L2^2))/(2*L1*L2);

if cos_q3 > 1 || cos_q3 < -1
    disp("Target out of reach")
    q = zeros(4,1); 
    return
end
if strcmp(conf_elbow, "eu")
    sin_q3 = fact_elbow*sqrt(1 - cos_q3^2);
elseif strcmp(conf_elbow, "ed")
    sin_q3 = -fact_elbow*sqrt(1-cos_q3^2);
end

q(3) = atan2(sin_q3, cos_q3); % q3

% q2

k1 = L1 + L2*cos_q3;
k2 = L2*sin_q3;

q(2) = atan2(z, x_prime) - atan2(k2, k1); % q2
% q4
q(4) = atan2(sin(pitch), cos(pitch)) - (q(2) + q(3));

% Final adjustments for azimuthal angle

if strcmp(conf_azimut, "af")
    % keep the azimuth between the first and second quadrant
    q(2) = q(2) + pi;
elseif strcmp(conf_azimut, "ab")
    % do nothing
else
    error("Invalid option %s", conf_azimut)
    
    
end
    

end


