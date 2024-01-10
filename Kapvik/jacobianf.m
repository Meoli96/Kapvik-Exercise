function [J] = jacobianf(q)

% Jacobian for the 4 link manipulator of the Robotic Arm Exercise
% q=[q1 q2 q3 q4] :  vector of the joint angles
% The forth row is the selection of omega4(3),
% The Jacobian is written wrt to the End Effector

q1=q(1);
q2=q(2);
q3=q(3);
q4=q(4);

alpha = pi/2;
L1 = 0.44;
L2 = 0.46;

J =[ cos(alpha)*(L1*sin(q3 + q4) + L2*sin(q4)), L1*sin(q3 + q4) + L2*sin(q4), L2*sin(q4), 0;
    cos(alpha)*(L1*cos(q3 + q4) + L2*cos(q4)), L1*cos(q3 + q4) + L2*cos(q4), L2*cos(q4), 0;
    -sin(alpha)*(L2*cos(q2 + q3) + L1*cos(q2)),                            0,          0, 0;
    sin(q2 + q3 + q4)*sin(alpha),                            0,          0, 0;
    cos(q2 + q3 + q4)*sin(alpha),                            0,          0, 0;
    cos(alpha),                            1,          1, 1];


end
