function setRLink(obj,q0, thetalim, jointType, jointMass, linkmass, R, r,  Tc, G, dcmotor)
% This function sets some joints and link parameters and computes the
% remaining parameters

% Inputs:
%   thetalim: 2x1 limits of joint angle
%   jointType: 2 for spherical, 1 for revolute, 0 for prismatic
%   jointMass: mass of the joint
%   linkmass: mass of the link
%   R: outer radius of the link
%   r: inner radius of the link
%   Tc: Coulomb friction
%   G: Gear ratio
%   dcmotor: DC motor parameters
% Outputs:
%   obj: updated object with all the parameters set

obj.linkMass = linkmass;
obj.G = G;
obj.dcmotor = dcmotor;

obj.qlim = thetalim;
obj.jointType = jointType;
obj.jointMass = jointMass;
obj.Tc = Tc;

% Compute joint axix with alpha
obj.axis = rot_axis(obj.alpha);

% Compute the CoM position in joint space
% Ia is computed at [0,0,0] link frame and point mass of joint is at [a/2,0,0]


obj.cm = com_off(obj.linkMass, [obj.a/2;0;0], obj.jointMass, [0;0;0]);
Il = inertia_link(obj.linkMass, obj.a, R, r); % Central principal inertia matrix of the link
Ij = inertia_joint(obj.jointMass, obj.a/2);
Ia = Il + Ij; % Moment of inertia about a/2 (link com)
obj.I = inertia_off(Ia, obj.linkMass + obj.jointMass, obj.a/2*[1;0;0] - obj.cm); % Inertia matrix of the joint+link assembly
obj.q_off = q0;

end