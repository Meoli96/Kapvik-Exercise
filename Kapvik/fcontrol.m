function [dE,tau] = fcontrol(t, E, q, qd, qdd, t_traj, kp, kd, slink)



qi=interp1(t_traj,q,t);
qi_d=interp1(t_traj,qd,t);
qi_dd=interp1(t_traj,qdd,t);

Q(1:4)=qi'-E(1:4);
Q(5:8)=qi_d'-E(5:8);

% Here I need to define the ''hat'' matrices of
M = slink.get_JSIM(Q(1:4)); 
F  = slink.get_F(Q(5:8));
C = slink.get_C(Q(1:4), Q(5:8));
G = slink.get_G(Q(1:4));

% Define the error vector
e=qi'-Q(1:4)';
ed=qi_d'-Q(5:8)';
 
% And now the controller
taup = qi_dd' + kd*ed + kp*e;
for i = 1:length(taup)
    if abs(taup(i)) >  0.00867
        taup(i) = 0.00867*sign(taup(i)); % Locking the torque to the max allowed
    end
end


% Once we ha step by step the knoledge of those matrices we can compute the
% control 
tau = M*taup + C*Q(5:8)'+ G - F;



%%% Error Dynamic equation %%%
Edd = qi_dd' - ((M)\(- C*Q(5:8)' -G + F + tau));

dE(1) = E(5);
dE(2) = E(6);
dE(3) = E(7);
dE(4) = E(8);
dE(5) = Edd(1);
dE(6) = Edd(2);
dE(7) = Edd(3);
dE(8) = Edd(4);
dE = dE';





end

