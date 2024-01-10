%% Initialization
clc; clear all;

density = 3.5 * 1000; % g/cm^3 to kg/m^3


a1 = 0.44;
a2 = 0.46;
R = 20*10^-3;
r = 18*10^-3;

vol1 = pi*(R^2 - r^2)*a1; % m^3
vol2 = pi*(R^2 - r^2)*a2; % m^3

% Here data is packed as follows:
% [thetalim, jm, lm, R, r, Tc, G]
% where:
% thetalim - joint limits
% jm - joint mass
% lm - link inertia
% R - External radius of hollow tube
% r - Internal radius of hollow tube
% Tc - Coulomb friction --- Tc(1) if qd<0 Tc(2) if qd>0
% G - gear ratio


% Azimuthal joint at base
thetalim0 = [-160, 100];
jm0 = 1.15;
lm0 = 0;
R0 = 0;
r0 = 0;
Tc0 = [-5*10^-4, 1.5*10^-4];
G0 = 8400;
q00 = 0; % x_az parallel to direction of motion of the rover

% Elevation joint at base
thetalim1 = [-90, 90];
jm1 = 1.28;
lm1 = 0;
R1 = 0;
r1 = 0;
Tc1 = [-5*10^-4, 1.5*10^-4];
G1 = 8400;
q01 = 90;

% Elbow joint
thetalim2 = [-150, 110];
jm2 = 1.39;
lm2 = vol1*density;
R2 = R;
r2 = r;
Tc2 = [-5*10^-4, 1.5*10^-4];
G2 = 5300;
q02 = 180;

% Wrist joint
thetalim3 = deg2rad([-90, 5]);
jm3 = 0.67;
lm3 = vol2*density;
R3 = R;
r3 = r;
Tc3 = [-5*10^-4, 1.5*10^-4];
G3 = 6700;
q03 = 0;

% Create DCmotor object
MaxonDC = DCMotor(8.67*10^-3, 0.84, 1.44*10^-6, 5.3*10^-6, 10300);

% Define the base dimensions
x_b = 0.85;  % meters
y_b = 0.78;   % meters
z0_b=0.31;

% Calculate the coordinates of the vertices
x_v = [-x_b/2, x_b/2, x_b/2, -x_b/2, -x_b/2];
y_v = [-y_b/2, -y_b/2, y_b/2, y_b/2, -y_b/2];
z_v = [z0_b, z0_b, z0_b, z0_b, z0_b];  % Assuming a flat rectangular plot




% Create the links objects
l0 = RLink(0, 0, 0, 0); % Azimuthal joint
l1 = RLink(0, pi/2, 0, 0); % Elevation joint
l2 = RLink(a1, 0, 0, 0); % Elbow joint
l3 = RLink(a2, 0, 0, 0); % Wrist joint

% Set the links properties
setRLink(l0, deg2rad(q00), thetalim0, 'R', jm0, lm0, R0, r0, Tc0, G0, MaxonDC);
setRLink(l1, deg2rad(q01), thetalim1, 'R', jm1, lm1, R1, r1, Tc1, G1, MaxonDC);
setRLink(l2, deg2rad(q02), thetalim2, 'R', jm2, lm2, R2, r2, Tc2, G2, MaxonDC);
setRLink(l3, deg2rad(q03), thetalim3, 'R', jm3, lm3, R3, r3, Tc3, G3, MaxonDC);

% Create chain
slink = RSLink([l0 l1 l2 l3]);
slink.setGravity([0 0 3.37]);

% Set ikine function
slink.setIkine(@ikine);
% Set Jacobian function
slink.setJhandle(@jacobianf);




% Toolbox code initialization --- FOR TEACH ONLY



RTool = SerialLink( [ RevoluteMDH('a', 0) ...            %%% Shoulder azimuth
    RevoluteMDH('a', 0, 'alpha', pi/2) ...  %%% Shoulder elevation
    RevoluteMDH('a', 0.44) ...                 %%% Elbow
    RevoluteMDH('a', 0.46)], ...               %%% Wrist
    'name', 'RTool')

RTool.offset = deg2rad([0, 90, 180, 0]);

% Define the dimensions
x = 0.85;  % meters
y = 0.78;   % meters
z0=0.31;

% Calculate the coordinates of the vertices
x = [-x/2, x/2, x/2, -x/2, -x/2];
y = [-y/2, -y/2, y/2, y/2, -y/2];
z = [z0, z0, z0, z0, z0];  % Assuming a flat rectangular plot



robot.base = [0.3 0 z0];


%% Test fkine (funzionante)
q = [0, 90, 0 ,0];
T_mio = slink.fkine(q);

%% Ikine for final joint positions


% Navigation
x_nav = [-0.6,0,0,0];
q_res_nav = slink.ikine(x_nav, "eu", "af");
q_res_nav = slink.translate_ref(q_res_nav');
for i = 1:length(q_res_nav)
    q_res_nav(i) = mod(q_res_nav(i),2*pi());
    if (q_res_nav(i) > pi)
        q_res_nav(i) = q_res_nav(i) - 2*pi;
    end
end
RTool.teach(q_res_nav)
x_nav
q_res_nav

% Sample retrieval
x_sample_ret = [0.6,0,-z0, 0];
q_res_sample = slink.ikine(x_sample_ret, "eu", "ab");
q_res_sample  = slink.translate_ref(q_res_sample');
for i = 1:length(q_res_sample )
    q_res_sample (i) = mod(q_res_sample (i),2*pi());
    if (q_res_sample (i) > pi)
        q_res_sample (i) = q_res_sample (i) - 2*pi;
    end
end
RTool.teach(q_res_sample)
x_sample_ret
q_res_sample

% Sample transfer
x_sample_trans = [0.1, 0.4, 0, 0];
q_res_trans = slink.ikine(x_sample_trans, "eu", "ab");
q_res_trans  = slink.translate_ref(q_res_trans');
for i = 1:length(q_res_trans )
    q_res_trans (i) = mod(q_res_trans (i),2*pi());
    if (q_res_trans (i) > pi)
        q_res_trans (i) = q_res_trans (i) - 2*pi;
    end
end
RTool.teach(q_res_trans)
x_sample_trans
q_res_trans




hold off;




%% Test forward dynamic
q = deg2rad([0,90,0,0]);
q_tool = deg2rad([0,180,180,0]);
qdot = deg2rad([60, 60,60, 60]);
qddot = deg2rad([0.1, 0.1, 0.1, 0.1]);

Mmio = slink.get_JSIM(q)

CMio = slink.get_C(q, qdot)  

GMio = slink.get_G(q)

Fmio = slink.get_F(qdot)




%% CONTROL LAW
% In this part we are defining the control law partioning inside the
% control function.

%% Stowage to Navigation
kp1=100;
kd1=2*sqrt(kp1); % To obtain critical damping

acc1=0.005*([0.7168 0.7168 1.1360 0.8986]);
freq=0.01;
sigma=deg2rad(0.025);

tf1=[45,30,30];
q1=deg2rad([0 0 0 0 ]);
q2=deg2rad([ 90    0    0    40.37]);
q3=deg2rad([0   0   -65   -83.58]);
q4=deg2rad([0 0 0 0]);


[q_out, q_dout, q_ddout, t_traj] = slink.ftraj_generator_viapoints(q1, q2, q3, q4,acc1, tf1, freq);

q0(1,:)=q_out(1,:)-(normrnd(q_out(1,:),3*sigma)-q_out(1,:));
qd0(1,:)=q_dout(1,:);
t_traj=t_traj(:,1);


options = odeset('RelTol', 1.0e-4, 'AbsTol', 1.0e-4);

for i=1:length(t_traj)-1

    E1(i,:)=[q_out(i,:)'-q0(i,:)';q_dout(i,:)'-qd0(i,:)'];
    [tint,X,tau] = ode113(@(ode_time,E1)fcontrol(ode_time,E1, q_out, q_dout, q_ddout, t_traj, kp1, kd1, slink), [t_traj(i) t_traj(i+1)], [q_out(i,:)'-q0(i,:)'; q_dout(i,:)'-qd0(i,:)'], options);
    row=length(X);
    q0(i+1,:)=q_out(i+1,:)-X(row,1:4);
    qd0(i+1,:)=q_dout(i+1,:)-X(row,5:8);
    if i==1
        error=E1(1,:)'
    end

end

figure

plot(t_traj,rad2deg(q_out-q0),'LineWidth',1.5)
legend('Joint 1','Joint 2','Joint 3','Joint 4')
xlabel('Time [s]'),ylabel('Error [deg]')
title('Control of the trajectory from Stowage to Navigation')
grid on

%% Stowage to Sample Retrieval
kp2=100;
kd2=2*sqrt(kp2); % To obtain critical damping

acc2=0.005*([0.7168 0.7168 1.1360 0.8986]);
freq=0.01;
sigma=deg2rad(0.025);

tf2=[45,1,45];
q1=deg2rad([0 0 0 0]);
q2=deg2rad([90   0  0  -74.81]); 
q3=deg2rad([0   0  0   97.22]);
q4=deg2rad([0 0 0 0]);

% q2=deg2rad([90   10  -40  -70.2]); 
% q3=deg2rad([0   20  60   78]);
[q_out, q_dout, q_ddout, t_traj] = slink.ftraj_generator_viapoints(q1, q2, q3, q4, acc2,tf2, freq);

q0(1,:)=q_out(1,:)-(normrnd(q_out(1,:),3*sigma)-q_out(1,:));
qd0(1,:)=q_dout(1,:);
t_traj=t_traj(:,1);


options = odeset('RelTol', 1.0e-4, 'AbsTol', 1.0e-4);

for i=1:length(t_traj)-1

    E2(i,:)=[q_out(i,:)'-q0(i,:)';q_dout(i,:)'-qd0(i,:)'];
    [tint,X,tau] = ode113(@(ode_time,E2)fcontrol(ode_time,E2, q_out, q_dout, q_ddout, t_traj, kp2, kd2, slink), [t_traj(i) t_traj(i+1)], [q_out(i,:)'-q0(i,:)'; q_dout(i,:)'-qd0(i,:)'], options);
    row=length(X);
    q0(i+1,:)=q_out(i+1,:)-X(row,1:4);
    qd0(i+1,:)=q_dout(i+1,:)-X(row,5:8);
    if i==1
        error=E2(1,:)'
    end

end

figure
plot(t_traj,rad2deg(q_out-q0),'LineWidth',1.5)
legend('Joint 1','Joint 2','Joint 3','Joint 4')
xlabel('Time [s]'),ylabel('Error [deg]')
title('Control of the trajectory from Stowage to Sample Retrieval')
grid on

%% Sample Retireval to Sample  Transfer
kp3=100;
kd3=2*sqrt(kp3); % To otain crituca damping

acc3=0.006*([0.7168 0.7168 1.1360 0.8986]);
freq=0.01;
sigma=deg2rad(0.025);

tf3=[30,1,40];
q1=deg2rad([0 0 0 75.96]);
q2=deg2rad([-74.81  -24.76  -24.76  -24.76]);
q3=deg2rad([97.22   54.47  54.47  54.47]);
q4=deg2rad([0 0 0 0]);

[q_out, q_dout, q_ddout, t_traj] = slink.ftraj_generator_viapoints(q1, q2, q3, q4,acc3, tf3, freq);
q0(1,:)=q_out(1,:)-(normrnd(q_out(1,:),3*sigma)-q_out(1,:));
qd0(1,:)=q_dout(1,:);
t_traj=t_traj(:,1);


options = odeset('RelTol', 1.0e-4, 'AbsTol', 1.0e-4);

for i=1:length(t_traj)-1

    E3(i,:)=[q_out(i,:)'-q0(i,:)';q_dout(i,:)'-qd0(i,:)'];
    [tint,X,tau] = ode113(@(ode_time,E3)fcontrol(ode_time,E3, q_out, q_dout, q_ddout, t_traj, kp3, kd3, slink), [t_traj(i) t_traj(i+1)], [q_out(i,:)'-q0(i,:)'; q_dout(i,:)'-qd0(i,:)'], options);
    row=length(X);
    q0(i+1,:)=q_out(i+1,:)-X(row,1:4);
    qd0(i+1,:)=q_dout(i+1,:)-X(row,5:8);

    if i==1
        error=E3(1,:)'
    end

end

figure
plot(t_traj,rad2deg(q_out-q0),'LineWidth',1.5)
legend('Joint 1','Joint 2','Joint 3','Joint 4')
xlabel('Time [s]'),ylabel('Error [deg]')
title('Control of the trajectory from Sample Retrieval to Sample Transfer')
grid on
