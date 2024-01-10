function [q,q_d,q_dd,t] = ftraj_generator(q0,qf,tf,acc,freq)

% Function to generate a trajectory with trapezoidal blend from a starting
% position of singular joint to end position of same jopint
% -------------------------------------------------------------------------
% Inputs: The angles can be taken as [deg] or [rad]
% q0    :   Starting joint position 
% qf    :   End joint positions 
% tf    :   Time to go from q0 to qf [s]
% ac    :   Acceleration of single joint [rad/s]
% freq  :   Frequency of refernece trajectory [1/s]
% qd0   :   velocity last joint iteration
% -------------------------------------------------------------------------
% Outputs 
% q     : Joint position 
% q_d   : Joint velocity 
% q_dd  : Joint acceleration [rad/s^2]
% t     : Time span of trajectory 
% -------------------------------------------------------------------------
n_step=1/freq;
t=linspace(0,tf,n_step); 

if acc< 4*(qf-q0)/tf^2
    fprintf('No linear blend, try differnt acceleration of time tf\n')
    q=NaN;
    q_d=NaN;
    q_dd=NaN;
    return 
end
if qf-q0<0
    acc=-acc;
elseif qf-q0==0
    acc=0;
    q=linspace(q0,q0,length(t))';
    q_d=linspace(0,0,length(t))';
    q_dd=linspace(0,0,length(t))';
    return
end


tc=abs(tf/2-1/2*sqrt((tf^2*acc-4*(qf-q0))/acc)); % Parabolic time

% Preallocating the output vectors
% q=[q0;zeros(length(t)-2,1);qf];
% q_d=[qd0;zeros(length(t)-2;);
% q_dd=[acc;zeros(length(t)-2,1);-acc];

q=zeros(length(t),1);
q_d=zeros(length(t),1);
q_dd=zeros(length(t),1);

for i=1:length(t) %2:length(t)

    if t(i)<=tc % First Parabolic belnd
        q(i)=q0+1/2*acc*t(i)^2;
        q_d(i)=acc*t(i);
        q_dd(i)=acc;

    elseif tc<t(i) && t(i)<=(tf-tc) % Linear blend
        q(i)=q0 + acc*tc*(t(i)-tc/2);
        q_d(i)=acc*tc;
        q_dd(i)=0;

    elseif (tf-tc)<t(i) && t(i)<=tf % Last parabolic blend
        q(i)=qf-1/2*acc*(t(i)-tf)^2;
        q_d(i)=-acc*(t(i)-tf);
        q_dd(i)=-acc;
    end
end


end