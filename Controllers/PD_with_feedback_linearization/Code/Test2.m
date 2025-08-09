dt = 0.01;
T = 50;
t = 0:dt:T;
g = 9.81;
t = 0.2*t;
% xd means x desired
% x_d means x_dot
xd = 1*sin(t);
yd = 1*cos(t);
zd = 0*sin(t);
phid = 0*t;
thetad = 0*t;
psid = 0*(t);
th1d = sin(t);
th2d = cos(t);
% plot3(xd,yd,zd);
% grid on;
qd = [xd;yd;zd;phid;thetad;psid;th1d;th2d];
%% Controller

% Initial sates
q_d = [0;0;0;0;0;0;0;0];  % initial
q = [0;0;0;0;0;0;0;0];   % initial

e = qd(:,1)-q;
qd_d = (qd(:,1) - qd(:,2))/dt;
e_d = qd_d-q_d;

% K1 = 0.6; K2 = 1; % sum
Kp = [0.8,0.8,2,0.4,0.4,0.4,0.4,0.4]';
Kd = [1,1,1,4,2,2,2,2]';
% K2 = 2*sqrt(K1);
Q = [q];

Tp = [0;0];
for i = 2:size(t,2)-1
    e_dd = Kp.*e + Kd.*e_d;
    
    T_c = f_dynamics(q,q_d,e_dd);
    if(isnan(T_c))
        disp('nan');
        return;
    end
    w = Roll_pitch(q,T_c);
    
    qd(5,i) = w(1);
    qd(4,i) = w(2);
    
    qd_d = (qd(:,i+1) - qd(:,i))/dt;

    [q, q_d] = PDFB_mod_RK4(q,q_d,qd(:,i),qd_d,Kp,Kd,dt);
    
    Q = [Q,q];
    
    e = qd(:,i)-q;
    e_d = qd_d-q_d;

end

t = t(1:end-1);
qd = qd(:,1:end-1);
figure(1)
% tiledlayout(4, 2, 'TileSpacing', 'none', 'Padding', 'compact'); % Minimal space between subplots
subplot(4, 2, 1);
plot(t,Q(1, :),'LineWidth',1.5);
hold on;
grid on;
plot(t,qd(1,:),'LineWidth',1.5);
title(['q1 (x)']);
xlabel('time(s)');
% legend('actual','desired');

subplot(4, 2, 2);
plot(t,Q(2, :),'LineWidth',1.5);
hold on;
grid on;
plot(t,qd(2,:),'LineWidth',1.5);
title(['q2 (y)']);
xlabel('time(s)');
% legend('actual','desired');

subplot(4, 2, 3);
plot(t,Q(3, :),'LineWidth',1.5);
hold on;
grid on;
plot(t,qd(3,:),'LineWidth',1.5);
title(['q3 (z)']);
xlabel('time(s)');
% legend('actual','desired');

subplot(4, 2, 4);
plot(t,Q(4, :),'LineWidth',1.5);
hold on;
grid on;
plot(t,qd(4,:),'LineWidth',1.5);
title(['q4 (roll)']);
xlabel('time(s)');
% legend('actual','desired');

subplot(4, 2, 5);
plot(t,Q(5, :),'LineWidth',1.5);
hold on;
grid on;
plot(t,qd(5,:),'LineWidth',1.5);
title(['q5 (pitch)']);
xlabel('time(s)');
% legend('actual','desired');

subplot(4, 2, 6);
plot(t,Q(6, :),'LineWidth',1.5);
hold on;
grid on;
plot(t,qd(6,:),'LineWidth',1.5);
title(['q6 (yaw)']);
xlabel('time(s)');
% legend('actual','desired');

subplot(4, 2, 7);
plot(t,Q(7, :),'LineWidth',1.5);
hold on;
grid on;
plot(t,qd(7,:),'LineWidth',1.5);
title(['q7 (joint1)']);
xlabel('time(s)');
% legend('actual','desired');

subplot(4, 2, 8);
plot(t,Q(8, :),'LineWidth',1.5);
hold on;
grid on;
plot(t,qd(8,:),'LineWidth',1.5);
title(['q8 (joint2)']);
xlabel('time(s)');
% legend('actual','desired');

disp('Simulation completed');
figure(2)
plot3(Q(1,:),Q(2,:),Q(3,:));
hold on;
plot3(xd,yd,zd);
animateUAVwithManipulator(t, Q(1:6,:)', Q(7:8,:)',dt);