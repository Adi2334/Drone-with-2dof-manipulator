dt = 0.1;
T = 50;
t = 0:dt:T;
g = 9.81;
t = 0.2*t;
% xd means x desired
% x_d means x_dot

xd = 0*sin(t);
yd = 0*cos(t);
zd = 0*(t);
phid = 0.*t;
thetad = 0.*t;
psid = 0.*t;
th1d = 0.1*t;
th2d = 0.1*t;
% plot3(xd,yd,zd);
% grid on;
qd = [xd;yd;zd;phid;thetad;psid;th1d;th2d];

%% Controller

% Initial sates
q_d = [0;0;0;0;0;0;0;0];  % initial
q = [0;0;0;0;0;0;0;0];   % initial

e = -qd(:,1)+q;
E = [e];
qd_d = (qd(:,1) - qd(:,2))/dt;
e_d = qd_d-q_d;

% ----------------Sliding mode controller---------------
lamd = [1, 1, 1, 0.01, 0.01, 0.01, 20, 20]';
K = [0.0, 0.0, 0.0, 0, 0, 0.0, 0, 0]';
A = [0.1, 0.1, 0.3, 0, 0, 0, 0.0, 0.0]';
% lamd = 1 * ones(8,1);
% K = 1*ones(8,1);
% A = 1*ones(8,1);
s = e_d+lamd.*e;
qr_d_old = zeros(8,1);
qr_d = qd_d-lamd.*e;
qr_dd = (qr_d - qr_d_old)/dt;

s_int = s;
% ------------------------------------------------------

Q = [q];
threshold = nan;
Tp = [0;0];
for i = 2:size(t,2)-1
    
    T_c = asmc(q,q_d,qr_d, qr_dd, A, K, s, s_int);
    T_c(T_c<-threshold) = -threshold;
    T_c(T_c>threshold) = threshold;
    % T_c
    % q
    if(isnan(T_c))
        % T_c = zeros(8,1);
        disp('nan');
        return;
    end

    w = Roll_pitch(q,T_c);
    qd(5,i) = w(1); qd(4,i) = w(2);

    [M,C,G] = Matrices(q,q_d);
    M;
    if (isnan(M))
        break;
    end
    [q, q_d] = f_inverse_dyna_RK4(T_c,q,q_d,dt);
    
    Q = [Q,q];
    
    qd_d = (qd(:,i+1) - qd(:,i-1))/(2*dt);
    e = -qd(:,i)+q;
    E = [E,e];
    e_d = qd_d-q_d;

    s = e_d+lamd.*e;
    s_int = s_int + s;
    qr_d_old = qr_d;
    qr_d = qd_d-lamd.*e;
    qr_dd = (qr_d - qr_d_old)/dt;

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

% disp('Simulation completed');
% figure(2)
% plot3(Q(1,:),Q(2,:),Q(3,:));
% hold on;
% plot3(xd,yd,zd);
% animateUAVwithManipulator(t, Q(1:6,:)', Q(7:8,:)',dt);

