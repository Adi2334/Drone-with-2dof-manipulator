dt = 0.1;
T = 50;
t = 0:dt:T;
g = 9.81;
t = 0.2*t;
% xd means x desired
% x_d means x_dot

xd = cos(t);
yd = 0.5*(t);
zd = 0*t;
phid = 0.*t;
thetad = 0.*t;
psid = 0.*t;
th1d = -(0)*ones(size(t));
th2d = 0*t;
qd = [xd;yd;zd;phid;thetad;psid;th1d;th2d];

%% Controller

% Initial sates
q_d = [0;0;0;0;0;0;0;0];  % initial
q = [0;0;0;0;0;0;-0/2;0];   % initial

e = qd(:,1)-q;
qd_d_old = zeros(8,1);
qd_d = (qd(:,1) - qd(:,2))/dt;
qd_dd = (qd(:,1)-2*qd(:,2)+qd(:,3))./(2*dt);
Qd_dd = [qd_dd];
e_d = qd_d-q_d;

% K1 = 0.1; K2 = 0; % sum
K1 = [0.1,0.1,2,0,0,0.2,0.5,0.5]';
% K2 = [2,2,2,0,0,2,2,2]';
K2 = 2.*sqrt(K1);
Q = [q];
% V = [0,0,0,0,0,0,0,0]';
V = ones(1,8)';

Tp = [0;0];
for i = 2:size(t,2)-1
    e_dd = K1.*e + K2.*e_d + V.*qd_dd;
    
    T_c = f_dynamics(q,q_d,e_dd);
    if(isnan(T_c))
        disp('nan');
        return;
    end
    if(~isreal(T_c))
        return;
    end
    R_phi = [1 0 0;
        0 cos(q(4)) sin(q(4));
        0 -sin(q(4)) cos(q(4))];
    R_th = [cos(q(5)) 0 -sin(q(5))
        0 1 0;
        sin(q(5)) 0 cos(q(5))];
    R_psi = [cos(q(6)) sin(q(6)) 0;
        -sin(q(6)) cos(q(6)) 0;
        0 0 1];
    RctoI = [0 1 0;
        1 0 0;
        0 0 -1];

    Rt_T = (R_phi * R_th * R_psi);
    Rt = transpose(Rt_T); % B to I
    Rt = RctoI * Rt;

    w = (Rt(3,3)/T_c(3))*[cos(q(6)), sin(q(6));
                             sin(q(6)), -cos(q(6))] * T_c(1:2);
    qd(5,i) = w(1); qd(4,i) = w(2);
    % qd_d(5,i) = (qd(5,i) - qd5old)/dt;
    % qd_d(4,i) = (qd(4,i) - qd4old)/dt;
    % qd5old = qd(5,i);
    % qd4old = qd(4,i);

    [q, q_d] = f_inverse_dyna_RK4(T_c,q,q_d,dt);
    
    Q = [Q,q];
    
    qd_d_old = qd_d;
    qd_d = (qd(:,i+1) - qd(:,i-1))/(2*dt);
    qd_dd = (qd(:,i+1) - 2*qd(:,i) + qd(:,i-1))./(2*dt);
    Qd_dd = [Qd_dd,qd_dd];
    e = qd(:,i)-q;
    e_d = qd_d-q_d;

end

figure(1)
for i = 1:8
    subplot(4, 2, i);
    plot(Q(i, :));
    hold on;
    grid on;
    plot(qd(i,:));
    title(['q' num2str(i)]);
end
disp('Simulation completed');
figure(2)
plot3(Q(1,:),Q(2,:),Q(3,:));
hold on;
plot3(xd,yd,zd);
animateUAVwithManipulator(t, Q(1:6,:)', Q(7:8,:)');

