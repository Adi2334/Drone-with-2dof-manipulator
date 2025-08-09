dt = 0.01;
T = 20;
t = 0:dt:T;
g = 9.81;
t = 0.2*t;
O = zeros(8,8);
I = eye(8,8);
% xd means x desired
% x_d means x_dot

xd = 0*t;
yd = 0*t;
zd = 0*(t);
phid = 0.*t;
thetad = 0.*t;
psid = 0.*t;
th1d = 0.*t;
th2d = 0.*t;
% plot3(xd,yd,zd);
% grid on;
qd = [xd;yd;zd;phid;thetad;psid;th1d;th2d];
%% Controller

% Initial sates
q_d = [0;0;0;0;0;0;0;0];  % initial
q = [0;0;0;0;0;0;0;0];   % initial
[M,C,G] = Matrices(q,q_d);
K = LQR_K(M,C,dt);
disp(size(K));

qd_d = (qd(:,1) - qd(:,2))/dt;
X = [q; q_d];
Xd = [qd(:,1); qd_d];

Q = [q];

T_c = zeros(8,1);
threshold = nan;
for i = 2:size(t,2)-1

    T_c = G - K*(Xd-X)
    % T_c(T_c<-threshold) = -threshold;
    % T_c(T_c>threshold) = threshold;

    w = Roll_pitch(q,T_c);
    qd(5,i) = w(1); qd(4,i) = w(2);

    [q, q_d] = f_inverse_dyna_RK4(T_c,q,q_d,dt);
    Q = [Q,q];
    qd_d = (qd(:,i+1) - qd(:,i-1))/(2*dt);

    [M,C,G] = Matrices(q,q_d);
    K = LQR_K(M,C,dt);
    
    X = [q; q_d];
    Xd = [qd(:,i); qd_d];

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

