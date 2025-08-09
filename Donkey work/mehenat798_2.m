syms th1 th2 x y z phi theta psi p q r u v w Ix Iy Iz ...
I11 I12 I13 I21 I22 I23 phidot thetadot psidot ...
l1 l2 th1dot th2dot M m1 m2 g xdot ydot zdot

T = [1 0 -sin(theta);
    0 cos(phi) sin(phi)*cos(theta);
    0 -sin(phi) cos(theta)*cos(phi)];

R_phi = [1 0 0;
    0 cos(phi) sin(phi);
    0 -sin(phi) cos(phi)];
R_th = [cos(theta) 0 -sin(theta)
    0 1 0;
    sin(theta) 0 cos(theta)];
R_psi = [cos(psi) sin(psi) 0;
    -sin(psi) cos(psi) 0;
    0 0 1];
RctoI = [0 1 0;
    1 0 0;
    0 0 -1];

Rt_T = (R_phi * R_th * R_psi);
Rt = transpose(Rt_T); % B to I
Rt = RctoI * Rt;
% disp(Rt_T);


Q = transpose(Rt) * T;
RBtoL1 = [cos(th1) 0 -sin(th1); % BODY TO LINK1
    0 1 0;
    sin(th1) 0 cos(th1)];
RL1toL2 = [cos(th2) 0 -sin(th2); % LINK1 TO LINK2
    0 1 0;
    sin(th2) 0 cos(th2)];
RBtoL2 = simplify(RL1toL2 * RBtoL1); % BODY TO LINK2

% Inertia tensor
Id = [Ix 0 0;
    0 Iy 0;
    0 0 Iz];
I1 = [I11 0 0;
    0 I12 0;
    0 0 I13];

I2 = [I21 0 0;
    0 I22 0;
    0 0 I23];
P = transpose([x y z]); % position in inertial frame

P1B = simplify(transpose(RBtoL1) * transpose([0 0 l1/2])); % position of com of link 1 in B frame
P21 = simplify(transpose(RL1toL2) * transpose([0 0 l2/2])); % position of cqm of link 2 in link 1 frame wrt link 1 end pt
P2B = simplify(transpose(RBtoL1)*P21 + 2.*P1B); % position of com of link 2 in body frame wrt body frame
P1 = simplify(P + Rt * P1B);% pos of com of link 1 in inertial frame
P2 = simplify(P + Rt * P2B);% pos of com of l2 in inertial frame

PP1B = Rt * P1B;
PP2B = Rt * P2B;

PP1B_sk = [0 -PP1B(3) PP1B(2);
           PP1B(3) 0 -PP1B(1);
           -PP1B(2) PP1B(1) 0];

PP2B_sk = [0 -PP2B(3) PP2B(2);
           PP2B(3) 0 -PP2B(1);
           -PP2B(2) PP2B(1) 0];

Jt1 = [0.5*l1*cos(th1) 0;
        0 0;
        -0.5*sin(th1) 0];

Jt2 = [cos(th1)*(l1 + 0.5*l2) cos(th1 + th2)*0.5*l2;
        0 0;
        -sin(th1)*(l1 + 0.5*l2) -sin(th1 + th2)*0.5*l2];

Jr1  = [0 0;
        1 0;
        0 0];
Jr2 = [0 0;
        1 1;
        0 0];

Mtb = ([eye(3) zeros(3,3) zeros(3,2)]);
Mrb = ([zeros(3,3) T zeros(3,2)]);
Mt1 = ([eye(3) -PP1B_sk*T Rt*Jt1]);
Mt2 = ([eye(3) -PP2B_sk*T Rt*Jt2]);
Mr1 = ([zeros(3,3) T Rt*Jr1]);
Mr2 = ([zeros(3,3) T Rt*Jr2]);


Mqd = simplify(transpose(Mtb)*M*Mtb + transpose(Mrb)*Rt*Id*transpose(Rt)*Mrb);
Mq1 = simplify(transpose(Mt1)*m1*Mt1 + transpose(Mr1)*(Rt*transpose(RBtoL1))*I1*transpose(Rt*transpose(RBtoL1))*Mr1);
Mq2 = simplify(transpose(Mt2)*m2*Mt2 + transpose(Mr2)*(Rt*transpose(RBtoL2))*I2*transpose(Rt*transpose(RBtoL2))*Mr2);

% Mq = simplify(Mqd + Mq1 + Mq2);


