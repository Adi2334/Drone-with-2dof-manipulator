syms t th1(t) th2(t) x(t) y(t) z(t) phi(t) theta(t) psi(t) Ix Iy Iz ...
I11 I12 I13 I21 I22 I23 phidot thetadot psidot ...
l1 l2 M1 m1 m2 g

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

Rt_T = simplify(R_phi * R_th * R_psi);
Rt = transpose(Rt_T); % B to I
Rt = simplify(RctoI * Rt);
% disp(Rt_T);


Q = simplify(transpose(Rt) * T);
RBtoL1 = [cos(th1) 0 -sin(th1); % BODY TO LINK1
    0 1 0;
    sin(th1) 0 cos(th1)];
RL1toL2 = [cos(th2) 0 -sin(th2); % LINK1 TO LINK2
    0 1 0;
    sin(th2) 0 cos(th2)];
RBtoL2 = simplify(RL1toL2 * RBtoL1); % BODY TO LINK2
% disp(RBtoL2);


P = transpose([x y z]); % position in inertial frame
% COM1 = [0; 0; l1/2];
P1B = simplify(transpose(RBtoL1) * transpose([0 0 l1/2])); % position of com of link 1 in B frame
P21 = simplify(transpose(RL1toL2) * transpose([0 0 l2/2])); % position of cqm of link 2 in link 1 frame wrt link 1 end pt
P2B = simplify(transpose(RBtoL1)*P21 + 2.*P1B); % position of com of link 2 in body frame wrt body frame
P1 = simplify(P + Rt * P1B);% pos of com of link 1 in inertial frame
P2 = simplify(P + Rt * P2B);% pos of com of l2 in inertial frame


Pdot = diff(P, t);% vel of drone wrt I in B
PHI = transpose([phi theta psi]);
PHI_dot = diff(PHI,t);
wB = simplify(Q * PHI_dot); % omega_body in body frame
wI = simplify(Rt * wB);  % omega_body in inertial frame

wB_1 = cos(theta(t))*diff(psi(t), t)*(cos(phi(t))*sin(theta(t)) - sin(psi(t))*sin(theta(t)) + cos(psi(t))*cos(theta(t))*sin(phi(t))) - (sin(phi(t))*sin(theta(t)) - cos(phi(t))*cos(psi(t))*cos(theta(t)))*diff(theta(t), t) + cos(theta(t))*sin(psi(t))*diff(phi(t), t);
wB_2 = (cos(phi(t))*cos(psi(t)) + sin(phi(t))*sin(psi(t))*sin(theta(t)))*diff(phi(t), t) - diff(psi(t), t)*(sin(theta(t))*(cos(phi(t))*cos(psi(t)) + sin(phi(t))*sin(psi(t))*sin(theta(t))) + cos(phi(t))*cos(theta(t))^2*sin(phi(t)) + cos(theta(t))*sin(phi(t))*(cos(phi(t))*sin(psi(t)) - cos(psi(t))*sin(phi(t))*sin(theta(t)))) - (cos(phi(t))*(cos(phi(t))*sin(psi(t)) - cos(psi(t))*sin(phi(t))*sin(theta(t))) - cos(theta(t))*sin(phi(t))^2)*diff(theta(t), t);
wB_3 = (cos(phi(t))*(sin(phi(t))*sin(psi(t)) + cos(phi(t))*cos(psi(t))*sin(theta(t))) + cos(phi(t))*cos(theta(t))*sin(phi(t)))*diff(theta(t), t) + diff(psi(t), t)*(sin(theta(t))*(cos(psi(t))*sin(phi(t)) - cos(phi(t))*sin(psi(t))*sin(theta(t))) - cos(phi(t))^2*cos(theta(t))^2 + cos(theta(t))*sin(phi(t))*(sin(phi(t))*sin(psi(t)) + cos(phi(t))*cos(psi(t))*sin(theta(t)))) - (cos(psi(t))*sin(phi(t)) - cos(phi(t))*sin(psi(t))*sin(theta(t)))*diff(phi(t), t); 
                                                                                                                                                                                              

 



th = transpose([th1 th2]);
th1dot = diff(th1, t);
th2dot = diff(th2,t);


w1 = wI + Rt * transpose([0 th1dot 0]);
w2 = wI + Rt * transpose([0 th1dot + th2dot 0]);

P1dot1 = transpose([th1dot * l1/2 0 0]);
P1dotB = simplify(transpose(RBtoL1) * P1dot1);
P2dotB = simplify(transpose(RBtoL1) * transpose([l1*th1dot+(l2/2)*th1dot 0 0]) + transpose(RBtoL2) * transpose([(l2/2)*(th2dot) 0 0]));

wB_skew = [0 -wB_3 wB_2;
    wB_3 0 -wB_1;
    -wB_2 wB_1 0];


P1dotI = simplify(Pdot + Rt * P1dotB + wB_skew * Rt * P1B);
P2dotI = simplify(Pdot + Rt * P2dotB + wB_skew * Rt * P2B);

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

Kb = simplify(0.5*(transpose(Pdot)*M1*Pdot) + 0.5*( transpose(PHI_dot)*transpose(T)*Rt*Id*transpose(Rt)*T*PHI_dot));
K1 = 0.5*( transpose(P1dotI)*m1*P1dotI ) + 0.5*( transpose(w1)*(Rt*transpose(RBtoL1))*I1*transpose((transpose(RBtoL1)*Rt))*w1);
K2 = 0.5*( transpose(P2dotI)*m2*P2dotI ) + 0.5*( transpose(w2)*(Rt*transpose(RBtoL2))*I2*transpose((transpose(RBtoL2)*Rt))*w2);

K = K1 + K2 + Kb;

Ud = simplify(M1*g*[0 0 1]*P) ;
U1 = simplify(m1*g*[0 0 1]*( P + Rt*P1B ));
U2 = simplify(m2*g*[0 0 1]*( P + Rt*P2B ));

U = Ud + U1 + U2;

U = simplify(U);

L = K - U; % Lagrangian
x_dot = diff(x,t);
exp_x = diff(diff(L,x_dot),t) - diff(L,x);
y_dot = diff(y,t);
exp_y = diff(diff(L,y_dot),t) - diff(L,y);
z_dot = diff(z,t);
exp_z = diff(diff(L,z_dot),t) - diff(L,z);
phi_dot = diff(phi,t);
exp_phi = diff(diff(L,phi_dot),t) - diff(L,phi);
theta_dot = diff(theta,t);
exp_theta = diff(diff(L,theta_dot),t) - diff(L,theta);
psi_dot = diff(psi,t);
exp_psi = diff(diff(L,psi_dot),t) - diff(L,psi);

exp_th1 = diff(diff(L,th1dot),t) - diff(L,th1);
exp_th2 = diff(diff(L,th2dot),t) - diff(L,th2);

%% CALCULATING M MATRIX

% M = zeros(8,8);
M = sym('M', [8, 8]);

% for x
M(1,1) = diff(exp_x,diff(x_dot,t));
M(1,2) = diff(exp_x,diff(y_dot,t));
M(1,3) = diff(exp_x,diff(z_dot,t));

M(1,4) = diff(exp_x,diff(phi_dot,t));
M(1,5) = diff(exp_x,diff(theta_dot,t));
M(1,6) = diff(exp_x,diff(psi_dot,t));

M(1,7) = diff(exp_x,diff(th1dot,t));
M(1,8) = diff(exp_x,diff(th2dot,t));


% for y
M(2,1) = diff(exp_y,diff(x_dot,t));
M(2,2) = diff(exp_y,diff(y_dot,t));
M(2,3) = diff(exp_y,diff(z_dot,t));

M(2,4) = diff(exp_y,diff(phi_dot,t));
M(2,5) = diff(exp_y,diff(theta_dot,t));
M(2,6) = diff(exp_y,diff(psi_dot,t));

M(2,7) = diff(exp_y,diff(th1dot,t));
M(2,8) = diff(exp_y,diff(th2dot,t));


% for z
M(3,1) = diff(exp_z,diff(x_dot,t));
M(3,2) = diff(exp_z,diff(y_dot,t));
M(3,3) = diff(exp_z,diff(z_dot,t));

M(3,4) = diff(exp_z,diff(phi_dot,t));
M(3,5) = diff(exp_z,diff(theta_dot,t));
M(3,6) = diff(exp_z,diff(psi_dot,t));

M(3,7) = diff(exp_z,diff(th1dot,t));
M(3,8) = diff(exp_z,diff(th2dot,t));

% for phi
M(4,1) = diff(exp_phi,diff(x_dot,t));
M(4,2) = diff(exp_phi,diff(y_dot,t));
M(4,3) = diff(exp_phi,diff(z_dot,t));

M(4,4) = diff(exp_phi,diff(phi_dot,t));
M(4,5) = diff(exp_phi,diff(theta_dot,t));
M(4,6) = diff(exp_phi,diff(psi_dot,t));

M(4,7) = diff(exp_phi,diff(th1dot,t));
M(4,8) = diff(exp_phi,diff(th2dot,t));

% for theta
M(5,1) = diff(exp_theta,diff(x_dot,t));
M(5,2) = diff(exp_theta,diff(y_dot,t));
M(5,3) = diff(exp_theta,diff(z_dot,t));

M(5,4) = diff(exp_theta,diff(phi_dot,t));
M(5,5) = diff(exp_theta,diff(theta_dot,t));
M(5,6) = diff(exp_theta,diff(psi_dot,t));

M(5,7) = diff(exp_theta,diff(th1dot,t));
M(5,8) = diff(exp_theta,diff(th2dot,t));


% for psi
M(6,1) = diff(exp_psi,diff(x_dot,t));
M(6,2) = diff(exp_psi,diff(y_dot,t));
M(6,3) = diff(exp_psi,diff(z_dot,t));

M(6,4) = diff(exp_psi,diff(phi_dot,t));
M(6,5) = diff(exp_psi,diff(theta_dot,t));
M(6,6) = diff(exp_psi,diff(psi_dot,t));

M(6,7) = diff(exp_psi,diff(th1dot,t));
M(6,8) = diff(exp_psi,diff(th2dot,t));

% for th1
M(7,1) = diff(exp_th1,diff(x_dot,t));
M(7,2) = diff(exp_th1,diff(y_dot,t));
M(7,3) = diff(exp_th1,diff(z_dot,t));

M(7,4) = diff(exp_th1,diff(phi_dot,t));
M(7,5) = diff(exp_th1,diff(theta_dot,t));
M(7,6) = diff(exp_th1,diff(psi_dot,t));

M(7,7) = diff(exp_th1,diff(th1dot,t));
M(7,8) = diff(exp_th1,diff(th2dot,t));


% for th2
M(8,1) = diff(exp_th2,diff(x_dot,t));
M(8,2) = diff(exp_th2,diff(y_dot,t));
M(8,3) = diff(exp_th2,diff(z_dot,t));

M(8,4) = diff(exp_th2,diff(phi_dot,t));
M(8,5) = diff(exp_th2,diff(theta_dot,t));
M(8,6) = diff(exp_th2,diff(psi_dot,t));

M(8,7) = diff(exp_th2,diff(th1dot,t));
M(8,8) = diff(exp_th2,diff(th2dot,t));


%% CALCULATING C MATRIX

% C = zeros(8,8);
C = sym('C', [8, 8]);

% for x
C(1,1) = diff(exp_x,x_dot);
C(1,2) = diff(exp_x,y_dot);
C(1,3) = diff(exp_x,z_dot);

C(1,4) = diff(exp_x,phi_dot);
C(1,5) = diff(exp_x,theta_dot);
C(1,6) = diff(exp_x,psi_dot);

C(1,7) = diff(exp_x,th1dot);
C(1,8) = diff(exp_x,th2dot);


% for y
C(2,1) = diff(exp_y,x_dot);
C(2,2) = diff(exp_y,y_dot);
C(2,3) = diff(exp_y,z_dot);

C(2,4) = diff(exp_y,phi_dot);
C(2,5) = diff(exp_y,theta_dot);
C(2,6) = diff(exp_y,psi_dot);

C(2,7) = diff(exp_y,th1dot);
C(2,8) = diff(exp_y,th2dot);


% for z
C(3,1) = diff(exp_z,x_dot);
C(3,2) = diff(exp_z,y_dot);
C(3,3) = diff(exp_z,z_dot);

C(3,4) = diff(exp_z,phi_dot);
C(3,5) = diff(exp_z,theta_dot);
C(3,6) = diff(exp_z,psi_dot);

C(3,7) = diff(exp_z,th1dot);
C(3,8) = diff(exp_z,th2dot);

% for phi
C(4,1) = diff(exp_phi,x_dot);
C(4,2) = diff(exp_phi,y_dot);
C(4,3) = diff(exp_phi,z_dot);

C(4,4) = diff(exp_phi,phi_dot);
C(4,5) = diff(exp_phi,theta_dot);
C(4,6) = diff(exp_phi,psi_dot);

C(4,7) = diff(exp_phi,th1dot);
C(4,8) = diff(exp_phi,th2dot);

% for theta
C(5,1) = diff(exp_theta,x_dot);
C(5,2) = diff(exp_theta,y_dot);
C(5,3) = diff(exp_theta,z_dot);

C(5,4) = diff(exp_theta,phi_dot);
C(5,5) = diff(exp_theta,theta_dot);
C(5,6) = diff(exp_theta,psi_dot);

C(5,7) = diff(exp_theta,th1dot);
C(5,8) = diff(exp_theta,th2dot);



% for psi
C(6,1) = diff(exp_psi,x_dot);
C(6,2) = diff(exp_psi,y_dot);
C(6,3) = diff(exp_psi,z_dot);

C(6,4) = diff(exp_psi,phi_dot);
C(6,5) = diff(exp_psi,theta_dot);
C(6,6) = diff(exp_psi,psi_dot);

C(6,7) = diff(exp_psi,th1dot);
C(6,8) = diff(exp_psi,th2dot);

% for th1
C(7,1) = diff(exp_th1,x_dot);
C(7,2) = diff(exp_th1,y_dot);
C(7,3) = diff(exp_th1,z_dot);

C(7,4) = diff(exp_th1,phi_dot);
C(7,5) = diff(exp_th1,theta_dot);
C(7,6) = diff(exp_th1,psi_dot);

C(7,7) = diff(exp_th1,th1dot);
C(7,8) = diff(exp_th1,th2dot);

% for th2
C(8,1) = diff(exp_th2,x_dot);
C(8,2) = diff(exp_th2,y_dot);
C(8,3) = diff(exp_th2,z_dot);

C(8,4) = diff(exp_th2,phi_dot);
C(8,5) = diff(exp_th2,theta_dot);
C(8,6) = diff(exp_th2,psi_dot);

C(8,7) = diff(exp_th2,th1dot);
C(8,8) = diff(exp_th2,th2dot);

%% CALCULATING THE G MATRIX

% C = zeros(8,1);
G = sym('G', [8, 1]);

G(1,1) = diff(exp_x,g);
G(2,1) = diff(exp_y,g);
G(3,1) = diff(exp_z,g);

G(4,1) = diff(exp_phi,g);
G(5,1) = diff(exp_theta,g);
G(6,1) = diff(exp_psi,g);

G(7,1) = diff(exp_th1,g);
G(8,1) = diff(exp_th2,g);

