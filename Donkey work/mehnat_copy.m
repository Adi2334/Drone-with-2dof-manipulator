syms t th1(t) th2(t) x(t) y(t) z(t) phi(t) theta(t) psi(t) Ix Iy Iz ...
I11 I12 I13 I21 I22 I23 phidot thetadot psidot ...
l1 l2 M m1 m2 g

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

Kb = simplify(0.5*(transpose(Pdot)*M*Pdot) + 0.5*( transpose(PHI_dot)*transpose(T)*Rt*Id*transpose(Rt)*T*PHI_dot));
K1 = 0.5*( transpose(P1dotI)*m1*P1dotI ) + 0.5*( transpose(w1)*(Rt*transpose(RBtoL1))*I1*transpose((transpose(RBtoL1)*Rt))*w1);
K2 = 0.5*( transpose(P2dotI)*m2*P2dotI ) + 0.5*( transpose(w2)*(Rt*transpose(RBtoL2))*I2*transpose((transpose(RBtoL2)*Rt))*w2);

K = K1 + K2 + Kb;

Ud = simplify(M*g*[0 0 1]*P) ;
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





% % 
% % expr = K;
% % 
% % % Convert the symbolic expression to a string
% % expr_str = char(expr);
% % 
% % % Open a file to write the symbolic expression
% % fid = fopen('symbolic_expression.txt', 'w');  % 'w' for writing
% % 
% % % Write the string version of the symbolic expression to the file
% % fprintf(fid, '%s\n', expr_str);
% % 
% % % Close the file
% % fclose(fid);
% % 
% % % Confirm that the expression is written to the file
% % disp('The symbolic expression has been written to symbolic_expression.txt');
% % 
% % 

% % disp(K1);
% % disp(U);