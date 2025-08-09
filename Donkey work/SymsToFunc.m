syms X Y Z PHI THETA PSI TH1 TH2 x_dot y_dot z_dot phi_dot theta_dot psi_dot th1_dot th2_dot   % Define x and y as symbolic variables (not x(t))
% C = x + sin(y);  % Symbolic expression

load('M_mod2.mat');
load('C_mod2.mat');
load('G_mod2.mat');

% Convert the symbolic expression to a function handle
M_func = matlabFunction(M_mod2, 'Vars', [X, Y, Z, PHI, THETA, PSI, TH1, TH2]);
C_func = matlabFunction(C_mod2, 'Vars', [X, Y, Z, PHI, THETA, PSI, TH1, TH2, x_dot, y_dot, z_dot, phi_dot, theta_dot, psi_dot, th1_dot, th2_dot]);
G_func = matlabFunction(G_mod2, 'Vars', [X, Y, Z, PHI, THETA, PSI, TH1, TH2]);



