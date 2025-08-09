function K = LQR_K(M,C,dt)
 
Q = eye(16);
R = eye(8);
O = zeros(8,8);
I = eye(8,8);

% M_inv = gauss_jordan_inverse(M);
A = [O I; O -M\C];
B = [O; M\I];

A_ = A.*dt + eye(16);
B_ = B.*dt;

[K,~,~] = dlqr(A_,B_,Q,R);


