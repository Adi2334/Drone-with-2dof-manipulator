% Function to compute q_dd dynamics
function q_dd = dynamics(q, q_d, T)
    M = M_f(q(1), q(2), q(3), q(4), q(5), q(6), q(7), q(8));
    G = G_f(q(1), q(2), q(3), q(4), q(5), q(6), q(7), q(8));
    C = C_f(q(1), q(2), q(3), q(4), q(5), q(6), q(7), q(8), ...
            q_d(1), q_d(2), q_d(3), q_d(4), q_d(5), q_d(6), q_d(7), q_d(8));
    if det(M) == 0
        error('Matrix M is singular!');
    end
    % cn = cond(M)
    % M = G_S_Ortho(M);
    K = (T-G-C*q_d);
    [M,K] = G_S_Ortho_mod2(M,K);
    % epsilon = 1e-6;
    % q_dd = (M + epsilon * eye(size(M))) \ (T - G - C * q_d);
    q_dd = (M)\  K;
end