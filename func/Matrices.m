% Function to compute q_dd dynamics
function [M,C,G] = Matrices(q,q_d)
    M = M_f(q(1), q(2), q(3), q(4), q(5), q(6), q(7), q(8));
    G = G_f(q(1), q(2), q(3), q(4), q(5), q(6), q(7), q(8));
    C = C_f(q(1), q(2), q(3), q(4), q(5), q(6), q(7), q(8), ...
            q_d(1), q_d(2), q_d(3), q_d(4), q_d(5), q_d(6), q_d(7), q_d(8));

end