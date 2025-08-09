% Function to compute q_dd dynamics
function T = asmc(q, q_d, qr_d, qr_dd, A, K, s, s_int)
    M = M_f(q(1), q(2), q(3), q(4), q(5), q(6), q(7), q(8));
    G = G_f(q(1), q(2), q(3), q(4), q(5), q(6), q(7), q(8));
    C = C_f(q(1), q(2), q(3), q(4), q(5), q(6), q(7), q(8), ...
            q_d(1), q_d(2), q_d(3), q_d(4), q_d(5), q_d(6), q_d(7), q_d(8));


    % T = M*qr_dd + C*qr_d + G - K.*sign(s) - A.*s;
    s_int = zeros(size(s_int));
    T = M*qr_dd + C*qr_d + G - A.*s - s_int;
    % T = - A.*s - s_int;
    if all(s < 0.05 & s > -0.05)
        T = T - K .* s;
    else
        T = T - K .* sign(s);
    end

end