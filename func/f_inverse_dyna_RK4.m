function [q, q_d] = f_inverse_dyna_RK4(T,q,q_d,dt)

% Calculate intermediate values using RK4 method
k1_qd = dt * dynamics(q, q_d, T);
k1_q  = dt * (q_d);

k2_qd = dt * dynamics(q + 0.5*k1_q, q_d + 0.5*k1_qd, T);
k2_q  = dt * (q_d + 0.5*k1_qd);

k3_qd = dt * dynamics(q + 0.5*k2_q, q_d + 0.5*k2_qd, T);
k3_q  = dt * (q_d + 0.5*k2_qd);

k4_qd = dt * dynamics(q + k3_q, q_d + k3_qd, T);
k4_q  = dt * (q_d + k3_qd);

% Update q and q_d using RK4 weighted sum
q_d = q_d + (1/6) * (k1_qd + 2*k2_qd + 2*k3_qd + k4_qd);
q   = q   + (1/6) * (k1_q  + 2*k2_q  + 2*k3_q  + k4_q);


end

