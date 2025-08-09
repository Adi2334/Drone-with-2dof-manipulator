function [q, q_d] = PDFB_mod_RK4(q,q_d,qd,qd_d,Kp,Kd,dt)

% Calculate intermediate values using RK4 method
k1_qd = dt * (Kp.*(qd-q)+Kd.*(qd_d-q_d));
k1_q  = dt * (q_d);

% k2_qd = dt * dynamics(q + 0.5*k1_q, q_d + 0.5*k1_qd, T);
k2_qd = dt * (Kp.*(qd-q + 0.5*k1_q)+Kd.*(qd_d-q_d + 0.5*k1_qd));
k2_q  = dt * (q_d + 0.5*k1_qd);

% k3_qd = dt * dynamics(q + 0.5*k2_q, q_d + 0.5*k2_qd, T);
k3_qd = dt * (Kp.*(qd-q + 0.5*k2_q)+Kd.*(qd_d-q_d + 0.5*k2_qd));
k3_q  = dt * (q_d + 0.5*k2_qd);

% k4_qd = dt * dynamics(q + k3_q, q_d + k3_qd, T);
k4_qd = dt * (Kp.*(qd-q + k3_q)+Kd.*(qd_d-q_d + k3_qd));
k4_q  = dt * (q_d + k3_qd);

% Update q and q_d using RK4 weighted sum
q_d = q_d + (1/6) * (k1_qd + 2*k2_qd + 2*k3_qd + k4_qd);
q   = q   + (1/6) * (k1_q  + 2*k2_q  + 2*k3_q  + k4_q);

end

