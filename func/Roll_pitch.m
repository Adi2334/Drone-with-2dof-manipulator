function w = Roll_pitch(q,T_c)

    R_phi = [1 0 0;
            0 cos(q(4)) sin(q(4));
            0 -sin(q(4)) cos(q(4))];
    R_th = [cos(q(5)) 0 -sin(q(5))
        0 1 0;
        sin(q(5)) 0 cos(q(5))];
    R_psi = [cos(q(6)) sin(q(6)) 0;
        -sin(q(6)) cos(q(6)) 0;
        0 0 1];
    RctoI = [0 1 0;
        1 0 0;
        0 0 -1];

    Rt_T = (R_phi * R_th * R_psi);
    Rt = transpose(Rt_T); % B to I
    Rt = RctoI * Rt;


    w = (Rt(3,3)/T_c(3))*[cos(q(6)), sin(q(6));
                             sin(q(6)), -cos(q(6))] * T_c(1:2);
    