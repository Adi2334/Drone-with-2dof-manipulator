load('G_val.mat');
G_mod2 = sym('G_mod2', [8, 1]);

% Define variables and derivatives symbolically
syms x(t) y(t) z(t) phi(t) theta(t) psi(t) th1(t) th2(t) x_dot y_dot ...
    z_dot phi_dot theta_dot psi_dot th1_dot th2_dot X Y Z PHI PSI THETA TH1 TH2
% x_dot = diff(x, t); y_dot = diff(y, t); z_dot = diff(z, t);
% phi_dot = diff(phi, t); theta_dot = diff(theta, t); psi_dot = diff(psi, t);
% th1_dot = diff(th1, t); th2_dot = diff(th2, t);


%% For changing x(t) to X
for i = 1:8
    for j = 1:1
        tic
        symbolic_expr = G(i,j);
        if symbolic_expr == 0
            G_mod2(i,j) = symbolic_expr;
            disp(complex(i, -j));
            continue;
        else
            % Substitute derivatives with new symbolic variables
            modified_expr = subs(symbolic_expr, ...
                {x(t), y(t), z(t), phi(t), theta(t), psi(t), th1(t), th2(t)}, ...
                {X, Y, Z, PHI, THETA, PSI, TH1, TH2});
            
            G_mod2(i,j) = modified_expr;
        end
        disp(complex(i, j));
        toc
    end
end

% exp = diff(modified_expr,theta_dot);
% exp2 = diff(symbolic_expr, diff(theta(t), t));
% a = exp - exp2
% toc

% %% For changing diff to dot
% for i = 1:8
%     for j = 1:8
%         tic
%         symbolic_expr = C(i,j);
%         if symbolic_expr == 0
%             C_mod(i,j) = symbolic_expr;
%             disp(complex(i, -j));
%             continue;
%         else
%             % Substitute derivatives with new symbolic variables
%             modified_expr = subs(symbolic_expr, ...
%                 {diff(x(t), t), diff(y(t), t), diff(z(t), t), ...
%                  diff(phi(t), t), diff(theta(t), t), diff(psi(t), t), ...
%                  diff(th1(t), t), diff(th2(t), t)}, ...
%                 {x_dot, y_dot, z_dot, phi_dot, theta_dot, psi_dot, th1_dot, th2_dot});
% 
%             C_mod(i,j) = modified_expr;
%         end
%         disp(complex(i, j));
%         toc
%     end
% end
