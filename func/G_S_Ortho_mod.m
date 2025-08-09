function [Q,P] = G_S_Ortho_mod(M,K)
    [n, m] = size(M);  % n: rows, m: columns
    Q = zeros(n, m);   % Initialize orthonormal matrix Q
    P = zeros(n, 1);
    for j = 1:n
        v = M(j, :);  % Take the j-th row of M
        w = K(j);
        % Subtract the projections of v onto the previous q vectors
        for i = 1:j-1
            v = v - (Q(i, :) * M(j, :)') * Q(i, :);   % LHS
            w = w - (Q(i, :) * M(j, :)') * P(i)/norm(M(i, :));    % RHS
        end

        % Normalize the vector
        M(j,:) = v;
        Q(j,:) = v/norm(v);
        P(j) = w;
    end
end
