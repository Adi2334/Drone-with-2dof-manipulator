function [M,K] = G_S_Ortho_mod2(M,K)
    [n, m] = size(M);  % n: rows, m: columns
    % Q = zeros(n, m);   % Initialize orthonormal matrix Q
    % P = zeros(n, 1);
    for j = 1:n
        v = M(j, :);  % Take the j-th row of M
        % w = K(j);
        % Subtract the projections of v onto the previous q vectors
        for i = 1:j-1
            M(j,:) = M(j,:) - (M(i, :) * v') * M(i,:)/norm(M(i,:));   % LHS
            K(j) = K(j) - (M(i, :) * v') * K(i)/norm(M(i, :));    % RHS
        end

        % Normalize the vector
        d = norm(M(j,:));
        M(j,:) = M(j,:)/d;
        % M(j,:) = v/norm(v);
        K(j) = K(j)/d;
    end
end
