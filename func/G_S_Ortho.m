function Q = G_S_Ortho(M)
    [n, m] = size(M);  % n: rows, m: columns
    Q = zeros(n, m);   % Initialize orthonormal matrix Q

    for j = 1:m
        v = M(:, j);  % Take the j-th column of M

        % Subtract the projections of v onto the previous q vectors
        for i = 1:j-1
            v = v - (Q(:, i)' * M(:, j)) * Q(:, i);
        end

        % Normalize the vector
        Q(:, j) = v / norm(v);
    end
end

