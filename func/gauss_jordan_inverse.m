function A_inv = gauss_jordan_inverse(A)
    % Check if A is a square matrix
    [n, m] = size(A);
    if n ~= m
        error('Matrix must be square');
    end
    A_aug = [A eye(n)];
    for i = 1:n
        if A_aug(i, i) == 0
            error('Matrix is singular and cannot be inverted');
        end
        A_aug(i, :) = A_aug(i, :) / A_aug(i, i);
        for j = 1:n
            if j ~= i
                A_aug(j, :) = A_aug(j, :) - A_aug(j, i) * A_aug(i, :);
            end
        end
    end
    A_inv = A_aug(:, n+1:end);
end
