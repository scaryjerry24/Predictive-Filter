function S = build_S(A_hat, R_hat, zeta_hat_all)
% build_S - stacks all N beacons' 3x6 blocks into the full sensitivity matrix (Eq. 46)
% Inputs:
%   A_hat        - estimated attitude matrix (3x3)
%   R_hat        - 3xN matrix, each COLUMN is one beacon's unit vector r_hat_i
%   zeta_hat_all - Nx1 vector, each entry is beacon i's squared range
% Output:
%   S - full 3N x 6 sensitivity matrix

    N = size(R_hat, 2);      % number of beacons = number of columns now
    V = A_hat * R_hat;       % rotate every beacon's r_hat at once (3xN)
    S = [];

        for i = 1:N
        v_i = V(:, i);              % this beacon's pre-rotated vector
        r_hat_i = R_hat(:, i);      % this beacon's unrotated reference vector
        zeta_hat_i = zeta_hat_all(i);

        S_i = build_S_single(A_hat, v_i, r_hat_i, zeta_hat_i);

        S = [S; S_i];
        
        end
end