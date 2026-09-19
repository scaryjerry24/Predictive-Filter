function S_i = build_S_single(A_hat, v_i, r_hat_i, zeta_hat_i)
% build_S_single - one beacon's 3x6 block of the sensitivity matrix S (Eq. 46)
% Inputs:
%   A_hat      - estimated attitude matrix (3x3)
%   v_i        - PRE-ROTATED reference vector, A_hat*r_hat_i (3x1)
%   r_hat_i    - beacon i's unit vector, object frame, unrotated (3x1)
%   zeta_hat_i - beacon i's squared range (scalar)
% Output:
%   S_i - 3x6 block for this beacon

    v_cross = [0 -v_i(3) v_i(2); v_i(3) 0 -v_i(1); -v_i(2) v_i(1) 0];
    product_outer = r_hat_i * r_hat_i';
    right_block = -A_hat * zeta_hat_i^(-0.5) * (eye(3) - product_outer);

    S_i = [v_cross, right_block];
end