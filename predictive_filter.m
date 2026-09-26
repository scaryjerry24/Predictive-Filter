function [p_hat_hist, q_hat_hist] = predictive_filter(beacons, b_tilde_hist, sigma, dt, p0, q0)
% predictive_filter - runs the noniterative predictive filter (Eqs. 46-51)
% Inputs:
%   p_truth, q_truth - truth history, used only for K (length), NOT fed to filter
%   beacons          - 3xN known beacon positions
%   b_tilde_hist     - 3xNxK noisy measurements (from gen_measurements)
%   sigma            - measurement noise std dev (rad)
%   dt               - time step (s)
%   p0, q0           - initial position/attitude estimate
% Outputs:
%   p_hat_hist, q_hat_hist - estimated position/attitude history

    K = size(b_tilde_hist, 3);
    N = size(beacons, 2);

    p_hat_hist = zeros(3, K);
    q_hat_hist = zeros(4, K);

    p_hat = p0;
    q_hat = q0;
    p_hat_hist(:,1) = p_hat;
    q_hat_hist(:,1) = q_hat;

    R = sigma^2 * eye(3*N);   % Eq. 47d, since all beacons share the same sigma here

    for k = 1:K-1

    R_hat = zeros(3, N);
    zeta_hat_all = zeros(N, 1);

    for i = 1:N
        dist_i = beacons(:,i) - p_hat;
        R_hat(:,i) = dist_i / norm(dist_i);
        zeta_hat_all(i) = norm(dist_i)^2;
    end


    A_hat = q2dcm(q_hat);
    S = build_S(A_hat, R_hat, zeta_hat_all);

    V = A_hat * R_hat;   % say V = [1 4; 2 5; 3 6]  (beacon 1 = [1;2;3], beacon 2 = [4;5;6])
    y_hat = reshape(V, [], 1); 
    % result: [1; 2; 3; 4; 5; 6] 18 x 1
    
    y_tilde = reshape(b_tilde_hist(:,:,k+1), [], 1);
    d = -(1/dt) * ((S' / R * S) \ (S' / R * (y_hat - y_tilde))); % correcction vector

    d_q = d(1:3);
    d_p = d(4:6);

    q_hat = propagate_quat(q_hat, d_q, dt);
    p_hat = p_hat + d_p * dt;

    p_hat_hist(:,k+1) = p_hat;
    q_hat_hist(:,k+1) = q_hat;

    end

end