function q_next = propagate_quat(q, dq, dt)
    % propagate_quat - discrete quaternion propagation (Eqs. 49-50, Crassidis et al.)
    % Inputs:
    %   q  - current quaternion estimate (4x1)
    %   dq - model error correction for attitude, d_q (3x1)
    %   dt - time step (s)
    % Output:
    %   q_next or q_(k+1) - propagated quaternion (4x1)

    % Compute magnitude of dq
    magnitude_dq = norm(dq); 

    % Compute chi_k and mu_k
    chi_k = cos(magnitude_dq * dt / 2); % EQN 50a
    mu_k = sin(magnitude_dq * dt / 2); % EQN 50b

    % Build omega_k and omega_cross
    omega_k = dq / magnitude_dq; % EQN 50c
    omega_cross = [0 -omega_k(3) omega_k(2); 
    omega_k(3) 0 -omega_k(1);
    -omega_k(2) omega_k(1) 0];

    % Create 4x4 Omega(omega_k)
    Omega = [-omega_cross, omega_k; % EQN 50d
        -omega_k', 0];

    q_next = (chi_k*eye(4) + mu_k*Omega)*q ; % EQN 49

end