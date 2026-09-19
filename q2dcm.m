function A = q2dcm(q)
% q2dcm - convert quaternion to direction cosine matrix
% q = [q1 q2 q3 q4]' where [q1 q2 q3] = rho (vector part), q4 = scalar part
% Uses A(q) = Xi'(q) * Psi(q),  Eqs. 43-44 in Crassidis et al.

    rho = q(1:3); % EQN 41
    q4  = q(4); % EQN 41

    rho_cross = [0 -rho(3) rho(2); rho(3) 0 -rho(1); -rho(2) rho(1) 0];

    Xi = [ (q4*eye(3)) + rho_cross ; -transpose(rho)];   % EQN 44a
 
    Psi = [q4*eye(3) - rho_cross; -transpose(rho)];  % EQN 44b

    A = Xi'*Psi; % EQN 43

end