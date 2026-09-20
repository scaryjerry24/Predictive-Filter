function b_tilde = add_los_noise(b_true, sigma)
% add_los_noise - adds Gaussian noise to a true LOS unit vector, constrained
% to the tangent plane (Eqs. 4-5, Crassidis et al.)
% Inputs:
%   b_true - true unit vector (3x1)
%   sigma  - noise standard deviation (rad)
% Output:
%   b_tilde - noisy unit vector (3x1)

    % Step 1: pick a helper vector not parallel to b_true
    if abs(b_true(1)) < 0.9
        ref = [1;0;0];
    else
        ref = [0;1;0];
    end
    
    t1 = cross(b_true, ref);
    t1 = t1 / norm(t1);
    t2 = cross(b_true, t1);
    % Generate tangent-plane Gaussian perturbations and renormalize.
    noise = sigma * randn(2, 1);
    bNoisy = b_true + noise(1) * t1 + noise(2) * t2;
    b_tilde = bNoisy / norm(bNoisy);
   
end