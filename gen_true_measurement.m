function b_true = gen_true_measurement(p_truth_k, q_truth_k, beacon_i)
% gen_true_measurement - noiseless LOS unit vector for one beacon, one time step
% Inputs:
%   p_truth_k - true position at this time step (3x1)
%   q_truth_k - true quaternion at this time step (4x1)
%   beacon_i  - this beacon's known position (3x1)
% Output:
%   b_true - true (noiseless) unit vector, sensor frame (3x1)

    r_i = (beacon_i - p_truth_k) / norm(beacon_i - p_truth_k);
    A = q2dcm(q_truth_k); % Recall previous quaternion to directional cosine matrix
    b_true = A * r_i; % Rotated to sensor frame true unit vector (noiseless)
  
end