function [t, p_truth, q_truth] = traj_truth()
% traj_truth - generates the true trajectory: simple linear approach
% 45 m -> 0 m along Z over 30 minutes, 100 Hz sampling.
% Output:
%   t       - 1xK time vector (s)
%   p_truth - 3xK matrix, true position [X;Y;Z] at each time step (m)

    dt = 0.01;          % 100 Hz
    T = 30*60;          % 30 minutes in seconds

    t = 0:dt:T; % Time vector (starting : step : final)
    K = length(t); % Consistent sizing measures
    p_truth = zeros(3, K);
    p_truth(3, :) = 45 * (1 - t / T); % Trends for the most simple case

     phi = deg2rad(10) * (t/T);     % Roll change over same time period
    theta = deg2rad(10) * (t/T);   % Pitch change over same time period
    psi = zeros(1,K);              % No yaw (rotation about vertical axis)

    q_truth = zeros(4, K); % Reserving space via zeros

    for k = 1:K
     q_truth(1,k) = sin(phi(k)/2)*cos(theta(k)/2)*cos(psi(k)/2) - cos(phi(k)/2)*sin(theta(k)/2)*sin(psi(k)/2);
     q_truth(2,k) = cos(phi(k)/2)*sin(theta(k)/2)*cos(psi(k)/2) + sin(phi(k)/2)*cos(theta(k)/2)*sin(psi(k)/2);
     q_truth(3,k) = cos(phi(k)/2)*cos(theta(k)/2)*sin(psi(k)/2) - sin(phi(k)/2)*sin(theta(k)/2)*cos(psi(k)/2);
     q_truth(4,k) = cos(phi(k)/2)*cos(theta(k)/2)*cos(psi(k)/2) + sin(phi(k)/2)*sin(theta(k)/2)*sin(psi(k)/2);
    end

end