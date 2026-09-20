function beacons = beacon_setup()
% beacon_setup - beacon positions from Fig. 4, Crassidis, Alonso & Junkins
% Coordinates estimated from the published figure (paper gives no table).
% Output:
%   beacons - 3x6 matrix, each column is one beacon's [X;Y;Z] position (m)

beacons =     [-0.15   0.45   0.15   0.13   0.15   0.15;   % X
                0.30  -0.10  -0.15  -0.17  -0.25  -0.30;   % Y
                0.00  -0.10   0.15   0.18  -0.10  -0.28];  % Z

end


