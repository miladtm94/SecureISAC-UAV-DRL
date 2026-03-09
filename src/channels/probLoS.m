function P_LoS = probLoS(loc_i, loc_j)
%probLoS  ITU-R urban line-of-sight probability between two 3-D points.
%
%   P_LoS = probLoS(loc_i, loc_j)
%
%   Computes the probability that the link between location loc_i and
%   loc_j is line-of-sight (LoS) using the S-curve model commonly applied
%   in UAV channel research (ITU-R P.1410 / urban macro cell parameters).
%
%   Inputs:
%       loc_i  - 3-D Cartesian position [x; y; z] of point i  [m]
%       loc_j  - 3-D Cartesian position [x; y; z] of point j  [m]
%
%   Output:
%       P_LoS  - Probability of LoS link  (scalar, 0 ≤ P_LoS ≤ 1)
%
%   Model:
%       P_LoS(θ) = 1 / (1 + C · exp(−D · (θ_deg − C)))
%       where θ is the elevation angle in degrees.
%
%   Environment constants (urban):
%       C = 9.61,  D = 0.16
%
%   Reference: Eq. (2) of Mamaghani et al., IEEE JSAC 2026, DOI: 10.1109/JSAC.2025.3611404; Al-Hourani et al., 2014.

    %% Environment-specific S-curve constants (urban)
    C = 9.61;   % Horizontal offset / shape parameter
    D = 0.16;   % Steepness parameter

    %% Elevation angle computation
    delta_xyz  = loc_i - loc_j;
    h          = abs(delta_xyz(3));            % Vertical separation [m]
    d_horiz    = norm(delta_xyz(1:2));         % Horizontal distance [m]

    theta_deg  = atan2d(h, d_horiz);          % Elevation angle [degrees]

    %% LoS probability
    P_LoS = 1 / (1 + C * exp(-D * (theta_deg - C)));
end
