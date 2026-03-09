function [A_0, zeta_0] = sensingChannel(RBS_loc, target_loc)
%sensingChannel  Compute the ISAC radar sensing channel matrix and path loss.
%
%   [A_0, zeta_0] = sensingChannel(RBS_loc, target_loc)
%
%   Computes:
%       A_0    — Sensing channel matrix (outer product of Tx/Rx steering
%                vectors for a UPA at the R-BS), size [Mr × Mt].
%       zeta_0 — One-way complex path-loss amplitude (including RCS),
%                |zeta_0|² = beta_0 · d^{-alpha} · |RCS|.
%
%   Inputs:
%       RBS_loc    - 3-D position of the Radar Base Station [x; y; z] [m]
%       target_loc - 3-D position of the UAV target          [x; y; z] [m]
%
%   Outputs:
%       A_0    - Sensing channel matrix  [Mr × Mt]  (complex)
%       zeta_0 - Complex one-way path-loss amplitude  (scalar)
%
%   Parameters loaded from sysParams:
%       M_tx_h, M_tx_v, M_rx_h, M_rx_v, wavelength, fc,
%       beta_0, alpha_pathloss, RCS
%
%   Reference: Eq. (6)–(9) of Mamaghani et al., IEEE JSAC 2026, DOI: 10.1109/JSAC.2025.3611404.

    sysParams   % Load shared system parameters

    %% Geometry: distance and angles from R-BS to target
    delta_xyz    = target_loc - RBS_loc;
    distance     = norm(delta_xyz);
    distance_xy  = norm(delta_xyz(1:2));

    theta = atan2(delta_xyz(3), distance_xy);   % Elevation angle [rad]
    phi   = atan2(delta_xyz(2), delta_xyz(1));  % Azimuth angle   [rad]
    ang   = rad2deg([phi; theta]);              % [azimuth; elevation] [deg]

    %% Steering vectors via Phased Array System Toolbox
    a_t = steeringVector(M_tx_h, M_tx_v, ang, wavelength, fc);   % Tx (AoD)
    a_r = steeringVector(M_rx_h, M_rx_v, ang, wavelength, fc);   % Rx (AoA)

    %% Sensing channel matrix  A_0 = a_r * a_t'   [Mr × Mt]
    A_0 = a_r * a_t';

    %% One-way complex path-loss amplitude  zeta_0
    %   |zeta_0|² encodes free-space path loss and radar cross-section (RCS).
    zeta_0 = sqrt(beta_0 * distance^(-alpha_pathloss) * RCS);
end


%% -------------------------------------------------------------------------
function a = steeringVector(M_h, M_v, ang_deg, wavelength, fc)
%steeringVector  Compute the UPA steering vector using MATLAB's phased toolbox.
%
%   a = steeringVector(M_h, M_v, ang_deg, wavelength, fc)
%
%   Inputs:
%       M_h       - Number of horizontal antenna elements
%       M_v       - Number of vertical   antenna elements
%       ang_deg   - [azimuth; elevation] angle pair [degrees]
%       wavelength - Carrier wavelength [m]
%       fc         - Carrier frequency  [Hz]
%
%   Output:
%       a  - Steering vector  [(M_h * M_v) × 1]  (complex)

    d_spacing = wavelength / 2;   % Half-wavelength inter-element spacing

    array    = phased.URA('Size', [M_h, M_v], ...
                          'ElementSpacing', [d_spacing, d_spacing], ...
                          'ArrayNormal', 'y');
    steervec = phased.SteeringVector('SensorArray', array);
    a        = steervec(fc, ang_deg);
end
