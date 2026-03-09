%% sysParams.m — Global System Parameters for SecISAC Simulation
%
%   This script defines all physical and simulation parameters used
%   across the SecISAC codebase.  It is intended to be called via
%       sysParams
%   at the top of any function or script that needs these constants.
%
%   Sections:
%       1. Antenna array configuration
%       2. Channel and propagation parameters
%       3. Signal and bandwidth parameters
%       4. Noise and power parameters
%       5. Radar sensing parameters
%       6. Optimisation thresholds and convergence
%
%   Reference: Eq. (1)–(15) of Mamaghani et al., IEEE JSAC 2026, DOI: 10.1109/JSAC.2025.3611404.

%% 1. Antenna Array Configuration
% R-BS uses a Uniform Planar Array (UPA) for both transmit and receive.

L = 5;          % Number of single-antenna uplink (UL) users
K = 10;         % Number of single-antenna downlink (DL) users

M_tx_h = 5;    % Horizontal transmit antennas at R-BS
M_tx_v = 5;    % Vertical   transmit antennas at R-BS
M_rx_h = 5;    % Horizontal receive  antennas at R-BS
M_rx_v = 5;    % Vertical   receive  antennas at R-BS

Mt = M_tx_h * M_tx_v;  % Total transmit antennas (25)
Mr = M_rx_h * M_rx_v;  % Total receive  antennas (25)

%% 2. Channel and Propagation Parameters

fc          = 10e9;                        % Carrier frequency [Hz]  (10 GHz)
C_light     = physconst('LightSpeed');     % Speed of light [m/s]
wavelength  = C_light / fc;               % Carrier wavelength [m]

alpha_pathloss   = 3;       % Path-loss exponent (urban environment)
K_factor         = 10;      % Rician K-factor (ratio LoS/NLoS power) [linear]
excessivePL      = 1e-2;    % Additional NLoS attenuation factor

% Reference channel gain at 1 m, normalised by noise variance.
% invsigma2_factor corresponds to sigma2 = -110 dBm.
invsigma2_factor = 1e11;
beta_0 = invsigma2_factor * (wavelength / (4 * pi))^2;

%% 3. Signal and Bandwidth Parameters

B        = 30e6;              % Signal bandwidth [Hz]  (30 MHz)
gamma_BW = sqrt(pi^2 / 3);   % RMS-bandwidth normalisation factor (see CRLB derivation)

%% 4. Noise and Power Parameters

sigma2   = 1;           % Normalised receiver noise power (σ² = 1, i.e., noise-normalised model)
sigma2a  = sigma2;      % Additive noise variance at R-BS receive chain
sigma2e  = sigma2;      % Noise variance at the eavesdropper
sigma2k  = sigma2;      % Noise variance at each DL user
sigma2SI = sigma2;      % Self-interference noise variance (residual SI after cancellation)

% Default clutter-noise covariance (CNR = 1, i.e., 0 dB).
% In sweep scripts, CNR is updated before calling optimizeResources.
Rc     = sigma2 * (1 / Mr) * eye(Mr);
Sigman = (1 / Mr) * (sigma2a + sigma2SI) * eye(Mr) + Rc;

Pmax = 1e2;  % Maximum transmit power per UL user [W]

%% 5. Radar Sensing Parameters

RCS = 0.01 * (1 + 1j) / sqrt(2);  % Complex radar cross-section of the UAV target

%% 6. Optimisation Thresholds and Convergence

rho_ul  = 0.1;    % Minimum uplink secrecy rate threshold [bits/s/Hz]   (Constraint C1)
rho_dl  = 0.5;    % Minimum downlink secrecy rate threshold [bits/s/Hz]  (Constraint C2)
rho_est = 0.001;  % Maximum allowed CRLB [m²]                           (Constraint C3)
epsilon = 1e-3;   % SCA convergence tolerance (fractional error)
