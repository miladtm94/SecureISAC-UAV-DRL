function h = ricianChannel(M, loc_i, loc_j)
%ricianChannel  Generate a Rician fading channel vector between two nodes.
%
%   h = ricianChannel(M, loc_i, loc_j)
%
%   Models the baseband channel from a single-antenna node at loc_j to an
%   M-antenna node at loc_i (or vice-versa, since the model is symmetric
%   in terms of magnitude).  Combines:
%       - Distance-dependent path loss (with LoS probability weighting)
%       - LoS component: deterministic phase-shifted array response
%       - NLoS component: Rayleigh (zero-mean complex Gaussian) fading
%
%   Inputs:
%       M      - Number of antennas at the multi-antenna node (integer ≥ 1)
%       loc_i  - 3-D position of the multi-antenna node  [x; y; z]  [m]
%       loc_j  - 3-D position of the single-antenna node [x; y; z]  [m]
%
%   Output:
%       h      - Complex channel vector  [M × 1]
%
%   Parameters loaded from sysParams:
%       beta_0, alpha_pathloss, K_factor, excessivePL, wavelength
%
%   Reference: Eq. (3)–(5) of Mamaghani et al., IEEE JSAC 2026, DOI: 10.1109/JSAC.2025.3611404.

    sysParams   % Load shared system parameters

    %% Path-loss computation
    P_LoS    = probLoS(loc_i, loc_j);
    distance = norm(loc_i - loc_j);

    % LoS and NLoS large-scale attenuation
    eta_LoS  = P_LoS       * beta_0 * distance^(-alpha_pathloss);
    eta_NLoS = (1 - P_LoS) * excessivePL * beta_0 * distance^(-alpha_pathloss);

    %% Deterministic LoS component (array phase response)
    % Phase shift across M antennas based on inter-element path difference
    phase_shift = exp(1j * 2 * pi * (0:M-1)' * distance / wavelength);
    g_LoS       = phase_shift;   % [M × 1]

    %% Stochastic NLoS component (Rayleigh / CN(0,I))
    g_NLoS = sqrt(1 / (K_factor + 1)) ...
           * (randn(M, 1) + 1j * randn(M, 1)) / sqrt(2);   % [M × 1]

    %% Combined Rician channel
    h = sqrt(eta_LoS) .* g_LoS + sqrt(eta_NLoS) .* g_NLoS;
end
