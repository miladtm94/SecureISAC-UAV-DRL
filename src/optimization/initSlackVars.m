function slackvars = initSlackVars(params, channels, CNR)
%initSlackVars  Compute a strictly feasible initial point for all SCA slack variables.
%
%   slackvars = initSlackVars(params, channels, CNR)
%
%   Given an initial resource allocation (params) and channel realisations,
%   this function analytically evaluates the exact SINR values and derives
%   slack variable values that satisfy all SCA constraints at the expansion
%   point.  These are required as the starting iterate for the SCA loop.
%
%   The slack variables correspond to auxiliary variables introduced during
%   the SCA convexification of:
%       mu     — DL SINR intermediaries  (Constraint 34,  Eq. 34a/34b)
%       omega  — UL SINR intermediaries  (Constraint 35,  Eq. 35a/35b)
%       t      — Quadratic lifting vars for DL  (Constraint 41)
%       s      — Quadratic lifting vars for UL Eve  (Constraint 44)
%       iota   — Power-ratio lifting vars  (Constraint 48)
%
%   Inputs:
%       params    - Cell {P, V, W} (initial resource allocation)
%       channels  - Struct with .comm, .sensing1, .sensing2
%       CNR       - Clutter-to-noise ratio [linear]
%
%   Output:
%       slackvars - Struct with fields: mu, omega, t, s, iota
%
%   Reference: Appendix / Feasibility analysis of Mamaghani et al., IEEE JSAC 2026, DOI: 10.1109/JSAC.2025.3611404.

    sysParams   % Load system parameters

    % Noise covariance
    Rc     = CNR * sigma2 * (1/Mr) * eye(Mr);
    Sigman = (1/Mr) * (sigma2a + sigma2SI) * eye(Mr) + Rc;

    %% Unpack inputs
    [P_lo, V_lo, W_lo] = params{:};
    [h_la, h_el, h_lk, h_ak, h_ea] = channels.comm{:};
    [L, K] = size(h_lk);

    A_0    = channels.sensing1;
    zeta_0 = channels.sensing2;

    H_ea = h_ea * h_ea';

    % Aggregate covariance at expansion point
    SumVk_lo = zeros(size(V_lo, 1));
    for k = 1:K
        SumVk_lo = SumVk_lo + V_lo(:, :, k);
    end
    S_lo = SumVk_lo + W_lo;

    %% Retrieve true SINR values at the initial point
    [~, ~, ~, gamma_k_dl, gamma_ea_dl, ~, gamma_le_ul] = ...
        computeMetrics(params, channels, CNR);

    %% ---------------------------------------------------------------
    %  Slack variables  mu  (DL SINR intermediaries, Constraint 34)
    %  mu(K+1) = gamma_ea_dl  (Eve DL SINR — upper bound side)
    %  mu(1:K) = 2^(rho_dl + log2(1+mu(K+1))) - 1  (secrecy rate active)
    mu_lo         = zeros(K+1, 1);
    mu_lo(K+1)    = gamma_ea_dl;
    mu_lo(1:K)    = 2^(rho_dl + log2(1 + mu_lo(K+1))) - 1;

    %% ---------------------------------------------------------------
    %  Slack variables  omega  (UL SINR intermediaries, Constraint 35)
    %  omega(L+1:2L) = gamma_le_ul  (Eve UL SINR — upper bound side)
    %  omega(1:L)    = 2^(rho_ul + log2(1+omega(L+1:2L))) - 1
    omega_lo              = zeros(2*L, 1);
    omega_lo(L+1:end)     = gamma_le_ul(:);
    omega_lo(1:L)         = 2.^(rho_ul + log2(1 + omega_lo(L+1:end))) - 1;

    % Further tighten omega(1:L) to ensure Constraint 48 is feasible
    for l = 1:L
        MUI_lo = zeros(Mr);
        for lp = 1:L
            if lp ~= l
                MUI_lo = MUI_lo + P_lo(lp) * real(h_la(:,lp) * h_la(:,lp)');
            end
        end
        Psi_lo = MUI_lo + abs(zeta_0)^2 * real(A_0 * S_lo * A_0') + Sigman;
        QQ     = real(h_la(:,l)' * (Psi_lo \ h_la(:,l)));   % MMSE numerator term
        omega_lo(l) = min(P_lo(l) * QQ, omega_lo(l));
    end

    %% ---------------------------------------------------------------
    %  Slack variables  t  (quadratic lifting for DL SINR, Constraint 41)
    %  t(k)   = sqrt( mu_k  * (denominator of gamma_k^DL) )
    %  t(K+1) = sqrt( mu_{K+1} * (denominator of gamma_ea^DL) )
    t_lo = zeros(K+1, 1);
    for k = 1:K
        SumV_kp = zeros(size(V_lo, 1));
        for kp = 1:K
            if kp ~= k
                SumV_kp = SumV_kp + V_lo(:, :, kp);
            end
        end
        H_ak   = h_ak(:, k) * h_ak(:, k)';
        denom_k = sum(P_lo .* abs(h_lk(:,k)).^2) ...
                + real(trace((SumV_kp + W_lo) * H_ak)) + sigma2k;
        t_lo(k) = sqrt(mu_lo(k) * denom_k);
    end
    denom_ea  = sum(P_lo .* abs(h_el).^2) + real(trace(W_lo * H_ea)) + sigma2e;
    t_lo(K+1) = sqrt(mu_lo(K+1) * denom_ea);

    %% ---------------------------------------------------------------
    %  Slack variables  s  (quadratic lifting for Eve UL SINR, Constraint 44)
    %  s(l) = sqrt( p_l |h_el(l)|² / omega(L+l) )
    PP_lo = P_lo .* abs(h_el).^2;
    s_lo  = sqrt(PP_lo ./ omega_lo(L+1:end));

    %% ---------------------------------------------------------------
    %  Slack variables  iota  (power-ratio lifting, Constraint 48)
    %  iota(l) = sqrt( omega(l) )
    iota_lo = sqrt(omega_lo(1:L));

    %% Pack output
    slackvars.mu    = mu_lo;
    slackvars.omega = omega_lo;
    slackvars.s     = s_lo;
    slackvars.t     = t_lo;
    slackvars.iota  = iota_lo;
end
