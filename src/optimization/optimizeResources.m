function [Sol, slackvars] = optimizeResources(params, channels, slackvars, flag, CNR)
%optimizeResources  SCA-based resource allocation solver (one CVX iteration).
%
%   [Sol, slackvars] = optimizeResources(params, channels, slackvars, flag, CNR)
%
%   This is the core optimisation function implementing one iteration of
%   the Successive Convex Approximation (SCA) algorithm for Problem (P1):
%
%       minimise    Tr(Σ_k V_k) + Tr(W) + Σ_l p_l          (NPC)
%       subject to  DCSR ≥ rho_dl    (DL secrecy rate)       (C1)
%                   UCSR ≥ rho_ul    (UL secrecy rate)        (C2)
%                   CRLB ≤ rho_est   (ISAC sensing quality)   (C3)
%                   V_k ≽ 0,  W ≽ 0,  P ≥ 0
%
%   Non-convex constraints are successively convexified via first-order
%   Taylor approximations (see taylorLinearize.m) around the current
%   expansion point (params, slackvars).
%
%   Inputs:
%       params    - Cell {P_lo, V_lo, W_lo} — current SCA iterate
%       channels  - Struct with .comm, .sensing1, .sensing2
%       slackvars - Struct of current slack variable values (mu, omega, s, t, iota)
%       flag      - 0 → Proposed (W free), 1 → Without AN (W = 0)
%       CNR       - Clutter-to-noise ratio [linear]
%
%   Outputs:
%       Sol       - Struct with fields:
%                     .V_opt  [Mt×Mt×K]  — optimised DL beamforming matrices
%                     .W_opt  [Mt×Mt]    — optimised AN/radar covariance
%                     .P_opt  [L×1]      — optimised UL user powers
%                     .OF                — CVX optimal objective value
%       slackvars - Updated slack variable values at the new iterate
%
%   Solver: CVX with SDP mode.  MOSEK or SDPT3 is recommended.
%
%   Reference: Algorithm 1, Eq. (33)–(48) of Mamaghani et al., IEEE JSAC 2026, DOI: 10.1109/JSAC.2025.3611404.

    sysParams   % Load all system parameters

    % Noise covariance at R-BS (depends on CNR)
    Rc     = CNR * sigma2 * (1/Mr) * eye(Mr);
    Sigman = (1/Mr) * (sigma2a + sigma2SI) * eye(Mr) + Rc;

    %% Unpack current iterate
    [P_lo, V_lo, W_lo] = params{:};

    %% Unpack channel matrices
    [h_la, h_el, h_lk, h_ak, h_ea] = channels.comm{:};
    [L, K] = size(h_lk);
    Mt     = size(h_ak, 1);

    A_0    = channels.sensing1;
    zeta_0 = channels.sensing2;

    H_ea = h_ea * h_ea';

    %% Aggregate covariance at the expansion point  S_lo = Σ V_lo + W_lo
    SumVk_lo = zeros(Mt);
    for k = 1:K
        SumVk_lo = SumVk_lo + V_lo(:, :, k);
    end
    S_lo = SumVk_lo + W_lo;

    %% Unpack slack variables at the expansion point
    mu_lo    = slackvars.mu;
    omega_lo = slackvars.omega;
    s_lo     = slackvars.s;
    t_lo     = slackvars.t;
    iota_lo  = slackvars.iota;

    %% =====================================================================
    %  CVX Semidefinite Programme
    %  Variable dimensions:
    %    V(Mt,Mt,K)     — K DL beamforming covariance matrices
    %    W(Mt,Mt)       — AN / radar covariance (fixed to 0 if flag = 1)
    %    P(L,1)         — UL user transmit powers
    %    mu_opt(K+1,1)  — DL SINR slack variables  (Constraint 34)
    %    omega(2L,1)    — UL SINR slack variables   (Constraint 35)
    %    t(K+1,1)       — Quadratic lifting vars for DL  (Constraint 41)
    %    s(L,1)         — Quadratic lifting vars for UL Eve  (Constraint 44)
    %    iota(L,1)      — Power-ratio lifting vars   (Constraint 48)
    %  =====================================================================
    cvx_begin sdp quiet
        variable V(Mt, Mt, K) hermitian semidefinite
        variable W(Mt, Mt)    hermitian semidefinite
        variable P(L, 1)      nonnegative
        variable mu_opt(K+1, 1) nonnegative
        variable omega(2*L, 1)  nonnegative
        variable t(K+1, 1)      nonnegative
        variable s(L, 1)        nonnegative
        variable iota(L, 1)     nonnegative
        dual variable WW

        % Pre-compute Σ_k V_k (used repeatedly in constraints)
        SumVk = 0;
        for k = 1:K
            SumVk = SumVk + V(:, :, k);
        end

        %% Objective: minimise total network power consumption (NPC)
        %  NPC = Tr(Σ_k V_k) + Tr(W) + Σ_l p_l
        minimize( trace(SumVk) + trace(W) + sum(P) )

        subject to

        %% ---------------------------------------------------------------
        %  Constraint C1: DL Secrecy Rate ≥ rho_dl  (Eq. 33b)
        %  Convexified form via SCA — Constraints (34a)–(34c).
        %% ---------------------------------------------------------------

        for k = 1:K
            % Interference from other DL users (Σ_{k'≠k} V_{k'})
            SumV_kp = 0;
            for kp = 1:K
                if kp ~= k
                    SumV_kp = SumV_kp + V(:, :, kp);
                end
            end
            H_ak = h_ak(:,k) * h_ak(:,k)';

            % (41a) Upper bound on DL user-k SINR denominator
            %       [concave; linearised around (t_lo(k), mu_lo(k))]
            sum(P .* abs(h_lk(:,k)).^2) ...
                + real(trace((SumV_kp + W) * H_ak)) + sigma2k ...
                <= taylorLinearize(t(k), mu_opt(k), t_lo(k), mu_lo(k));

            % (41b) Lower bound on DL user-k numerator  (convex ≥)
            square_pos(t(k)) <= real(trace(V(:,:,k) * H_ak));
        end

        % (43a) Upper bound on Eve DL SINR denominator
        quad_over_lin(t(K+1), mu_opt(K+1)) <= ...
            sum(P .* abs(h_el).^2) + real(trace(W * H_ea)) + sigma2e;

        % (43b) Upper bound on Eve DL numerator (SCA linearisation)
        LHS = 0;
        for k = 1:K
            LHS = LHS + real(trace(V(:,:,k) * H_ea));
        end
        LHS <= -(t_lo(K+1))^2 + 2 * t(K+1) * t_lo(K+1);

        % (34c) DL secrecy rate: log2(1+mu_k) − log2(1+mu_{K+1}) ≥ rho_dl
        log(1 + mu_opt(1:K)) ...
            - (log(1 + mu_lo(K+1)) + (mu_opt(K+1) - mu_lo(K+1)) / (1 + mu_lo(K+1))) ...
            >= log(2) * rho_dl;

        %% ---------------------------------------------------------------
        %  Constraint C3: CRLB ≤ rho_est  (Eq. 45)
        %  Equivalent to: Tr(X' S X) ≥ c² / (8π² B_rms² rho_est)
        %  where X = |zeta_0| Σn^{-½} A_0  (whitened sensing matrix).
        %% ---------------------------------------------------------------
        S = W + SumVk;
        X = abs(zeta_0) * (Sigman^(-1/2)) * A_0;
        real(trace(X' * S * X)) >= (C_light^2) / (8 * gamma_BW^2 * B^2 * rho_est);

        %% ---------------------------------------------------------------
        %  Constraint C2: UL Secrecy Rate ≥ rho_ul  (Eq. 35)
        %  Convexified form — Constraints (48a)–(48b) and (44a)–(44b).
        %% ---------------------------------------------------------------

        % (48a/48b) UL SINR at R-BS lower bound
        for l = 1:L
            MUI    = 0;
            MUI_lo = 0;
            for lp = 1:L
                if lp ~= l
                    MUI    = MUI    + P(lp)    * real(h_la(:,lp) * h_la(:,lp)');
                    MUI_lo = MUI_lo + P_lo(lp) * real(h_la(:,lp) * h_la(:,lp)');
                end
            end
            Psi    = MUI    + abs(zeta_0)^2 * real(A_0 * S    * A_0') + Sigman;
            Psi_lo = MUI_lo + abs(zeta_0)^2 * real(A_0 * S_lo * A_0') + Sigman;

            % First-order Taylor expansion of MMSE output SINR (h' Ψ^{-1} h)
            QQ = real(h_la(:,l)' * (Psi_lo \ h_la(:,l)) ...
                 - h_la(:,l)' * (Psi_lo \ (Psi - Psi_lo)) * (Psi_lo \ h_la(:,l)));

            % (48a) p_l / iota_l ≤ QQ  →  iota_l² / p_l ≤ QQ
            quad_over_lin(iota(l), P(l)) - QQ <= 0;

            % (48b) omega_l ≤ 2 iota_lo · iota_l − iota_lo²  (SCA)
            omega(l) <= -(iota_lo(l))^2 + 2 * iota(l) * iota_lo(l);
        end

        % (44a/44b) UL SINR at Eve upper bound
        S = SumVk + W;
        for l = 1:L
            MUI_eve = 0;
            for lp = 1:L
                if lp ~= l
                    MUI_eve = MUI_eve + P(lp) * abs(h_el(lp))^2;
                end
            end

            % (44a)  s_l² ≤ Eve UL noise + interference
            square_pos(s(l)) <= MUI_eve + real(trace(S * H_ea)) + sigma2e;

            PP_lo = P_lo(l) * abs(h_el(l))^2;
            PP    = abs(h_el(l))^2 * P(l);

            % (44b)  1/omega(L+l) ≤ taylorLinearize(s_l, PP_l, …)
            inv_pos(omega(L+l)) <= taylorLinearize(s(l), PP, s_lo(l), PP_lo);

            % (35c) UL secrecy rate
            log(1 + omega(l)) ...
                - (log(1 + omega_lo(L+l)) + (omega(L+l) - omega_lo(L+l)) / (1 + omega_lo(L+l))) ...
                >= log(2) * rho_ul;
        end

        %% Variable domain constraints
        W >= 0 : WW;
        for k = 1:K
            V(:, :, k) >= 0;
        end
        P >= 0;

        if flag == 1   % Without AN: lock W to zero
            W == W_lo;
        end

    cvx_end

    %% =====================================================================
    %  Pack outputs
    %% =====================================================================
    Sol.V_opt = V;
    Sol.W_opt = W;
    Sol.P_opt = P;
    Sol.OF    = cvx_optval;

    slackvars.mu    = mu_opt;
    slackvars.omega = omega;
    slackvars.s     = s;
    slackvars.t     = t;
    slackvars.iota  = iota;
end
