function [params, slackvars] = initFeasible(channels, flag, CNR)
%initFeasible  Find a strictly feasible starting point for the SCA resource allocation.
%
%   [params, slackvars] = initFeasible(channels, flag, CNR)
%
%   Constructs a random Hermitian-PSD initialisation for {V, W, P} and
%   then projects it onto the feasible set by solving a Phase-I CVX
%   feasibility problem (minimise 0 subject to all constraints).  The
%   resulting point satisfies all secrecy rate and CRLB constraints and
%   serves as the seed iterate for the SCA loop in optimizeResources.
%
%   Inputs:
%       channels  - Struct with .comm, .sensing1, .sensing2  (from simulateChannels)
%       flag      - Operating mode:
%                     0 → Proposed  (V, W, P all optimised)
%                     1 → Without AN  (W is fixed to zero)
%       CNR       - Clutter-to-noise ratio [linear]
%
%   Outputs:
%       params    - Cell {P, V, W} — feasible initial resource allocation
%       slackvars - Struct of feasible slack variable values (from initSlackVars)
%
%   Reference: Feasibility initialisation described in Section IV-A of
%              Mamaghani et al., IEEE JSAC 2026, DOI: 10.1109/JSAC.2025.3611404.

    sysParams   % Load system parameters

    % Noise covariance matrix at R-BS
    Rc     = CNR * sigma2 * (1/Mr) * eye(Mr);
    Sigman = (1/Mr) * (sigma2a + sigma2SI) * eye(Mr) + Rc;

    %% Unpack channels
    [h_la, h_el, h_lk, h_ak, h_ea] = channels.comm{:};
    A_0    = channels.sensing1;
    zeta_0 = channels.sensing2;

    Mt = size(h_ak, 1);
    [L, K] = size(h_lk);
    H_ea   = h_ea * h_ea';

    %% Random Hermitian-PSD initialisation
    % UL powers — start at maximum allowed power
    P_lo = Pmax * ones(L, 1);

    % DL beamforming matrices — rank-1 random PSD, normalised
    V_lo = zeros(Mt, Mt, K);
    for k = 1:K
        b         = randn(Mt, 1) + 1j * randn(Mt, 1);
        V_lo(:,:,k) = (b * b') / norm(b * b', 'fro');
    end

    % AN/radar covariance — rank-Mt random PSD, normalised (or zero if flag=1)
    if flag == 0
        B_W     = randn(Mt) + 1j * randn(Mt);
        W_lo    = (B_W * B_W') / norm(B_W * B_W', 'fro');
    else
        W_lo    = zeros(Mt);
    end

    % Compute initial aggregates
    SumVk_lo = zeros(Mt);
    for k = 1:K
        SumVk_lo = SumVk_lo + V_lo(:, :, k);
    end
    S_lo = SumVk_lo + W_lo;

    % Compute initial slack variable values based on the random initialisation
    params_init = {P_lo, V_lo, W_lo};
    slackvars   = initSlackVars(params_init, channels, CNR);

    mu_lo    = slackvars.mu;
    omega_lo = slackvars.omega;
    s_lo     = slackvars.s;
    t_lo     = slackvars.t;
    iota_lo  = slackvars.iota;

    %% CVX Phase-I Feasibility Problem (minimise 0 over constraint set)
    cvx_begin sdp
        variable V(Mt, Mt, K) hermitian semidefinite
        variable W(Mt, Mt)    hermitian semidefinite
        variable P(L, 1)      nonnegative
        variable mu_opt(K+1, 1) nonnegative
        variable omega(2*L, 1)  nonnegative
        variable t(K+1, 1)      nonnegative
        variable s(L, 1)        nonnegative
        variable iota(L, 1)     nonnegative
        dual variable WW

        SumVk = 0;
        for k = 1:K
            SumVk = SumVk + V(:, :, k);
        end

        minimize(0)   % Feasibility — no objective

        subject to

            %% Constraint 34: DL secrecy rate ≥ rho_dl
            for k = 1:K
                SumV_kp = 0;
                for kp = 1:K
                    if kp ~= k
                        SumV_kp = SumV_kp + V(:, :, kp);
                    end
                end
                H_ak = h_ak(:,k) * h_ak(:,k)';

                % (34a / 41a) Upper-bound DL denominator
                sum(P .* abs(h_lk(:,k)).^2) + real(trace((SumV_kp + W) * H_ak)) + sigma2k ...
                    <= taylorLinearize(t(k), mu_opt(k), t_lo(k), mu_lo(k));

                % (34a / 41b) Lower-bound DL numerator via lifted variable t
                square_pos(t(k)) <= real(trace(V(:,:,k) * H_ak));
            end

            % (34b / 43a) Eve DL SINR upper bound
            quad_over_lin(t(K+1), mu_opt(K+1)) <= ...
                sum(P .* abs(h_el).^2) + real(trace(W * H_ea)) + sigma2e;

            % (34b / 43b) Eve DL numerator — SCA linearisation
            LHS = 0;
            for k = 1:K
                LHS = LHS + real(trace(V(:,:,k) * H_ea));
            end
            LHS <= -(t_lo(K+1))^2 + 2 * t(K+1) * t_lo(K+1);

            % (34c) DL secrecy rate constraint
            log(1 + mu_opt(1:K)) - (log(1 + mu_lo(K+1)) ...
                + (mu_opt(K+1) - mu_lo(K+1)) / (1 + mu_lo(K+1))) >= log(2) * rho_dl;

            %% Constraint 45: CRLB (radar sensing quality)
            S = W + SumVk;
            X = abs(zeta_0) * (Sigman^(-1/2)) * A_0;
            real(trace(X' * S * X)) >= (C_light^2) / (8 * gamma_BW^2 * B^2 * rho_est);

            %% Constraint 35: UL secrecy rate ≥ rho_ul
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

                QQ = real(h_la(:,l)' * (Psi_lo \ h_la(:,l)) ...
                     - h_la(:,l)' * (Psi_lo \ (Psi - Psi_lo)) * (Psi_lo \ h_la(:,l)));

                % (48a) UL SINR lower bound (via iota)
                quad_over_lin(iota(l), P(l)) - QQ <= 0;

                % (48b) Lifted variable iota ↔ omega
                omega(l) <= -(iota_lo(l))^2 + 2 * iota(l) * iota_lo(l);
            end

            % Constraint 35b (44a/44b): Eve UL SINR upper bound
            S = SumVk + W;
            for l = 1:L
                MUI_eve = 0;
                for lp = 1:L
                    if lp ~= l
                        MUI_eve = MUI_eve + P(lp) * abs(h_el(lp))^2;
                    end
                end
                % (44a)
                square_pos(s(l)) <= MUI_eve + real(trace(S * H_ea)) + sigma2e;

                PP_lo = P_lo(l) * abs(h_el(l))^2;
                PP    = abs(h_el(l))^2 * P(l);
                % (44b)
                inv_pos(omega(L+l)) <= taylorLinearize(s(l), PP, s_lo(l), PP_lo);

                % (35c) UL secrecy rate constraint
                log(1 + omega(l)) - (log(1 + omega_lo(L+l)) ...
                    + (omega(L+l) - omega_lo(L+l)) / (1 + omega_lo(L+l))) >= log(2) * rho_ul;
            end

            %% Variable domain constraints
            W >= 0 : WW;
            for k = 1:K
                V(:, :, k) >= 0;
            end
            P >= 0;

            if flag == 1   % Without AN: fix W = 0
                W == W_lo;
            end

    cvx_end

    %% Pack outputs
    params    = {P, V, W};
    slackvars.mu    = mu_opt;
    slackvars.omega = omega;
    slackvars.s     = s;
    slackvars.t     = t;
    slackvars.iota  = iota;
end
