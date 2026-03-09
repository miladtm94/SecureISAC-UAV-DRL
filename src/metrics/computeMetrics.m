function [UCSR, DCSR, CRLB, gamma_k_dl, gamma_ea_dl, gamma_la_ul, gamma_le_ul] = ...
         computeMetrics(params, channels, CNR)
%computeMetrics  Evaluate all key performance metrics for a given resource allocation.
%
%   [UCSR, DCSR, CRLB, gamma_k_dl, gamma_ea_dl, gamma_la_ul, gamma_le_ul] = ...
%       computeMetrics(params, channels, CNR)
%
%   Computes the three main system-level KPIs and the underlying SINR
%   values for both uplink and downlink communications.
%
%   Inputs:
%       params    - Cell {P, V, W}:
%                     P  [L×1]       — UL user transmit powers
%                     V  [Mt×Mt×K]   — DL beamforming covariance matrices
%                     W  [Mt×Mt]     — AN/radar covariance matrix
%       channels  - Struct with fields:
%                     .comm     — {h_la, h_el, h_lk, h_ak, h_ea} (from simulateChannels)
%                     .sensing1 — A_0  [Mr×Mt]  sensing channel matrix
%                     .sensing2 — zeta_0 (scalar) sensing path-loss amplitude
%       CNR       - Clutter-to-noise ratio [linear] (default = 1 if omitted)
%
%   Outputs:
%       UCSR        - Uplink Secrecy Rate   [bits/s/Hz]  (worst-case over UL users)
%       DCSR        - Downlink Secrecy Rate [bits/s/Hz]  (worst-case over DL users)
%       CRLB        - Cramér-Rao Lower Bound for range estimation [m²]
%       gamma_k_dl  - DL SINR at legitimate user k          [1×K]
%       gamma_ea_dl - DL SINR at eavesdropper (DL link)     [scalar]
%       gamma_la_ul - UL SINR at R-BS for user l            [1×L]
%       gamma_le_ul - UL SINR at eavesdropper (UL link)     [1×L]
%
%   Reference: Eq. (16)–(24) of Mamaghani et al., IEEE JSAC 2026, DOI: 10.1109/JSAC.2025.3611404.

    %% Defaults and parameter loading
    if nargin < 3 || isempty(CNR)
        CNR = 1;   % Default: 0 dB clutter-to-noise ratio
    end

    sysParams   % Load sigma2, sigma2a, sigma2SI, sigma2k, sigma2e, B, C_light, gamma_BW

    % Noise covariance at R-BS receiver (includes clutter + self-interference)
    Rc     = CNR * sigma2 * (1/Mr) * eye(Mr);
    Sigman = (1/Mr) * (sigma2a + sigma2SI) * eye(Mr) + Rc;

    %% Unpack optimisation variables
    [P, V, W] = params{:};

    %% Unpack channel matrices
    [h_la, h_el, h_lk, h_ak, h_ea] = channels.comm{:};
    [L, K] = size(h_lk);

    A_0    = channels.sensing1;
    zeta_0 = channels.sensing2;

    %% Aggregate transmit covariance  S = Σ_k V_k + W
    SumVk = zeros(size(V, 1));
    for k = 1:K
        SumVk = SumVk + V(:, :, k);
    end
    S = SumVk + W;

    % Shorthand outer product for eavesdropper Tx channel
    H_ea = h_ea * h_ea';

    %% ---------------------------------------------------------------
    %  Downlink SINRs
    %  gamma_{k}^{DL}  = h_ak(:,k)' V_k h_ak(:,k)
    %                    / (σ²_k + h_ak(:,k)' W h_ak(:,k)
    %                             + Σ_{k'≠k} h_ak(:,k)' V_{k'} h_ak(:,k)
    %                             + Σ_l p_l |h_lk(l,k)|²)
    %  Ref: Eq. (17)
    gamma_k_dl = zeros(1, K);
    for k = 1:K
        % Inter-user DL interference
        interference_k = 0;
        for kp = 1:K
            if kp ~= k
                interference_k = interference_k + real(h_ak(:,k)' * V(:,:,kp) * h_ak(:,k));
            end
        end

        num_k   = real(h_ak(:,k)' * V(:,:,k) * h_ak(:,k));
        denom_k = sigma2k ...
                + real(h_ak(:,k)' * W * h_ak(:,k)) ...
                + sum(P .* abs(h_lk(:,k)).^2) ...
                + interference_k;

        gamma_k_dl(k) = num_k / denom_k;
    end

    % Eavesdropper DL SINR
    % gamma_{ea}^{DL} = h_ea' (Σ_k V_k) h_ea / (σ²_e + h_ea' W h_ea + Σ_l p_l |h_el(l)|²)
    % Ref: Eq. (18)
    gamma_ea_dl = real(h_ea' * SumVk * h_ea) ...
                / (sigma2e + real(h_ea' * W * h_ea) + sum(P .* abs(h_el).^2));

    %% DL Secrecy Rate  (worst-case over DL users, floored at 0)
    %  DCSR = max( min_k [ log2(1+gamma_k^DL) - log2(1+gamma_ea^DL) ], 0 )
    %  Ref: Eq. (20)
    DCSR = max(min(log2(1 + gamma_k_dl) - log2(1 + gamma_ea_dl)), 0);

    %% ---------------------------------------------------------------
    %  Uplink SINRs
    %  Optimal MMSE receiver at R-BS suppresses inter-user interference
    %  and the ISAC waveform clutter simultaneously.
    %  Ref: Eq. (21)–(22)
    gamma_la_ul = zeros(1, L);
    gamma_le_ul = zeros(1, L);

    for l = 1:L
        % Interference covariance at R-BS from other UL users (excluding l)
        MUI = zeros(Mr);
        for lp = 1:L
            if lp ~= l
                MUI = MUI + P(lp) * real(h_la(:,lp) * h_la(:,lp)');
            end
        end

        % Total noise+interference+clutter covariance seen at R-BS
        Psi = MUI + abs(zeta_0)^2 * real(A_0 * S * A_0') + Sigman;

        % UL SINR at R-BS (MMSE output SINR)
        gamma_la_ul(l) = P(l) * real(h_la(:,l)' * (Psi \ h_la(:,l)));

        % UL SINR at eavesdropper (single-antenna, no beamforming)
        MUI_eve = 0;
        for lp = 1:L
            if lp ~= l
                MUI_eve = MUI_eve + P(lp) * abs(h_el(lp))^2;
            end
        end
        gamma_le_ul(l) = (P(l) * abs(h_el(l))^2) ...
                        / (MUI_eve + real(h_ea' * S * h_ea) + sigma2e);
    end

    %% UL Secrecy Rate  (worst-case over UL users, floored at 0)
    %  UCSR = max( min_l [ log2(1+gamma_l^UL) - log2(1+gamma_le^UL) ], 0 )
    %  Ref: Eq. (23)
    UCSR = max(min(log2(1 + gamma_la_ul) - log2(1 + gamma_le_ul)), 0);

    %% ---------------------------------------------------------------
    %  Cramér-Rao Lower Bound for range estimation
    %  CRLB = c² / (8 π² B_rms² |zeta_0|² Tr(A_0' S A_0 Σn⁻¹))
    %  Ref: Eq. (24) (Fisher information approach)
    X    = abs(zeta_0) * (Sigman^(-1/2)) * A_0;   % Whitened sensing matrix
    CRLB = (C_light^2) / (8 * gamma_BW^2 * B^2 * real(trace(X' * S * X)));
end
