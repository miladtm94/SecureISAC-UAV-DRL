function channels = simulateChannels(RBS_loc, ul_users_loc, dl_users_loc, eve_loc)
%simulateChannels  Generate all communication channel realisations for one snapshot.
%
%   channels = simulateChannels(RBS_loc, ul_users_loc, dl_users_loc, eve_loc)
%
%   Generates independent Rician channel realisations for all links in the
%   SecISAC system model at a single time snapshot.  The eavesdropper
%   (Eve) intercepts both the UL and DL transmissions.
%
%   Inputs:
%       RBS_loc       - R-BS 3-D position             [3 × 1]  [m]
%       ul_users_loc  - UL users' 3-D positions       [3 × L]  [m]
%       dl_users_loc  - DL users' 3-D positions       [3 × K]  [m]
%       eve_loc       - Eavesdropper 3-D position     [3 × 1]  [m]
%
%   Output:
%       channels  - Cell array {h_la, h_el, h_lk, h_ak, h_ea} where:
%           h_la  - UL channel (UL user l → R-BS Rx)    [Mr × L]  (complex)
%           h_el  - UL eavesdrop (UL user l → Eve)      [L  × 1]  (complex)
%           h_lk  - Cross channel (UL user l → DL user k) [L × K] (complex)
%           h_ak  - DL channel (R-BS Tx → DL user k)    [Mt × K]  (complex)
%           h_ea  - DL eavesdrop (R-BS Tx → Eve)        [Mt × 1]  (complex)
%
%   Reference: Eq. (3)–(5), system model of Mamaghani et al., IEEE JSAC 2026, DOI: 10.1109/JSAC.2025.3611404.

    sysParams   % Load L, K, Mt, Mr

    L = size(ul_users_loc, 2);
    K = size(dl_users_loc, 2);

    %% Preallocate channel matrices
    h_la = zeros(Mr, L);      % UL: L users → R-BS (Mr-antenna receive array)
    h_el = zeros(L, 1);       % Eve intercepts L uplink transmissions
    h_lk = zeros(L, K);       % Cross-link interference (UL user → DL user)
    h_ak = zeros(Mt, K);      % DL: R-BS (Mt-antenna transmit array) → K users
    h_ea = zeros(Mt, 1);      % Eve intercepts downlink (R-BS → Eve)

    %% Uplink channels  (UL users → R-BS and → Eve)
    for l = 1:L
        h_la(:, l) = ricianChannel(Mr, ul_users_loc(:, l), RBS_loc);
        h_el(l)    = ricianChannel(1,  eve_loc,             ul_users_loc(:, l));
    end

    %% Downlink channels  (R-BS → DL users and → Eve)
    for k = 1:K
        h_ak(:, k) = ricianChannel(Mt, RBS_loc, dl_users_loc(:, k));
    end
    h_ea = ricianChannel(Mt, RBS_loc, eve_loc);

    %% Cross-link channels  (UL user → DL user)
    for l = 1:L
        for k = 1:K
            h_lk(l, k) = ricianChannel(1, ul_users_loc(:, l), dl_users_loc(:, k));
        end
    end

    %% Pack into output cell array
    channels = {h_la, h_el, h_lk, h_ak, h_ea};
end
