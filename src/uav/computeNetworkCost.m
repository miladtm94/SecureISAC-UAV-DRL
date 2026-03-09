function [NPC_Proposed, NPC_Baseline, NPC_WithoutAN] = computeNetworkCost(target_loc)
%computeNetworkCost  Run SCA to obtain converged NPC for a given UAV position.
%
%   [NPC_Proposed, NPC_Baseline, NPC_WithoutAN] = computeNetworkCost(target_loc)
%
%   For a fixed UAV (eavesdropper / target) 3-D location, this function:
%     1. Generates channel realisations via simulateChannels / sensingChannel.
%     2. Runs the SCA loop (optimizeResources) to convergence for:
%           flag = 0 → Proposed (with AN)
%           flag = 1 → Without AN
%     3. Returns the converged NPC for each scheme.
%
%   The "Baseline" NPC is the un-optimised initial power (max power, random
%   beamforming) — it corresponds to the first SCA iterate before any
%   optimisation.
%
%   Input:
%       target_loc  - 3-D position of the UAV / target  [x; y; z]  [m]
%
%   Outputs:
%       NPC_Proposed   - Converged NPC with AN beamformer  [W]
%       NPC_Baseline   - Initial (un-optimised) NPC        [W]
%       NPC_WithoutAN  - Converged NPC without AN          [W]
%
%   NOTE: This function is computationally expensive (runs two full SCA
%   loops per call).  It is called once per UAV step inside computeUAVUtility.
%
%   Reference: Section IV-B of Mamaghani et al., IEEE JSAC 2026, DOI: 10.1109/JSAC.2025.3611404.

    rng(0)   % Fix random seed for reproducibility of channel realisations

    sysParams   % Load system parameters and epsilon

    %% Load pre-computed ground terminal locations
    data = load('myGroundTerminalDist.mat', 'groundTerminalsLoc');
    [RBS_loc, ul_users_loc, dl_users_loc] = data.groundTerminalsLoc{:};

    %% Generate channel realisations for this UAV position
    CommunChannels      = simulateChannels(RBS_loc, ul_users_loc, dl_users_loc, target_loc);
    [A_0, zeta_0]       = sensingChannel(RBS_loc, target_loc);

    channels.comm     = CommunChannels;
    channels.sensing1 = A_0;
    channels.sensing2 = zeta_0;

    CNR    = 1;       % Default 0 dB clutter-to-noise ratio
    maxItr = 10;      % Safety cap on SCA iterations

    NPC = zeros(1, 3);   % [Baseline, Proposed, WithoutAN]

    %% SCA loop for each benchmark
    for flag = 0:1
        npc = [];
        i   = 1;

        [params, slackvars] = initFeasible(channels, flag, CNR);
        npc(i) = sum(calcPower(params));

        if flag == 0
            NPC(1) = npc(i);   % Baseline = un-optimised initial NPC
        end

        converged = false;
        while ~converged && i < maxItr
            i = i + 1;
            [Sol, slackvars] = optimizeResources(params, channels, slackvars, flag, CNR);
            params  = {Sol.P_opt, Sol.V_opt, Sol.W_opt};
            npc(i)  = sum(calcPower(params));

            FE = abs(npc(i) - npc(i-1)) / npc(i-1);
            if FE < epsilon
                converged = true;
            end
        end

        if flag == 0
            NPC(2) = npc(i);   % Proposed converged NPC
        else
            NPC(3) = npc(i);   % Without AN converged NPC
        end
    end

    NPC_Proposed  = NPC(2);
    NPC_Baseline  = NPC(1);
    NPC_WithoutAN = NPC(3);
end
