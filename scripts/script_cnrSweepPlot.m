%% script_cnrSweepPlot.m
%
%   Reproduce the CNR sweep figure from Mamaghani et al., 2025.
%
%   Sweeps the Clutter-to-Noise Ratio (CNR) from −10 dB to +3 dB and
%   records the converged resource allocation for each CNR value.
%
%   Generates a 4-panel figure showing vs CNR [dB]:
%       Panel 1: Total NPC  (sum of all power)              [W]
%       Panel 2: DL power   (Tr(Σ_k V_k))                  [W]
%       Panel 3: AN/radar power  (Tr(W))                    [W]
%       Panel 4: UL power   (Σ_l p_l)                      [W]
%
%   Benchmarks compared:
%       (0) Proposed  (with AN)
%       (1) Without AN
%
%   Prerequisites: Run setup.m first. Place myGroundTerminalDist.mat in data/.
%
%   BUG FIX (vs original Plot_NoiseScale.m):
%       The original script iterated flag = 0:2 but preallocated arrays
%       of size (2, xVals, …), causing an out-of-bounds write at flag=2.
%       This script correctly limits the loop to flag ∈ {0, 1}.

clc; clear; close all;
rng(0);

setup
sysParams

%% =========================================================================
%  Channel setup (fixed snapshot for the sweep)
%% =========================================================================
target_loc = [25, 45, 50]';

data = load('myGroundTerminalDist.mat', 'groundTerminalsLoc');
[RBS_loc, ul_users_loc, dl_users_loc] = data.groundTerminalsLoc{:};

CommunChannels    = simulateChannels(RBS_loc, ul_users_loc, dl_users_loc, target_loc);
[A_0, zeta_0]     = sensingChannel(RBS_loc, target_loc);

channels.comm     = CommunChannels;
channels.sensing1 = A_0;
channels.sensing2 = zeta_0;

%% =========================================================================
%  CNR sweep parameters
%% =========================================================================
CNR_dB  = linspace(-10, 3, 10);   % CNR sweep range [dB]
nSweep  = length(CNR_dB);

nBenchmarks = 2;   % Proposed (flag=0) and Without AN (flag=1)

% Preallocate result arrays  [nBenchmarks × nSweep × 3 power components]
Power_all = NaN(nBenchmarks, nSweep, 3);
NPC_all   = NaN(nBenchmarks, nSweep);
UCSR_all  = NaN(nBenchmarks, nSweep);
DCSR_all  = NaN(nBenchmarks, nSweep);
CRLB_all  = NaN(nBenchmarks, nSweep);

flag_labels = {'Proposed (with AN)', 'Without AN'};

%% =========================================================================
%  Main sweep loop
%% =========================================================================
for iter = 1:nSweep
    CNR = db2pow(CNR_dB(iter));
    fprintf('\n--- CNR = %.1f dB ---\n', CNR_dB(iter));

    for flag = 0:1
        fprintf('  [%s]\n', flag_labels{flag+1});

        i   = 1;
        npc = NaN(1, 50);

        [params, slackvars] = initFeasible(channels, flag, CNR);
        npc(i) = sum(calcPower(params));

        converged = false;
        while ~converged
            i = i + 1;

            [Sol, slackvars] = optimizeResources(params, channels, slackvars, flag, CNR);
            params = {Sol.P_opt, Sol.V_opt, Sol.W_opt};

            Power = calcPower(params);
            npc(i) = sum(Power);

            [UCSR, DCSR, CRLB] = computeMetrics(params, channels, CNR);

            FE = abs(npc(i) - npc(i-1)) / npc(i-1);
            fprintf('    Iter %2d | NPC = %.2f W | FE = %.2f%%\n', i-1, npc(i), 100*FE);

            if FE < epsilon
                converged = true;
            end
        end

        fprintf('  Converged in %d iters. NPC: %.2f → %.2f W\n', i-1, npc(1), npc(i));

        % Store results at the converged point (benchmark index = flag+1)
        Power_all(flag+1, iter, :) = Power;
        NPC_all(flag+1, iter)      = npc(i);
        UCSR_all(flag+1, iter)     = UCSR;
        DCSR_all(flag+1, iter)     = DCSR;
        CRLB_all(flag+1, iter)     = CRLB;
    end
end

%% =========================================================================
%  Plot
%% =========================================================================
colors    = lines(nBenchmarks);
markers   = {'o', 's'};
linewidth = 2;
fontSize  = 12;

figure('Name', 'Performance vs CNR', 'NumberTitle', 'off', 'Position', [100 100 700 850]);

%% Panel 1: Total NPC
subplot(4, 1, 1); hold on; grid on; box on;
for f = 1:nBenchmarks
    plot(CNR_dB, NPC_all(f, :), ['-' markers{f}], 'Color', colors(f,:), ...
        'LineWidth', linewidth, 'DisplayName', flag_labels{f});
end
ylabel('NPC [W]', 'Interpreter', 'latex', 'FontSize', fontSize);
legend('show', 'Location', 'northwest');
xlim([CNR_dB(1) CNR_dB(end)]);

%% Panel 2: DL beamforming power  Tr(Σ V_k)
subplot(4, 1, 2); hold on; grid on; box on;
for f = 1:nBenchmarks
    plot(CNR_dB, squeeze(Power_all(f, :, 1)), ['-' markers{f}], 'Color', colors(f,:), ...
        'LineWidth', linewidth, 'DisplayName', flag_labels{f});
end
ylabel('DL Power [W]', 'Interpreter', 'latex', 'FontSize', fontSize);
xlim([CNR_dB(1) CNR_dB(end)]);

%% Panel 3: AN / radar power  Tr(W)
subplot(4, 1, 3); hold on; grid on; box on;
for f = 1:nBenchmarks
    plot(CNR_dB, squeeze(Power_all(f, :, 2)), ['-' markers{f}], 'Color', colors(f,:), ...
        'LineWidth', linewidth, 'DisplayName', flag_labels{f});
end
ylabel('AN/Radar Power [W]', 'Interpreter', 'latex', 'FontSize', fontSize);
xlim([CNR_dB(1) CNR_dB(end)]);

%% Panel 4: UL user power  Σ p_l
subplot(4, 1, 4); hold on; grid on; box on;
for f = 1:nBenchmarks
    plot(CNR_dB, squeeze(Power_all(f, :, 3)), ['-' markers{f}], 'Color', colors(f,:), ...
        'LineWidth', linewidth, 'DisplayName', flag_labels{f});
end
xlabel('CNR [dB]', 'Interpreter', 'latex', 'FontSize', fontSize);
ylabel('UL Power [W]', 'Interpreter', 'latex', 'FontSize', fontSize);
xlim([CNR_dB(1) CNR_dB(end)]);
