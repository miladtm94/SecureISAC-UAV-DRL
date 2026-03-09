%% script_convergencePlot.m
%
%   Reproduce the SCA convergence figure from Mamaghani et al., 2025.
%
%   Runs the SCA resource allocation loop to convergence for two benchmarks:
%       (0) Proposed  — V, W, P all jointly optimised (with AN)
%       (1) Without AN — W is fixed to zero
%
%   Generates a 4-panel figure showing convergence of:
%       Panel 1: NPC  (total network power consumption) [W]
%       Panel 2: UCSR (uplink secrecy rate)   [bits/s/Hz]
%       Panel 3: DCSR (downlink secrecy rate) [bits/s/Hz]
%       Panel 4: CRLB (Cramér-Rao lower bound for radar) [m²]
%   All plotted against SCA iteration index.
%
%   Prerequisites: Run setup.m first. Place myGroundTerminalDist.mat in data/.
%
%   Estimated runtime: ~5–15 min depending on solver.

clc; clear; close all;
rng(0);   % Fix seed for reproducibility

setup     % Add all SecISAC paths

%% =========================================================================
%  Simulation setup
%% =========================================================================
sysParams

target_loc = [25, 45, 50]';   % Fixed UAV / target 3-D position [m]

% Load pre-computed ground terminal positions
data = load('myGroundTerminalDist.mat', 'groundTerminalsLoc');
[RBS_loc, ul_users_loc, dl_users_loc] = data.groundTerminalsLoc{:};

% Generate channel snapshot
CommunChannels      = simulateChannels(RBS_loc, ul_users_loc, dl_users_loc, target_loc);
[A_0, zeta_0]       = sensingChannel(RBS_loc, target_loc);

channels.comm     = CommunChannels;
channels.sensing1 = A_0;
channels.sensing2 = zeta_0;

CNR    = 1;    % Clutter-to-noise ratio (0 dB)
maxItr = 10;   % Maximum SCA iterations

%% =========================================================================
%  Preallocate result arrays
%% =========================================================================
NPC_all  = NaN(2, maxItr);
UCSR_all = NaN(2, maxItr);
DCSR_all = NaN(2, maxItr);
CRLB_all = NaN(2, maxItr);
FE_all   = NaN(2, maxItr);

flag_labels = {'Proposed (with AN)', 'Without AN'};

%% =========================================================================
%  SCA loop — one pass per benchmark (flag = 0: Proposed, flag = 1: No AN)
%% =========================================================================
for flag = 0:1
    fprintf('\n[Benchmark: %s]\n', flag_labels{flag+1});

    i   = 1;
    npc = NaN(1, maxItr);

    [params, slackvars] = initFeasible(channels, flag, CNR);

    % Evaluate metrics at initial point
    npc(i) = sum(calcPower(params));
    [ucsr, dcsr, crlb] = computeMetrics(params, channels, CNR);

    NPC_all(flag+1, i)  = npc(i);
    UCSR_all(flag+1, i) = ucsr;
    DCSR_all(flag+1, i) = dcsr;
    CRLB_all(flag+1, i) = crlb;

    converged = false;

    while ~converged && i < maxItr
        i = i + 1;

        % One SCA iteration
        [Sol, slackvars] = optimizeResources(params, channels, slackvars, flag, CNR);
        params = {Sol.P_opt, Sol.V_opt, Sol.W_opt};

        npc(i) = sum(calcPower(params));
        [ucsr, dcsr, crlb] = computeMetrics(params, channels, CNR);

        NPC_all(flag+1, i)  = npc(i);
        UCSR_all(flag+1, i) = ucsr;
        DCSR_all(flag+1, i) = dcsr;
        CRLB_all(flag+1, i) = crlb;

        FE = abs(npc(i) - npc(i-1)) / npc(i-1);
        FE_all(flag+1, i) = FE;

        fprintf('  Iter %2d | NPC = %6.2f W | UCSR = %.3f | DCSR = %.3f | CRLB = %.4e | FE = %.2f%%\n', ...
            i-1, npc(i), ucsr, dcsr, crlb, 100*FE);

        if FE < epsilon
            converged = true;
        end
    end

    fprintf('[%s] Converged in %d iterations.  Initial NPC = %.2f W → Final NPC = %.2f W\n', ...
        flag_labels{flag+1}, i-1, npc(1), npc(i));
end

%% =========================================================================
%  Plot
%% =========================================================================
colors    = lines(2);
markers   = {'o', 's'};
linewidth = 2;
fontSize  = 12;
iterAxis  = 0 : maxItr-1;

figure('Name', 'SCA Convergence', 'NumberTitle', 'off', 'Position', [100 100 700 900]);

%% Panel 1: NPC
subplot(4, 1, 1); hold on; grid on; box on;
for f = 1:2
    y = fillToLength(NPC_all(f, :), maxItr);
    semilogy(iterAxis, y, ['-' markers{f}], 'Color', colors(f,:), ...
        'LineWidth', linewidth, 'DisplayName', flag_labels{f});
end
ylabel('NPC [W]', 'Interpreter', 'latex', 'FontSize', fontSize);
legend('show', 'Location', 'northeast');
ylim([50 120]);

%% Panel 2: UCSR
subplot(4, 1, 2); hold on; grid on; box on;
for f = 1:2
    y = fillToLength(UCSR_all(f, :), maxItr);
    semilogy(iterAxis, y, ['-' markers{f}], 'Color', colors(f,:), ...
        'LineWidth', linewidth, 'DisplayName', flag_labels{f});
end
yline(rho_ul, ':k', 'LineWidth', 1.5, 'DisplayName', '$\rho^{\mathrm{UL}}$');
ylabel('UCSR [b/s/Hz]', 'Interpreter', 'latex', 'FontSize', fontSize);
legend('show', 'Location', 'northeast', 'Interpreter', 'latex');

%% Panel 3: DCSR
subplot(4, 1, 3); hold on; grid on; box on;
for f = 1:2
    y = fillToLength(DCSR_all(f, :), maxItr);
    semilogy(iterAxis, y, ['-' markers{f}], 'Color', colors(f,:), ...
        'LineWidth', linewidth, 'DisplayName', flag_labels{f});
end
yline(rho_dl, ':k', 'LineWidth', 1.5, 'DisplayName', '$\rho^{\mathrm{DL}}$');
ylabel('DCSR [b/s/Hz]', 'Interpreter', 'latex', 'FontSize', fontSize);
legend('show', 'Location', 'northeast', 'Interpreter', 'latex');

%% Panel 4: CRLB
subplot(4, 1, 4); hold on; grid on; box on;
for f = 1:2
    y = fillToLength(CRLB_all(f, :), maxItr);
    semilogy(iterAxis, y, ['-' markers{f}], 'Color', colors(f,:), ...
        'LineWidth', linewidth, 'DisplayName', flag_labels{f});
end
yline(rho_est, ':k', 'LineWidth', 1.5, 'DisplayName', '$\rho^{\mathrm{est}}$');
xlabel('SCA Iteration', 'Interpreter', 'latex', 'FontSize', fontSize);
ylabel('CRLB [m$^2$]', 'Interpreter', 'latex', 'FontSize', fontSize);
legend('show', 'Location', 'east', 'Interpreter', 'latex');
