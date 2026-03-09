%% script_benchmarkComparison.m
%
%   Reproduce the benchmark comparison convergence figure from Mamaghani et al., 2025.
%
%   Compares the Proposed (with AN) and Without-AN benchmarks for a fixed
%   channel snapshot, running each SCA loop for up to maxIter iterations.
%   Plots NPC convergence, fractional error, and constraint satisfaction.
%
%   Prerequisites: Run setup.m first.  Place myGroundTerminalDist.mat in data/.

clc; clear; close all;
rng(0);

setup
sysParams

%% =========================================================================
%  Channel setup
%% =========================================================================
target_loc = [25, 45, 50]';

data = load('myGroundTerminalDist.mat', 'groundTerminalsLoc');
[RBS_loc, ul_users_loc, dl_users_loc] = data.groundTerminalsLoc{:};

CommunChannels    = simulateChannels(RBS_loc, ul_users_loc, dl_users_loc, target_loc);
[A_0, zeta_0]     = sensingChannel(RBS_loc, target_loc);

channels.comm     = CommunChannels;
channels.sensing1 = A_0;
channels.sensing2 = zeta_0;

CNR     = 1;
maxIter = 10;

%% =========================================================================
%  Preallocate
%% =========================================================================
Power_all = NaN(2, maxIter, 3);
NPC       = NaN(2, maxIter);
FE_all    = NaN(2, maxIter);
UCSR_all  = NaN(2, maxIter);
DCSR_all  = NaN(2, maxIter);
CRLB_all  = NaN(2, maxIter);

flag_labels = {'Proposed (with AN)', 'Without AN'};

%% =========================================================================
%  SCA loop
%% =========================================================================
for flag = 0:1
    fprintf('\n[Benchmark: %s]\n', flag_labels{flag+1});
    f = flag + 1;

    i = 1;
    [params, slackvars] = initFeasible(channels, flag, CNR);

    Power_all(f, i, :) = calcPower(params);
    NPC(f, i)          = sum(Power_all(f, i, :));
    [UCSR_all(f,i), DCSR_all(f,i), CRLB_all(f,i)] = computeMetrics(params, channels, CNR);

    converged = false;
    while ~converged && i < maxIter
        i = i + 1;

        [Sol, slackvars] = optimizeResources(params, channels, slackvars, flag, CNR);
        params = {Sol.P_opt, Sol.V_opt, Sol.W_opt};

        Power_all(f, i, :) = calcPower(params);
        NPC(f, i)          = sum(Power_all(f, i, :));
        [UCSR_all(f,i), DCSR_all(f,i), CRLB_all(f,i)] = computeMetrics(params, channels, CNR);

        FE_all(f, i) = abs(NPC(f,i) - NPC(f,i-1)) / NPC(f,i-1);
        fprintf('  Iter %2d | NPC = %.2f W | FE = %.2f%%\n', i-1, NPC(f,i), 100*FE_all(f,i));

        if FE_all(f, i) < epsilon
            converged = true;
        end
    end

    fprintf('[%s] Done. %d iters. NPC: %.2f → %.2f W\n', flag_labels{f}, i-1, NPC(f,1), NPC(f,i));
end

%% =========================================================================
%  Figure 1: NPC convergence and fractional error
%% =========================================================================
colors   = lines(2);
markers  = {'o', 's'};
lw       = 2;
fs       = 12;
iterAxis = 0 : maxIter-1;

figure('Name', 'NPC Convergence', 'NumberTitle', 'off');

subplot(2, 1, 1); hold on; grid on; box on;
for f = 1:2
    y = fillToLength(NPC(f, :), maxIter);
    semilogy(iterAxis, y, ['-' markers{f}], 'Color', colors(f,:), ...
        'LineWidth', lw, 'DisplayName', flag_labels{f});
end
ylabel('NPC [W]', 'Interpreter', 'latex', 'FontSize', fs);
legend('show', 'Location', 'best');

subplot(2, 1, 2); hold on; grid on; box on;
for f = 1:2
    y = fillToLength(FE_all(f, 2:end), maxIter);
    semilogy(1:maxIter, y, ['-' markers{f}], 'Color', colors(f,:), ...
        'LineWidth', lw, 'DisplayName', flag_labels{f});
end
xlabel('SCA Iteration', 'Interpreter', 'latex', 'FontSize', fs);
ylabel('Fractional Error', 'Interpreter', 'latex', 'FontSize', fs);
legend('show', 'Location', 'best');

%% =========================================================================
%  Figure 2: Constraint satisfaction vs iteration
%% =========================================================================
figure('Name', 'Constraint Satisfaction', 'NumberTitle', 'off');

% UCSR
subplot(3, 1, 1); hold on; grid on; box on;
for f = 1:2
    y = fillToLength(UCSR_all(f, :), maxIter);
    semilogy(iterAxis, y, ['-' markers{f}], 'Color', colors(f,:), ...
        'LineWidth', lw, 'DisplayName', flag_labels{f});
end
yline(rho_ul, '--k', 'LineWidth', 1.5, 'DisplayName', '$\rho^{\mathrm{UL}}$');
ylabel('UCSR [b/s/Hz]', 'Interpreter', 'latex', 'FontSize', fs);
title('Uplink Secrecy Rate', 'Interpreter', 'latex', 'FontSize', fs);
legend('show', 'Interpreter', 'latex');

% DCSR
subplot(3, 1, 2); hold on; grid on; box on;
for f = 1:2
    y = fillToLength(DCSR_all(f, :), maxIter);
    semilogy(iterAxis, y, ['-' markers{f}], 'Color', colors(f,:), ...
        'LineWidth', lw, 'DisplayName', flag_labels{f});
end
yline(rho_dl, '--k', 'LineWidth', 1.5, 'DisplayName', '$\rho^{\mathrm{DL}}$');
ylabel('DCSR [b/s/Hz]', 'Interpreter', 'latex', 'FontSize', fs);
title('Downlink Secrecy Rate', 'Interpreter', 'latex', 'FontSize', fs);
legend('show', 'Interpreter', 'latex');

% CRLB
subplot(3, 1, 3); hold on; grid on; box on;
for f = 1:2
    y = fillToLength(CRLB_all(f, :), maxIter);
    plot(iterAxis, y, ['-' markers{f}], 'Color', colors(f,:), ...
        'LineWidth', lw, 'DisplayName', flag_labels{f});
end
yline(rho_est, '--k', 'LineWidth', 1.5, 'DisplayName', '$\rho^{\mathrm{est}}$');
xlabel('SCA Iteration', 'Interpreter', 'latex', 'FontSize', fs);
ylabel('CRLB [m$^2$]', 'Interpreter', 'latex', 'FontSize', fs);
title('Radar Sensing (CRLB)', 'Interpreter', 'latex', 'FontSize', fs);
legend('show', 'Interpreter', 'latex');
