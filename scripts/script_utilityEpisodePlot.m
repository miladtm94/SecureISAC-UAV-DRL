%% script_utilityEpisodePlot.m
%
%   Reproduce the UAV training utility figure from Mamaghani et al., 2025.
%
%   Loads pre-saved episode trajectory MAT files (output from the RL
%   training stage) and plots the smoothed per-episode utility U2 as a
%   function of training episode for three λ values:
%       λ = 0    → Minimise flight power only  (P_f)
%       λ = 0.5  → Balanced trade-off          (Ū2)
%       λ = 1    → Minimise network cost only  (U1)
%
%   Benchmarks:
%       Proposed (Smart) — UAV trajectory optimised with AN-aided R-BS
%       Without AN (Dumb) — UAV trajectory with no AN beamformer
%
%   Prerequisites:
%       - Run setup.m first.
%       - Episode trajectory files must exist in data/:
%             visitedPositionsPerEpisode_Smart_lambda*.mat
%             visitedPositionsPerEpisode_Dumb_lambda*.mat

clc; close all;
clearvars -except buffer trajSet

setup

%% =========================================================================
%  Configuration
%% =========================================================================
lambdaVec    = [0, 0.5, 1];   % Trade-off weight values
nLambda      = length(lambdaVec);
windowSize   = 50;             % Moving-average smoothing window (episodes)
nEpisodes    = 1000;           % Total training episodes

%% =========================================================================
%  Load pre-saved episode trajectories  (cached across calls)
%% =========================================================================
if ~exist('buffer', 'var')
    disp('Loading replay buffer ...');
    buffer = load('myBuffer.mat', 'buffer');
end

if ~exist('trajSet', 'var')
    disp('Loading episode trajectory files ...');
    trajSet = cell(1, 2 * nLambda);   % {Smart_l0, Smart_l1, ..., Dumb_l0, ...}
    for j = 1:nLambda
        lambda = lambdaVec(j);
        fp_smart = buildTrajFilename(lambda, true);
        fp_dumb  = buildTrajFilename(lambda, false);
        trajSet{j}          = load(fp_smart, 'allVisitedPosition');
        trajSet{nLambda + j} = load(fp_dumb,  'allVisitedPosition');
    end
end

%% =========================================================================
%  Compute per-episode utilities
%% =========================================================================
%  Utility(network, lambda, episode) where:
%      network 1 → Proposed (Smart)
%      network 2 → Without AN (Dumb)
Utility = zeros(2, nLambda, nEpisodes);

for j = 1:nLambda
    lambda = lambdaVec(j);
    for k = 1:nEpisodes
        % Proposed (Smart) trajectory
        traj_smart = trajSet{j}.allVisitedPosition(k).Positions;
        U_smart    = computeUAVUtility(traj_smart, lambda);
        Utility(1, j, k) = U_smart(1);   % Proposed utility component

        % Without AN (Dumb) trajectory
        traj_dumb = trajSet{nLambda + j}.allVisitedPosition(k).Positions;
        U_dumb    = computeUAVUtility(traj_dumb, lambda);
        Utility(2, j, k) = U_dumb(2);    % Baseline utility component
    end
end

%% =========================================================================
%  Plot (3 rows, one per λ)
%% =========================================================================
Pmax_flight = 607.9678;   % Max flight power for normalisation [W]

figure('Name', 'UAV Utility vs Training Episode', 'NumberTitle', 'off', ...
       'Position', [100 100 650 750]);

colorSet     = {'b', 'r', 'g'};
lineStyleSet = {'-', '--', '-.'};

tl = tiledlayout(nLambda, 1, 'TileSpacing', 'Compact', 'Padding', 'Compact');

for j = 1:nLambda
    nexttile;
    lambda = lambdaVec(j);

    % Label formatting
    if mod(lambda, 1) == 0
        lbl_smart = sprintf('Proposed, $\\lambda = %d$', lambda);
        lbl_dumb  = sprintf('Without AN, $\\lambda = %d$', lambda);
    else
        lbl_smart = sprintf('Proposed, $\\lambda = %.1f$', lambda);
        lbl_dumb  = sprintf('Without AN, $\\lambda = %.1f$', lambda);
    end

    u_smart = squeeze(Utility(1, j, :))';
    u_dumb  = squeeze(Utility(2, j, :))';

    if lambda == 0
        % λ=0: utility = −P_f → plot rescaled flight power
        y_smart = Pmax_flight * abs(movmean(u_smart / nEpisodes, windowSize));
        y_dumb  = Pmax_flight * abs(movmean(u_dumb  / nEpisodes, windowSize));
        yLabel  = '$P_f$ [kW]';
    elseif lambda == 1
        % λ=1: utility = U1 (NPC) → plot on log scale
        rescale_smart = 5.2190e+03;
        rescale_dumb  = 2.0662e+07;
        y_smart = rescale_smart * abs(movmean(u_smart / nEpisodes, windowSize));
        y_dumb  = rescale_dumb  * abs(movmean(u_dumb  / nEpisodes, windowSize));
        yLabel  = '$U_1$ [kW]';
    else
        % λ=0.5: balanced utility
        y_smart = movmean(u_smart / nEpisodes, windowSize);
        y_dumb  = movmean(u_dumb  / nEpisodes, windowSize);
        yLabel  = '$\bar{U}_2$';
    end

    if lambda == 1
        h1 = semilogy(1:nEpisodes, y_smart, 'Color', colorSet{j}, ...
            'LineStyle', lineStyleSet{j}, 'LineWidth', 1.5, ...
            'DisplayName', lbl_smart, 'MarkerIndices', 1:50:nEpisodes);
        hold on;
        h2 = semilogy(1:nEpisodes, y_dumb, 'k--', 'LineWidth', 1.5, ...
            'DisplayName', lbl_dumb, 'MarkerIndices', 1:50:nEpisodes);
    else
        h1 = plot(1:nEpisodes, y_smart, 'Color', colorSet{j}, ...
            'LineStyle', lineStyleSet{j}, 'LineWidth', 1.5, ...
            'DisplayName', lbl_smart, 'MarkerIndices', 1:50:nEpisodes);
        hold on;
        h2 = plot(1:nEpisodes, y_dumb, 'k--', 'LineWidth', 1.5, ...
            'DisplayName', lbl_dumb, 'MarkerIndices', 1:50:nEpisodes);
    end

    grid on;
    ylabel(yLabel, 'Interpreter', 'latex', 'FontSize', 14);
    legend([h1, h2], 'Interpreter', 'latex', 'FontSize', 11, 'Location', 'southeast');
end

xlabel('Training Episode', 'Interpreter', 'latex', 'FontSize', 12);
