function filepath = buildResultsPath(isSmart, lambda, resultsDir)
%buildResultsPath  Construct the full path for saving a trained agent MAT file.
%
%   filepath = buildResultsPath(isSmart, lambda)
%   filepath = buildResultsPath(isSmart, lambda, resultsDir)
%
%   Builds a standardised file path for storing or loading trained RL
%   agent data files.  Creates the target directory if it does not exist.
%
%   Inputs:
%       isSmart    - true / 1 → "Smart" (proposed) agent
%                    false / 0 → "Dumb" (baseline) agent
%       lambda     - Trade-off weight  (e.g., 0, 0.5, 1)
%       resultsDir - (optional) Folder path.
%                    Defaults to 'data/results/' relative to the repo root.
%
%   Output:
%       filepath   - Full path string to the MAT file.
%
%   Example:
%       fp = buildResultsPath(true, 0.5)
%       % → 'data/results/TrainedAgent_Smart_lambda0_50.mat'

    if nargin < 3 || isempty(resultsDir)
        resultsDir = fullfile(fileparts(mfilename('fullpath')), '..', '..', 'data', 'results');
    end

    % Create directory if it does not exist
    if ~exist(resultsDir, 'dir')
        mkdir(resultsDir);
    end

    lambda_str = strrep(sprintf('lambda%0.2f', lambda), '.', '_');

    if isSmart
        strategy = 'Smart';
    else
        strategy = 'Dumb';
    end

    filename = sprintf('TrainedAgent_%s_%s.mat', strategy, lambda_str);
    filepath = fullfile(resultsDir, filename);
end
