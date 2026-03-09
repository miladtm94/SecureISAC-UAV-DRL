function filepath = buildTrajFilename(lambda, isSmart, resultsDir)
%buildTrajFilename  Construct the MAT filename for episode trajectory results.
%
%   filepath = buildTrajFilename(lambda, isSmart)
%   filepath = buildTrajFilename(lambda, isSmart, resultsDir)
%
%   Generates a standardised filename for the MAT file that stores the
%   visited positions per episode for a given (lambda, network type) pair.
%
%   Inputs:
%       lambda      - Trade-off weight  (e.g., 0, 0.5, 1)
%       isSmart     - true / 1 → "Smart" (proposed) UAV strategy
%                     false / 0 → "Dumb" (baseline) UAV strategy
%       resultsDir  - (optional) Folder path for results.
%                     Defaults to 'data/' relative to the repo root.
%
%   Output:
%       filepath    - Full path string to the MAT file.
%
%   Example:
%       fp = buildTrajFilename(0.5, true)
%       % → 'data/visitedPositionsPerEpisode_Smart_lambda0_50.mat'

    if nargin < 3 || isempty(resultsDir)
        % Default: data/ folder relative to the repo root
        resultsDir = fullfile(fileparts(mfilename('fullpath')), '..', '..', 'data');
    end

    % Lambda string: replace '.' with '_' for safe filename use
    lambda_str = strrep(sprintf('lambda%0.2f', lambda), '.', '_');

    if isSmart
        strategy = 'Smart';
    else
        strategy = 'Dumb';
    end

    filename = sprintf('visitedPositionsPerEpisode_%s_%s.mat', strategy, lambda_str);
    filepath = fullfile(resultsDir, filename);
end
