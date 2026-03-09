%% setup.m — Add all SecISAC source directories to the MATLAB path.
%
%   Run this script once from the repository root before executing any
%   other script or function in this project.
%
%   Usage:
%       >> cd /path/to/SecISAC-Mamaghani
%       >> setup
%
%   After setup, all src/ subdirectories are on the MATLAB path for the
%   current session.  To make the paths permanent, call savepath() after
%   running this script.

repoRoot = fileparts(mfilename('fullpath'));

addpath(fullfile(repoRoot, 'src', 'config'));
addpath(fullfile(repoRoot, 'src', 'channels'));
addpath(fullfile(repoRoot, 'src', 'optimization'));
addpath(fullfile(repoRoot, 'src', 'metrics'));
addpath(fullfile(repoRoot, 'src', 'uav'));
addpath(fullfile(repoRoot, 'src', 'utils'));
addpath(fullfile(repoRoot, 'scripts'));
addpath(fullfile(repoRoot, 'data'));

fprintf('[setup] SecISAC paths added successfully.\n');
fprintf('[setup] Repository root: %s\n', repoRoot);
