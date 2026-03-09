function y_out = fillToLength(y, targetLen)
%fillToLength  Pad a convergence vector to a fixed target length.
%
%   y_out = fillToLength(y, targetLen)
%
%   Finds the last valid (non-NaN, positive) entry in y and repeats it
%   to fill positions beyond that index up to targetLen.  This is used
%   for consistent x-axis alignment when plotting convergence curves
%   across benchmarks that may converge at different iterations.
%
%   Inputs:
%       y         - Input row vector (may contain NaN after convergence)
%       targetLen - Desired output length (integer)
%
%   Output:
%       y_out     - Row vector of length targetLen with last valid value
%                   repeated to the end.  Returns zeros if y is all-NaN.

    idx = find(~isnan(y) & y > 0, 1, 'last');

    if isempty(idx)
        y_out = zeros(1, targetLen);
    else
        y_out          = y;
        y_out(idx+1:targetLen) = y(idx);   % Repeat last valid value
    end
end
