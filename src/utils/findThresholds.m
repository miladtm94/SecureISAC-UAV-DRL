function [k3, k4] = findThresholds(rho_ul, k4_min, k4_max, num_values)
%findThresholds  Enumerate (k3, k4) threshold pairs that satisfy UCSR = rho_ul.
%
%   [k3, k4] = findThresholds(rho_ul, k4_min, k4_max, num_values)
%
%   For a given uplink secrecy rate threshold rho_ul, computes the
%   corresponding minimum legitimate UL SINR k3 for a sweep of Eve UL
%   SINR values k4.  The mapping is derived from:
%
%       log2(1 + k3) − log2(1 + k4) = rho_ul
%       →  k3 = 2^{rho_ul} · (1 + k4) − 1
%
%   This is useful for plotting the feasible operating region in the
%   (k3, k4) SINR plane and for sensitivity analysis.
%
%   Inputs:
%       rho_ul     - Target uplink secrecy rate [bits/s/Hz]
%       k4_min     - Minimum Eve UL SINR value to sweep
%       k4_max     - Maximum Eve UL SINR value to sweep
%       num_values - Number of k4 sample points (integer)
%
%   Outputs:
%       k3  - Required legitimate UL SINR values  [1 × num_values]
%       k4  - Eve UL SINR sweep values            [1 × num_values]

    k4 = linspace(k4_min, k4_max, num_values);
    k3 = 2^rho_ul * (1 + k4) - 1;
end
