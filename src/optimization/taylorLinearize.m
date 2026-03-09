function f_approx = taylorLinearize(x, y, x0, y0)
%taylorLinearize  First-order Taylor approximation of f(x,y) = x² / y.
%
%   f_approx = taylorLinearize(x, y, x0, y0)
%
%   Computes the first-order Taylor expansion of f(x, y) = x²/y around
%   the expansion point (x0, y0):
%
%       f̃(x,y; x0,y0) = (x0/y0²) · (2·y0·x − x0·y)
%
%   This linearisation is used in the SCA procedure to convexify the
%   non-convex SINR ratio constraints (Constraints C1–C2).  Since f(x,y)
%   is jointly convex in (x,y) for y > 0, its first-order under-estimator
%   provides a valid convex lower bound at each SCA iteration.
%
%   Inputs:
%       x   - Current value (optimisation variable)
%       y   - Current value (optimisation variable)
%       x0  - Expansion point for x (previous SCA iterate)
%       y0  - Expansion point for y (previous SCA iterate)
%
%   Output:
%       f_approx  - Scalar (or array) first-order approximation value
%
%   Note: Arguments may be CVX variables when called inside a CVX block.
%
%   Reference: Eq. (42) / Lemma 1 of Mamaghani et al., IEEE JSAC 2026, DOI: 10.1109/JSAC.2025.3611404.

    f_approx = (x0 ./ y0.^2) .* (2 .* y0 .* x - x0 .* y);
end
