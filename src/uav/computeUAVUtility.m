function Utility = computeUAVUtility(traj, lambda)
%computeUAVUtility  Compute the cumulative UAV utility U2 over a trajectory.
%
%   Utility = computeUAVUtility(traj, lambda)
%
%   Evaluates Player 2's (UAV's) episode utility for a sequence of 3-D
%   waypoints.  At each step, the UAV utility is:
%
%       U2(n) = lambda · U1(q_{n+1}) − (1−lambda) · P_f(n)
%
%   where:
%       U1(q) — Network cost (NPC) achieved at UAV position q (via SCA)
%       P_f   — UAV mechanical flight power over the step [W]
%       lambda — Trade-off weight (0 = minimise flight power; 1 = minimise NPC)
%
%   Inputs:
%       traj    - UAV waypoint sequence  [3 × (N+1)]  (each column is [x;y;z] in m)
%       lambda  - Trade-off weight  (scalar, 0 ≤ lambda ≤ 1)
%
%   Output:
%       Utility - [1 × 3] cumulative utility vector:
%                   Utility(1) = sum U2  (Proposed, with AN)
%                   Utility(2) = sum U2  (Baseline, random beamforming)
%                   Utility(3) = sum U2  (Without AN)
%
%   Note: computeNetworkCost() is called for every step, which invokes
%         an SCA solve — this function is computationally intensive.
%
%   Reference: Eq. (UAV utility) in Section IV-B of Mamaghani et al., IEEE JSAC 2026, DOI: 10.1109/JSAC.2025.3611404.

    N = size(traj, 2) - 1;   % Number of time steps

    U2_Proposed   = zeros(1, N);
    U2_Baseline   = zeros(1, N);
    U2_WithoutAN  = zeros(1, N);
    U1_Proposed   = zeros(1, N);
    U1_Baseline   = zeros(1, N);
    U1_WithoutAN  = zeros(1, N);
    P_f           = zeros(1, N);

    Pmax_flight   = 607.9678;   % Maximum flight power for normalisation [W]

    for n = 1:N
        q_current = traj(:, n);
        q_next    = traj(:, n+1);

        %% Flight power at step n
        v_xy   = norm(q_next(1:2) - q_current(1:2));   % Horizontal speed [m/s]
        v_z    = q_next(3) - q_current(3);              % Vertical speed (signed) [m/s]
        P_f(n) = uavFlightPower(v_xy, v_z);

        %% Network cost at the next waypoint (SCA solve)
        [U1_Proposed(n), U1_Baseline(n), U1_WithoutAN(n)] = computeNetworkCost(q_next);

        %% Player 2 utility  U2 = lambda · U1 − (1−lambda) · P_f
        U2_Proposed(n)  = lambda * U1_Proposed(n)  - (1 - lambda) * P_f(n);
        U2_Baseline(n)  = lambda * U1_Baseline(n)  - (1 - lambda) * P_f(n);
        U2_WithoutAN(n) = lambda * U1_WithoutAN(n) - (1 - lambda) * P_f(n);
    end

    Utility = [sum(U2_Proposed), sum(U2_Baseline), sum(U2_WithoutAN)];
end
