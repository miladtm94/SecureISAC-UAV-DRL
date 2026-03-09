function power = calcPower(params)
%calcPower  Decompose transmit parameters into individual power components.
%
%   power = calcPower(params)
%
%   Extracts the three power components from the optimisation variable
%   cell array {P, V, W}:
%
%       DL power   = Tr( Σ_k V_k )   [W] — downlink beamforming power
%       AN power   = Tr( W )          [W] — artificial noise / radar power
%       UL power   = Σ_l p_l          [W] — uplink user transmit power
%
%   The total network power consumption (NPC) is:
%       NPC = sum(power)
%
%   Input:
%       params - Cell array {P, V, W} where:
%           P  - UL power vector         [L × 1]         (real, ≥ 0)
%           V  - DL beamforming matrices [Mt × Mt × K]   (Hermitian PSD)
%           W  - AN/radar covariance     [Mt × Mt]        (Hermitian PSD)
%
%   Output:
%       power  - [1 × 3] vector:  [DL_power, AN_power, UL_power]  [W]
%
%   Reference: Objective function of Mamaghani et al., IEEE JSAC 2026, DOI: 10.1109/JSAC.2025.3611404 (Eq. 12).

    [P, V, W] = params{:};
    K = size(V, 3);

    %% Sum downlink beamforming matrices
    SumVk = zeros(size(V, 1));
    for k = 1:K
        SumVk = SumVk + V(:, :, k);
    end

    %% Return [DL power, AN/radar power, UL power]
    power = [trace(SumVk), trace(W), sum(P)];
end
