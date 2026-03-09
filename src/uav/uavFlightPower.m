function Pf = uavFlightPower(v_xy, v_z)
%uavFlightPower  Compute rotary-wing UAV mechanical power consumption.
%
%   Pf = uavFlightPower(v_xy, v_z)
%
%   Models the total instantaneous power consumed by a multi-rotor UAV
%   as a function of its horizontal and vertical velocity components.
%   The model has four terms:
%       1. Blade profile power  — aerodynamic drag on the rotating blades
%       2. Parasitic power      — fuselage drag (proportional to v_xy³)
%       3. Climb power          — potential energy rate (proportional to v_z)
%       4. Induced power        — rotor-induced downwash (depends on v_xy)
%
%   Inputs:
%       v_xy  - Horizontal speed (x-y plane) [m/s]  (scalar or array)
%       v_z   - Vertical speed (signed; positive = climb) [m/s]
%
%   Output:
%       Pf    - Total mechanical power [W]  (same shape as v_xy)
%
%   UAV parameters (fixed, representative multi-rotor):
%       mass = 3 kg, U_tip = 120 m/s, nu_0 = 4.03 m/s
%
%   Reference: Eq. (UAV power model) in Zeng et al., 2019;
%              applied in Section IV-B of Mamaghani et al., IEEE JSAC 2026, DOI: 10.1109/JSAC.2025.3611404.

    %% Physical and aerodynamic constants
    g     = 9.81;    % Gravitational acceleration [m/s²]
    mass  = 3;       % UAV mass [kg]
    G0    = g * mass;  % UAV weight [N]

    Pb    = 79.86;   % Blade profile power in hover [W]
    Pi    = 88.63;   % Induced power in hover [W]
    C0    = 0.0092;  % Parasitic drag coefficient [W/(m/s)³]
    U_tip = 120;     % Blade tip speed [m/s]
    nu_0  = 4.03;    % Mean rotor induced velocity in hover [m/s]

    %% Four-term power model
    P_blade    = Pb .* (1 + 3 .* v_xy.^2 ./ U_tip.^2);           % (1) Blade profile
    P_parasit  = C0 .* v_xy.^3;                                    % (2) Parasitic
    P_climb    = G0 .* v_z;                                        % (3) Climb
    P_induced  = Pi .* sqrt( sqrt(1 + v_xy.^4 ./ (4 .* nu_0.^4)) ...
                            - v_xy.^2 ./ (2 .* nu_0.^2) );        % (4) Induced

    Pf = P_blade + P_parasit + P_climb + P_induced;
end
