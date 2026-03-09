%% mdpParams.m — UAV / MDP Grid and Flight Parameters
%
%   Defines the 3-D discretised state space, UAV kinematics, and
%   grid geometry used by the MDP / reinforcement-learning component
%   of the SecISAC framework (Player 2 optimisation).
%
%   Reference: Section IV-B of Mamaghani et al., IEEE JSAC 2026, DOI: 10.1109/JSAC.2025.3611404.

%% Grid Definition

grid_size = 10;   % Number of cells per side (10 × 10 footprint)
cell_size = 20;   % Physical size of each cell [m]

z_min = 1 * cell_size;   % Minimum UAV altitude [m]  (20 m)
z_max = 5 * cell_size;   % Maximum UAV altitude [m]  (100 m)

% Spatial boundaries of the 3-D flight region
Llimit = [-grid_size * cell_size / 2; -grid_size * cell_size / 2; z_min];
Ulimit = [ grid_size * cell_size / 2;  grid_size * cell_size / 2; z_max];

%% Grid Coordinates

grid_x = -grid_size * cell_size / 2 : 0.5 * cell_size : grid_size * cell_size / 2;
grid_y = -grid_size * cell_size / 2 : 0.5 * cell_size : grid_size * cell_size / 2;
grid_z = z_min : 0.25 * cell_size : z_max;

% Cell centres (x, y)
cell_centers_x = grid_x(1:end-1) + cell_size / 4;
cell_centers_y = grid_y(1:end-1) + cell_size / 4;

[X, Y] = meshgrid(cell_centers_x, cell_centers_y);
cell_centers = [X(:), Y(:)];   % All 2-D cell centres  [N_cells × 2]

% 3-D grid for potential UAV / eavesdropper positions
[X_cord, Y_cord, Z_cord] = ndgrid(cell_centers_x, cell_centers_y, grid_z);
Eve_Locs = [X_cord(:), Y_cord(:), Z_cord(:)];

%% UAV Initial and Final Positions

q_i = [cell_centers_x(1);   cell_centers_x(end); grid_z(end)];  % Initial position [m]
q_f = [cell_centers_x(end);  cell_centers_x(1);   grid_z(end)];  % Final   position [m]

regionRadius = 30;   % Radius of circular hover region around target [m]

%% UAV Kinematics

v_max_xy = sqrt(2) * cell_size;  % Maximum horizontal speed [m/s]
v_max_x  = cell_size;            % Maximum speed along x-axis [m/s]
v_max_y  = cell_size;            % Maximum speed along y-axis [m/s]
v_max_z  = 0.5 * cell_size;     % Maximum vertical speed [m/s]
delta_t  = 1;                    % Time step duration [s]

d_xy = v_max_xy * delta_t;   % Maximum horizontal displacement per step [m]
d_z  = v_max_z  * delta_t;   % Maximum vertical  displacement per step [m]

%% Mission Parameters

num_uplink_users   = 5;   % Number of UL users (must match sysParams L)
num_downlink_users = 10;  % Number of DL users (must match sysParams K)

N = 100;   % Maximum number of time steps per episode (mission horizon)
