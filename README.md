# Securing Integrated Sensing and Communication Against a Mobile Adversary: A Stackelberg Game With Deep Reinforcement Learning #

**Companion code for:**

This repository contains the MATLAB simulation code for the paper:

> Milad Tatar Mamaghani et el., "[Securing Integrated Sensing and Communication Against a Mobile Adversary: A Stackelberg Game With Deep Reinforcement Learning](https://ieeexplore.ieee.org/abstract/document/11172669),"  IEEE Journal on Selected Areas in Communications, vol. 44, pp. 942-958, 2026.

[![MATLAB](https://img.shields.io/badge/MATLAB-R2022b%2B-blue)](https://www.mathworks.com/)
[![CVX](https://img.shields.io/badge/CVX-2.2-orange)](http://cvxr.com/cvx/)
[![License: MIT](https://img.shields.io/badge/License-MIT-green.svg)](LICENSE)

## Abstract of Article ##
In this paper, we study a secure integrated sensing and communication (ISAC) system employing a full-duplex base station with sensing capabilities against a mobile proactive adversarial target—a malicious uncrewed aerial vehicle (M-UAV). We develop a game-theoretic model to enhance communication security, radar sensing accuracy, and power efficiency. The interaction between the legitimate network and the mobile adversary is formulated as a non-cooperative Stackelberg game (NSG), where the M-UAV acts as the leader and strategically adjusts its trajectory to improve its eavesdropping ability while conserving power and avoiding obstacles. In response, the legitimate network, acting as the follower, dynamically allocates resources to minimize network power usage while ensuring required secrecy rates and sensing performance. To address this challenging problem, we propose a low-complexity successive convex approximation (SCA) method for network resource optimization combined with a deep reinforcement learning (DRL) algorithm for adaptive M-UAV trajectory planning through sequential interactions and learning. Simulation results demonstrate the efficacy of the proposed method in addressing security challenges of dynamic ISAC systems in 6G, i.e., achieving a Stackelberg equilibrium with robust performance while mitigating the adversary’s ability to intercept network signals.


---

## Overview

This repository provides the full MATLAB simulation code for a **Secure Integrated Sensing and Communication (ISAC)** system modelled as a two-player game:

- **Player 1 (Radar Base Station / R-BS):** Jointly optimizes downlink communication beamforming matrices **V**, radar sensing covariance **W**, and uplink user power allocation **P** to minimise total network power consumption (NPC) subject to physical-layer security and sensing quality constraints.
- **Player 2 (Malicious UAV/ M-UAV):** Learns an optimal 3-D trajectory via a Markov Decision Process (MDP) / reinforcement learning framework to maximise a utility that trades off communication/sensing quality against flight power.

Key features implemented:
- **Successive Convex Approximation (SCA)** for the non-convex resource allocation problem (solved via CVX/SDP)
- **Rician channel model** with LoS probability for both communication and sensing links
- **Cramér–Rao Lower Bound (CRLB)**-based radar sensing constraint
- **Physical-layer secrecy** constraints on both uplink and downlink secrecy rates (UCSR / DCSR)
- **Artificial noise (AN)** jamming beamformer as a design variable
- UAV **flight power model** (blade profile + parasitic + climb + induced)
- Benchmark comparison: *Proposed* (with AN) vs *Without AN*

---

## Implementation  ##
The code implements a Double DQN (DDQN) agent that jointly optimises the M-UAV trajectory and physical-layer resource allocation to maximise an ISAC utility while minimising flight power, subject to uplink/downlink secrecy, CRLB-based sensing, and obstacle avoidance constraints.

---

## Repository Structure

```
SecureISAC-UAV-DRL/
│
├── main.m                   ← Entry point: train & simulate the DDQN agent
├── setup.m                  ← Run once per session to add all paths
│
├── config/
│   ├── sysParams.m          ← Physical-layer constants (antenna, channel, power)
│   └── MDPsimulationParams.m← Gridworld definition (grid size, UAV speed, N)
│
├── environment/             ← MDP environment (MATLAB RL Toolbox)
│   ├── createEnv.m          ← Builds rlFunctionEnv with discrete action space
│   ├── stepFunctionD.m      ← One MDP step: move, check constraints, compute reward
│   ├── resetFunction.m      ← Episode initialisation
│   ├── rewardFunc.m         ← Reward shaping: main + distance + loop penalties
│   ├── getMainReward.m      ← Reward from buffer: U1 utility and P_f
│   ├── getmilestoneReward.m ← Milestone reward along path to goal
│   ├── detectLoop.m         ← Loop detection (recent position window)
│   ├── detectLoopTraj.m     ← Trajectory-level loop detection
│   └── checkTrajConstraints.m← Post-hoc constraint verification
│
├── agent/
│   └── createDDQNAgent.m    ← DDQN agent: Q-network, epsilon-greedy, replay buffer
│
├── optimization/            ← Resource optimisation (SCA-based, offline)
│   ├── bufferCalc.m         ← [OFFLINE] Builds myBuffer.mat — DO NOT re-run lightly
│   ├── solveOptimization.m  ← Per-location: channel → SCA → metrics
│   ├── OptimResources.m     ← SCA problem for V, W, P (downlink / radar / uplink)
│   ├── OptimResources_Clutter_WithoutAN.m ← Variant: clutter, no AN
│   ├── power_optimization.m ← Wrapper: feasible init + SCA iterations
│   ├── feasible_initialization.m ← Warm-start for SCA
│   ├── initializationSCA_clutterComp_withoutAN.m
│   ├── initFeasible.m
│   ├── initSlackVars.m
│   ├── checkFeasibility.m   ← Validates SCA constraint satisfaction
│   ├── rank1_approx.m       ← Rank-1 approximation for SDR solutions
│   └── find_thresholds.m    ← Compute QoS thresholds
│
├── channel/                 ← Channel models
│   ├── channelSim.m         ← Full uplink/downlink channel simulation
│   ├── sensing_channel.m    ← UPA steering vectors and A_0 for ISAC radar
│   ├── rician_channel.m     ← Rician fading channel generation
│   ├── updateChannels.m     ← Channel update at new UAV location
│   └── prob_LoS.m           ← LoS probability (urban air-to-ground model)
│
├── metrics/                 ← Performance metric calculations
│   ├── metrics_calc.m       ← UCSR, DCSR, CRLB, SINR for given P/V/W
│   ├── observed_metrics.m   ← Lookup metrics from buffer (with caching)
│   ├── calculate_utility.m  ← Episode-level utility (post-processing)
│   ├── calc_U1.m            ← U1 (total transmission power)
│   ├── calc_power.m         ← Instantaneous power breakdown
│   └── Calc_Utility_Updated.m← Updated utility calculation helper
│
├── utils/                   ← General-purpose utilities
│   ├── flightPow.m          ← UAV rotary-wing flight power model
│   ├── loc_init.m           ← Ground terminal and user location initialisation
│   ├── generateObstacles.m  ← Random obstacle placement in the region
│   ├── eveLocationKey.m     ← String key for buffer indexing by Eve location
│   ├── getResultsFolderPath.m← Build result save-path from lambda/network params
│   ├── getEpisodeTrajName.m ← Build episode trajectory filename
│   ├── fill_to_maxIter.m    ← Pad result arrays to fixed length
│   ├── helperfunc.m         ← Miscellaneous shared helpers
│   └── plot_rewards.m       ← Quick reward-curve helper
│
├── plots/                   ← Plotting scripts (paper figures)
│   ├── Plot_Trajectories.m  ← 3-D trajectory comparison (Fig. X in paper)
│   ├── Plot_reward_Episode.m← Episodic reward & utility curves
│   ├── Plot_Utility_Episode.m← Per-episode utility breakdown
│   ├── Plot_Utility.m       ← Final utility comparison across lambda
│   ├── Plot_Convergence.m   ← SCA convergence plot
│   ├── Plot_convergence.m   ← DRL training convergence plot
│   ├── Plot_FlightPower.m   ← Flight power vs. velocity curves
│   ├── Plot_RewardVisulization.m ← Reward landscape heatmap
│   ├── visualize_system.m   ← 3-D scene (UAV, BS, users)
│   └── visualize_system_obstacle.m ← Scene with obstacle cylinders
│
├── benchmarks/              ← Benchmark / comparison plots
│   ├── CompPlot.m           ← Algorithm comparison across baselines
│   ├── Metrics_Comp.m       ← UCSR/DCSR/CRLB metrics comparison
│   └── Plot_NoiseScale.m    ← Noise scaling sensitivity analysis
│
└── data/                    ← Large data files (NOT tracked by Git)
    └── .gitkeep             ← Placeholder — see Data section below
```

---

## Getting Started

### Requirements

| Software | Version |
|---|---|
| MATLAB | R2022b or later |
| Reinforcement Learning Toolbox | Required |
| Optimization Toolbox | Required |
| Phased Array System Toolbox | Required (for UPA steering vectors) |
| CVX | Required for SCA-based resource optimisation |

Install CVX from [cvxr.com](http://cvxr.com/cvx/).

### Data Files

The `data/` folder must contain the following `.mat` files **before** running any script. These are not tracked by Git due to their size.

| File | Description | How to obtain |
|---|---|---|
| `myBuffer.mat` | Pre-computed optimal resource allocation (P, V, W) and metrics for every UAV grid location | Run `optimization/bufferCalc.m` (takes hours — see note below) |
| `myGroundTerminalDist.mat` | BS, uplink user and downlink user 3-D locations | Run `utils/loc_init.m` and save |
| `ObstacleLocs.mat` | Obstacle centres and radii | Run `utils/generateObstacles.m` and save |

> ⚠️ **Buffer generation is computationally expensive.** `bufferCalc.m` calls `solveOptimization.m` for every point in the 3-D Eve/UAV grid (~6 800 locations). Pre-generated files will be made available via the paper's data repository link.

### Installation

```matlab
% 1. Clone the repository
% git clone https://github.com/<your-username>/SecureISAC-UAV-DRL.git

% 2. Open MATLAB and navigate to the project root
cd('path/to/SecureISAC-UAV-DRL')

% 3. Add all subfolders to the MATLAB path (run once per session)
setup

% 4. Place required .mat files in data/

% 5. Run the main script
main
```

---

## Usage

### Training the DDQN Agent

Open `main.m` and set the control flags:

```matlab
doTraining       = 1;    % 1: train from scratch
doSimulation     = 1;    % 1: simulate after training
lambda           = 0.5;  % trade-off weight  ∈ {0, 0.5, 1}
networkSelection = 1;    % 1: proposed | 0: benchmark
withObstacle     = 1;    % 1: obstacles enabled
maxStep_init     = 1e3;  % number of training episodes
```

Then run:
```matlab
>> setup
>> main
```

Trained agents are saved automatically to `results/` (created on first run).

### Resuming Training

```matlab
doTraining = 2;          % resume from last checkpoint
maxStep_further = 2e3;   % additional episodes
```

### Simulation Only

```matlab
doTraining   = 3;   % load saved agent, skip training
doSimulation = 1;   % run simulation and plot trajectory
```

### Generating Paper Figures

Each plotting script is self-contained. After training agents for `lambda ∈ {0, 0.5, 1}`:

```matlab
>> setup
>> Plot_Trajectories        % 3-D trajectory figure
>> Plot_reward_Episode      % training reward / utility curves
>> Plot_Utility_Episode     % per-episode utility breakdown
>> CompPlot                 % benchmark comparison
```

---

## System Model


The ISAC system consists of:

- A multi-antenna **Base Station (BS)** equipped with a UPA serving:
  - `L = 5` uplink (sensing-aided) users
  - `K = 10` downlink users
- A **UAV target / eavesdropper (Eve)** navigating the 3-D region
- A **10×10 horizontal grid** with 5 altitude layers; cell size = 20 m

**Optimised variables:**
- `P` — uplink power allocation vector (L×1)
- `V` — downlink beamforming covariance matrices (K×K×K)
- `W` — radar / AN covariance matrix

**UAV trajectory (MDP):**
- State: (x, y, z, dist-to-goal, time-step)
- Action: 6 directions × 2 speed levels + hover = 13 discrete actions
- Reward: main utility + distance shaping + loop penalty + constraint penalties

---

## Key Files Reference

| Script | Purpose |
|---|---|
| `main.m` | Train / simulate DDQN agent |
| `setup.m` | Configure MATLAB path |
| `config/sysParams.m` | All physical-layer constants |
| `config/MDPsimulationParams.m` | Grid, speed, timing parameters |
| `environment/createEnv.m` | MDP environment factory |
| `environment/stepFunctionD.m` | Discrete step: transition + reward |
| `agent/createDDQNAgent.m` | DDQN with ε-greedy exploration |
| `optimization/bufferCalc.m` | Offline buffer builder (DO NOT re-run) |
| `optimization/OptimResources.m` | SCA resource optimisation (CVXPY) |
| `plots/Plot_Trajectories.m` | 3-D trajectory visualisation |
| `benchmarks/CompPlot.m` | Comparison with baselines |

---

## Citation

If you use this code in your research, please cite:

```bibtex
@article{mamaghani2025secureisac,
  author={Tatar Mamaghani, Milad and Zhou, Xiangyun and Yang, Nan and Lee Swindlehurst, A.},
  journal={IEEE Journal on Selected Areas in Communications}, 
  title={Securing Integrated Sensing and Communication Against a Mobile Adversary: A Stackelberg Game With Deep Reinforcement Learning}, 
  year={2026},
  volume={44},
  number={},
  pages={942-958},
 doi={10.1109/JSAC.2025.3611404}}
}
```

---

## License

This project is released under the MIT License. See `LICENSE` for details.
