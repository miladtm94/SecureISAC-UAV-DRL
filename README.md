# SecISAC: Secure ISAC Game — MATLAB Simulation Codebase

> **Companion code for:**
> M. T. Mamaghani, X. Zhou, N. Yang, and A. L. Swindlehurst, "Securing Integrated Sensing and Communication Against a Mobile Adversary: A Stackelberg Game With Deep Reinforcement Learning," *IEEE Journal on Selected Areas in Communications*, vol. 44, pp. 942–958, 2026. DOI: [10.1109/JSAC.2025.3611404](https://doi.org/10.1109/JSAC.2025.3611404)

[![MATLAB](https://img.shields.io/badge/MATLAB-R2022b%2B-blue)](https://www.mathworks.com/)
[![CVX](https://img.shields.io/badge/CVX-2.2-orange)](http://cvxr.com/cvx/)
[![License: MIT](https://img.shields.io/badge/License-MIT-green.svg)](LICENSE)

---

## Overview

This repository provides the full MATLAB simulation code for a **Secure Integrated Sensing and Communication (ISAC)** system modelled as a two-player game:

- **Player 1 (Radar Base Station / R-BS):** Jointly optimizes downlink communication beamforming matrices **V**, radar sensing covariance **W**, and uplink user power allocation **P** to minimise total network power consumption (NPC) subject to physical-layer security and sensing quality constraints.
- **Player 2 (UAV):** Learns an optimal 3-D trajectory via a Markov Decision Process (MDP) / reinforcement learning framework to maximise a utility that trades off communication/sensing quality against flight power.

Key features implemented:
- **Successive Convex Approximation (SCA)** for the non-convex resource allocation problem (solved via CVX/SDP)
- **Rician channel model** with LoS probability for both communication and sensing links
- **Cramér–Rao Lower Bound (CRLB)**-based radar sensing constraint
- **Physical-layer secrecy** constraints on both uplink and downlink secrecy rates (UCSR / DCSR)
- **Artificial noise (AN)** jamming beamformer as a design variable
- UAV **flight power model** (blade profile + parasitic + climb + induced)
- Benchmark comparison: *Proposed* (with AN) vs *Without AN*

---

## Repository Structure

```
SecISAC-Mamaghani/
├── README.md                        ← This file
├── CITATION.cff                     ← Citation metadata (GitHub-standard)
├── setup.m                          ← Run once to add all paths to MATLAB path
│
├── src/
│   ├── config/
│   │   ├── sysParams.m              ← Global system parameters (antennas, SNR, thresholds …)
│   │   └── mdpParams.m              ← UAV / MDP grid and flight parameters
│   │
│   ├── channels/
│   │   ├── simulateChannels.m       ← Wrapper: generates all comm. channels for one snapshot
│   │   ├── ricianChannel.m          ← Rician fading channel vector generator
│   │   ├── sensingChannel.m         ← Radar sensing matrix A_0 and path-loss zeta_0
│   │   └── probLoS.m                ← ITU/3GPP urban LoS probability model
│   │
│   ├── optimization/
│   │   ├── optimizeResources.m      ← Main SCA solver (CVX SDP) — core contribution
│   │   ├── initFeasible.m           ← Finds a strictly feasible starting point via CVX
│   │   ├── initSlackVars.m          ← Computes feasible initial slack variable values
│   │   └── taylorLinearize.m        ← First-order Taylor linearization helper (x²/y)
│   │
│   ├── metrics/
│   │   ├── computeMetrics.m         ← UCSR, DCSR, CRLB, and individual SINRs
│   │   └── calcPower.m              ← Decomposes {V,W,P} into [DL-power, AN-power, UL-power]
│   │
│   ├── uav/
│   │   ├── uavFlightPower.m         ← Rotary-wing UAV power model (horizontal + vertical flight)
│   │   ├── computeUAVUtility.m      ← Episode-level UAV utility U2 over a trajectory
│   │   └── computeNetworkCost.m     ← Converged NPC (U1) for a given UAV position
│   │
│   └── utils/
│       ├── fillToLength.m           ← Pads a convergence vector to a fixed length
│       ├── findThresholds.m         ← Maps rho_ul → feasible (k3, k4) threshold pairs
│       ├── buildTrajFilename.m      ← Builds MAT filename for episode trajectory results
│       └── buildResultsPath.m       ← Builds full path for saving trained-agent MAT files
│
├── scripts/
│   ├── script_convergencePlot.m     ← Fig: SCA convergence (NPC, UCSR, DCSR, CRLB vs iter)
│   ├── script_cnrSweepPlot.m        ← Fig: NPC & power vs clutter-to-noise ratio (CNR)
│   ├── script_benchmarkComparison.m ← Fig: Proposed vs Without-AN (convergence comparison)
│   └── script_utilityEpisodePlot.m  ← Fig: UAV utility U2 vs training episode (RL results)
│
└── data/
    └── .gitkeep                     ← Place myGroundTerminalDist.mat and result MAT files here
```

---

## Dependencies

| Dependency | Version | Purpose |
|------------|---------|---------|
| MATLAB | ≥ R2022b | Core language |
| [CVX](http://cvxr.com/cvx/) | ≥ 2.2 | Convex optimisation (SDP solver) |
| Phased Array System Toolbox | bundled with MATLAB | `phased.URA`, `phased.SteeringVector` |
| Statistics and Machine Learning Toolbox | bundled | `randn`, distribution utilities |
| Reinforcement Learning Toolbox *(optional)* | bundled | Required only for `script_utilityEpisodePlot.m` |

---

## Getting Started

### 1. Clone the repository

```bash
git clone https://github.com/<your-username>/SecISAC-Mamaghani.git
cd SecISAC-Mamaghani
```

### 2. Install CVX

Download from [http://cvxr.com/cvx/download/](http://cvxr.com/cvx/download/) and follow the installation guide. Then, inside MATLAB:

```matlab
cd /path/to/cvx
cvx_setup
```

### 3. Add repository paths

Open MATLAB, navigate to the repository root, and run:

```matlab
setup          % Adds all src/ subdirectories to the MATLAB path
```

### 4. Provide data files

Place the following pre-generated MAT files in the `data/` folder:

| File | Contents |
|------|----------|
| `myGroundTerminalDist.mat` | `groundTerminalsLoc` cell: `{RBS_loc, ul_users_loc, dl_users_loc}` |
| `myBuffer.mat` *(optional)* | `buffer` variable for RL training replay |
| `visitedPositionsPerEpisode_*.mat` *(optional)* | Episode trajectory results from RL training |

> **Note:** The `.mat` data files are not included in the repository due to size constraints. Contact the authors or regenerate them using the channel simulation functions.

### 5. Run a simulation

```matlab
% Example: reproduce SCA convergence figure
run('scripts/script_convergencePlot.m')
```

---

## Key Scripts

### `script_convergencePlot.m`
Runs the SCA loop for **Proposed** (with AN) and **Without AN** benchmarks and plots:
- NPC convergence (log-scale)
- UCSR, DCSR, CRLB vs iteration (with threshold lines)

### `script_cnrSweepPlot.m`
Sweeps the Clutter-to-Noise Ratio (CNR) from −10 dB to +3 dB and plots the converged NPC and individual power components (DL, AN/radar, UL).

### `script_benchmarkComparison.m`
Side-by-side comparison of both benchmarks for a fixed channel snapshot with up to `maxIter` SCA iterations.

### `script_utilityEpisodePlot.m`
Plots the smoothed episode utility U2 curves for multiple λ values, comparing the proposed UAV strategy against the baseline.

---

## System Model Summary

```
         ┌──────────────────────────────────────────────┐
         │          Radar Base Station (R-BS)            │
         │   Mt = 25 Tx antennas  |  Mr = 25 Rx ant.    │
         │   f_c = 10 GHz         |  B  = 30 MHz         │
         └──────┬────────┬────────────────────┬──────────┘
                │ DL V_k │ AN W               │ Sensing A_0
         ┌──────┘        └──────┐             │
         ▼                      ▼             ▼
   K=10 DL Users           Eavesdropper   UAV Target
                            (Eve)          (Player 2)
         ▲
         │ P_l (UL)
   L=5 UL Users
```

**Optimisation variables:** `{V_k}` (DL beamforming), `W` (AN / radar covariance), `P` (UL power)  
**Objective:** Minimise NPC = Tr(ΣV_k) + Tr(W) + Σp_l  
**Constraints:**
- DCSR ≥ ρ_dl (downlink secrecy rate)
- UCSR ≥ ρ_ul (uplink secrecy rate)
- CRLB ≤ ρ_est (radar estimation quality)

---

## Configuration

All system parameters are centralised in `src/config/sysParams.m`. Key parameters:

| Parameter | Symbol | Default | Description |
|-----------|--------|---------|-------------|
| `L` | L | 5 | Number of uplink users |
| `K` | K | 10 | Number of downlink users |
| `Mt` | Mt | 25 | Transmit antennas (5×5 UPA) |
| `Mr` | Mr | 25 | Receive antennas (5×5 UPA) |
| `fc` | f_c | 10 GHz | Carrier frequency |
| `B` | B | 30 MHz | Signal bandwidth |
| `rho_ul` | ρ_ul | 0.1 | Uplink secrecy rate threshold |
| `rho_dl` | ρ_dl | 0.5 | Downlink secrecy rate threshold |
| `rho_est` | ρ_est | 0.001 | CRLB sensing threshold |
| `epsilon` | ε | 1e-3 | SCA convergence tolerance |
| `Pmax` | P_max | 100 W | Maximum uplink user power |

---

## Reproducing Paper Results

| Figure | Script |
|--------|--------|
| Convergence (NPC, UCSR, DCSR, CRLB vs iteration) | `script_convergencePlot.m` |
| NPC vs CNR sweep | `script_cnrSweepPlot.m` |
| Proposed vs No-AN benchmark | `script_benchmarkComparison.m` |
| UAV utility U2 vs training episode | `script_utilityEpisodePlot.m` |

---

## Citation

If you use this code in your research, please cite:

```bibtex
@ARTICLE{11172669,
  author    = {Mamaghani, Milad Tatar and Zhou, Xiangyun and Yang, Nan and Lee Swindlehurst, A.},
  journal   = {IEEE Journal on Selected Areas in Communications},
  title     = {Securing Integrated Sensing and Communication Against a Mobile Adversary:
               A Stackelberg Game With Deep Reinforcement Learning},
  year      = {2026},
  volume    = {44},
  pages     = {942--958},
  doi       = {10.1109/JSAC.2025.3611404}
}
```

Or use the `CITATION.cff` file (GitHub *Cite this repository* button).

---

## License

This project is released under the [MIT License](LICENSE). You are free to use, modify, and distribute this code for academic and non-commercial purposes, with appropriate attribution.

---

## Contact

For questions, bugs, or collaboration inquiries, please open a [GitHub Issue](../../issues) or contact the corresponding author via the paper's contact information.
