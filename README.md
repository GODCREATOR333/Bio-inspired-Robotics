# Bio-Inspired Navigation Simulation 🐜 🤖

### *Simulating Cataglyphis behavior for drift mitigation in infrastructure-free environments*

![Status](https://img.shields.io/badge/Status-Research%20Prototype-blue) ![Python](https://img.shields.io/badge/Python-3.8%2B-green) ![License](https://img.shields.io/badge/License-MIT-lightgrey)

## 📖 Overview
This project is a **simulation framework** developed to explore bio-inspired navigation strategies for mobile robots (AGVs) operating in featureless environments, such as large warehouses or desert terrains.

Inspired by the desert ant *Cataglyphis*, this framework models how **intermittent global orientation cues** (e.g., Sun Compass / Ceiling Geometry) can bound the unbounded drift inherent in low-cost dead-reckoning systems.

**Why is this useful?**
*   **For Robotics:** It demonstrates how to navigate without expensive infrastructure (QR code grids) or heavy computation (Visual SLAM).
*   **For Research:** It serves as a lightweight, stochastic environment for training **Reinforcement Learning (RL)** agents to optimize the trade-off between energy (stopping to scan) and accuracy (localization).


![Simulation Interface](Report/images/demo2.png)

---

## 📹 Video Demonstrations

| Mode | Description | Link |
|------|-------------|------|
| **Mode 1** | Blind Navigation (Estimated Position) | [Watch](https://youtu.be/hDBotoQ3_pA) |
| **Mode 2** | Control Baseline (True Position) | [Watch](https://youtu.be/sY_AZQPQl3A) |
| **Mode 3** | Path Integration + Sun Compass | [Watch](https://youtu.be/nh_rpq5Yy5c) |

---

## 🚀 Quick Start

```bash
git clone https://github.com/GODCREATOR333/Bio-inspired-Robotics.git
cd Bio-inspired-Robotics
pip install -r requirements.txt
python main.py
```

## 📊 Results Summary

| Mode | Strategy | Success Rate | Terminal Error |
|------|----------|:------------:|:--------------:|
| Blind (Est) | Path Integration Only | 10% | 52.00 mm |
| Control | Ground Truth | 100% | 2.96 mm |
| Sun Compass | PI + Scan (100mm) | **90%** | **9.72 mm** |

**Key insight:** Intermittent global corrections reduce drift by ~90% without infrastructure.


### Controls
*   **Experiment Selection:** Use the dropdown in the sidebar to switch between `Blind`, `Control`, and `Sun Compass` modes.
*   **Parameters:** Adjust noise levels and speed using the sidebar sliders before starting.

| Key / Mouse | Action |
| :--- | :--- |
| **W / A / S / D** | Pan Camera |
| **Left Mouse** | Orbit / Rotate View |
| **Scroll** | Zoom In / Out |
| **R** | Reset Simulation & View |

---

## 🧩 Project Context
This work was developed as part of the selection task for the **FedEx SMART Project** at **IIT Madras** (Dept. of Applied Mechanics and Biomedical Engineering).

**Objective:** To codify biological navigation behaviors observed in *Cataglyphis* ants and evaluate their applicability to autonomous warehouse logistics.

---

## 📄 License
This project is licensed under the MIT License - see the [LICENSE](LICENSE) file for details.






