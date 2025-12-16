# Bio-inspired-Robotics

A simulation framework modeling Cataglyphis navigation to mitigate odometry drift in featureless environments using intermittent spectral cues.

# Bio-Inspired Navigation Simulation 🐜 🤖

### *Simulating Cataglyphis behavior for drift mitigation in infrastructure-free environments*

![Status](https://img.shields.io/badge/Status-Research%20Prototype-blue) ![Python](https://img.shields.io/badge/Python-3.8%2B-green) ![License](https://img.shields.io/badge/License-MIT-lightgrey)

## 📖 Overview
This project is a **simulation testbed** developed to explore bio-inspired navigation strategies for mobile robots (AGVs) operating in featureless environments, such as large warehouses or desert terrains.

Inspired by the desert ant *Cataglyphis*, this framework models how **intermittent global orientation cues** (e.g., Sun Compass / Ceiling Geometry) can bound the unbounded drift inherent in low-cost dead-reckoning systems.

**Why is this useful?**
*   **For Robotics:** It demonstrates how to navigate without expensive infrastructure (QR code grids) or heavy computation (Visual SLAM).
*   **For Research:** It serves as a lightweight, stochastic environment for training **Reinforcement Learning (RL)** agents to optimize the trade-off between energy (stopping to scan) and accuracy (localization).

---

## 📸 Visual Demonstration

### The Problem: Unbounded Drift (Blind Navigation)
*The agent relies solely on noisy odometry. Heading error accumulates, causing the agent to spiral away from the target.*

![Blind Mode GIF](INSERT_LINK_TO_BLIND_GIF_HERE)

### The Solution: Spectral Compass (Sun Scan)
*The agent performs intermittent "Stop-and-Scan" maneuvers to recalibrate its heading. Drift is bounded, ensuring successful homing.*

![Sun Mode GIF](INSERT_LINK_TO_SUN_GIF_HERE)

*(Note: Replace the links above with your actual GIF or YouTube links)*

---

## 📊 Results (TL;DR)

We conducted A/B testing across three navigation modes to quantify the impact of biological behaviors on homing success.

| Experiment Mode | Condition | Behavior | Result |
| :--- | :--- | :--- | :--- |
| **1. Blind Navigation** | Short Range (<1m) | Path Integration Only | **✅ SUCCESS** (Drift < Threshold) |
| **2. Blind Navigation** | Long Range (>5m) | Path Integration Only | **❌ FAILED** (Drift > 500mm) |
| **3. Sun Compass** | Long Range (>5m) | PI + Intermittent Scans | **✅ SUCCESS** (Error < 20mm) |
| **4. Spiral Search** | Overshoot Case | Systematic Local Search | **✅ SUCCESS** (Target Acquired) |

> **Key Finding:** While dead reckoning is sufficient for short-range movements, long-distance returns require global orientation fixes. The "Stop-and-Scan" behavior reduces terminal position error by approximately **90%** in long-run trials.

---

## 🛠 Features
*   **Modular Architecture:** Decoupled Physics (Ground Truth) from Perception (Belief State).
*   **Realistic Noise Modeling:** Implements stochastic odometry with tunable **Stride Noise** (distance slip) and **Heading Drift** (bias + jitter).
*   **Finite State Machine (FSM):** Implements `SEARCH`, `RETURN`, `IDLE`, and `STOP` behaviors.
*   **Interactive Dashboard:** Real-time A/B testing with adjustable parameters (Noise levels, Speed, Scan Interval).
*   **Real-time Plotting:** Live visualization of Trajectory (True vs. Est) and Heading Error.

---

## 🚀 Installation & Usage

### Prerequisites
*   Python 3.8 or higher

### Installation
1.  **Clone the repository:**
    ```bash
    git clone https://github.com/GODCREATOR333/Bio-inspired-Robotics.git
    cd bio-inspired-Robotics
    ```

2.  **Install dependencies:**
    ```bash
    pip install -r requirements.txt
    ```

3.  **Run the simulation:**
    ```bash
    python main.py
    ```

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