# 🚀 Hybrid Autonomous Navigation Controller (Webots E-puck)

A robust vision-based autonomous navigation system for the **E-puck robot in Webots**, designed to achieve reliable goal-directed movement in dynamic environments while overcoming common failure modes like obstacle blindness, oscillation, and wall-lock.

---

## 🧠 Key Features

* **Adaptive Vision-Based Obstacle Detection**

  * Uses CIE-LAB color space + Mahalanobis distance
  * Real-time floor segmentation for accurate obstacle isolation

* **Hybrid State Machine Navigation**

  * `SEEK → AVOID_TURN → AVOID_DRIVE → ARRIVED`
  * Smooth goal tracking with heading correction

* **Intelligent Obstacle Avoidance**

  * Dynamic turn + forward stride strategy
  * Mid-stride obstacle detection (no blind spots)

* **Advanced Recovery Mechanisms**

  * ✅ Stride interrupt system (real-time reactivity)
  * ✅ Turn escalation for wide obstacles
  * ✅ Stall detection with forced escape
  * ✅ Oscillation prevention via cooldown logic

---

## ⚙️ Tech Stack

* **Python**
* **Webots Robotics Simulator**
* **OpenCV (Computer Vision)**
* **NumPy (Numerical Processing)**

---

## 🧩 Problem Solved

Traditional reactive navigation systems suffer from:

* ❌ Blind forward motion during avoidance
* ❌ Infinite oscillation near walls
* ❌ Sensor/model corruption from obstacles

This project **identifies root causes** and implements **system-level fixes**, resulting in stable and intelligent navigation.

---

## 📊 Core Innovations

* 🔍 **Floor Model Freezing** during avoidance (prevents data corruption)
* 🔁 **Interrupt-Driven Avoidance** instead of fixed motion
* 📈 **Escalating Turn Strategy** for complex obstacles
* 🧭 **Goal-Biased Decision Making** for efficient paths

---

## 🎯 Outcome

* Smooth and reliable navigation toward a target
* Strong resilience in cluttered and edge-case environments
* Eliminates oscillation and wall-lock scenarios

---

## 🖥️ Demo Highlights

* Real-time camera feed with obstacle overlay
* Floor segmentation visualization
* Debug insights for navigation states

---

## 📌 Future Improvements

* Path planning integration (A*, RRT)
* Multi-goal navigation
* Reinforcement learning for adaptive tuning

---

## 📎 How to Run

1. Open project in **Webots**
2. Attach controller to E-puck robot
3. Run simulation

---

## 👨‍💻 Author

Developed as part of advanced robotics and autonomous navigation exploration.
