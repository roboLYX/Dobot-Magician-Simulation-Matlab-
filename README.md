# Dobot-Magician-Simulation-Matlab-
This repository provides a complete MATLAB-based simulation framework for the Dobot Magician 3R robotic arm, focusing on forward/ inverse kinematics, trajectory generation, and sim-to-real analysis.
📌 README.md（中英双语）

⸻

Dobot-Magician-Simulation-MATLAB

A complete MATLAB simulation framework for the Dobot Magician 3R robotic arm
一个完整的 Dobot Magician 3R 机械臂 MATLAB 仿真框架

⸻

⭐ Overview｜项目概述

This repository provides a complete MATLAB-based simulation framework for the Dobot Magician 3R robotic arm, including:
	•	Forward kinematics (FK)
	•	Inverse kinematics (IK) — both numerical IK and analytical IK
	•	Trajectory generation (jtraj in joint space, ctraj in Cartesian space)
	•	Error comparison
	•	Visualization and animation
	•	Sim-to-Real analysis

本仓库提供一个完整的 Dobot Magician 3R 机械臂 MATLAB 仿真框架，包括：
	•	正运动学 FK
	•	逆运动学 IK（数值法 + 解析法）
	•	轨迹规划（关节空间 jtraj + 笛卡尔空间 ctraj）
	•	IK 精度对比
	•	仿真轨迹动画
	•	Sim-to-Real 误差分析

⸻

📁 Repository Structure｜仓库结构

Dobot-Magician-Simulation-MATLAB/
│
├── Dobot_Simulation_Chinese/    # 中文版仿真文档 & 代码
├── Dobot_Simulation_English/    # English simulation scripts
├── README.md                    # 双语描述文件

Each folder contains:
	•	Trajectory planning scripts
	•	Analytical IK implementation
	•	ikcon-based IK implementation
	•	jtraj vs ctraj trajectory comparison
	•	Error analysis and visualization

每个文件夹包含：
	•	轨迹规划脚本
	•	解析 IK 实现
	•	基于 ikcon 的逆运动学
	•	jtraj / ctraj 插值对比
	•	误差分析和可视化

⸻

🔧 Features｜功能特点

1. Forward & inverse kinematics
	•	Official DH FK (robot.fkine)
	•	Numerical IK (ikcon)
	•	Analytical IK (custom closed-form)

2. Trajectory generation
	•	jtraj → smooth joint trajectory
	•	ctraj → straight-line Cartesian path

3. Multiple IK–trajectory modes

Mode	IK	Trajectory	Description
Mode 1	Analytical IK	ctraj	Highest accuracy, direct path
Mode 2	Analytical IK	jtraj	Accurate + smooth
Mode 3	ikcon	jtraj	Smooth, less accurate
Mode 4	ikcon	ctraj	Straight-line but inaccurate


⸻

🔍 Why Numerical IK Fails?｜为什么数值 IK 不精准？
	•	ikcon is numerical → depends on initial guess
	•	Dobot official DH parameters inaccurate
	•	3R cannot satisfy 6D pose constraints
	•	Numerical IK + inaccurate DH → cannot pass four given points precisely

ikcon 属于迭代计算，需初值；
Dobot 官方 DH 不准确；
3R 无法满足 6 自由度约束；
因此在 4 个关键点上误差明显。

⸻

📐 Why Analytical IK Works?｜为什么解析 IK 更精准？
	•	Closed-form
	•	No iterations
	•	Zero IK–FK mismatch (when paired with the same FK model)
	•	Essential for high-precision trajectory planning

解析 IK 是闭式解，不需要迭代，
且与解析 FK 互逆 → 精度高，可做到零误差。

⸻

🎬 Trajectory Visualization｜轨迹可视化

The simulation includes:
	•	Cartesian path plots
	•	Joint trajectories
	•	Joint velocity curves
	•	Animation of robot movement

仿真包括：
	•	笛卡尔路径图
	•	关节角轨迹
	•	关节速度曲线
	•	机械臂运动动画

⸻

🧪 Sim-to-Real Discussion｜仿真到实机（Sim2Real）讨论

Practical robot (Dobot Magician) has:
	•	DH parameter errors
	•	Zero-position offset
	•	Joint backlash

So simulation ≠ real robot.

建议：
	1.	DH parameter calibration
	2.	Zero-position calibration
	3.	Use real TCP measurement to fit model
	4.	Optional: ML-based DH estimation

⸻

🚀 Future Work｜未来工作
	•	Derive true analytical IK based on official DH
	•	Add obstacle avoidance
	•	Add minimum-jerk trajectory
	•	Calibrate DH using real robot data

⸻

📦 License｜许可证

MIT License (自由使用)

⸻

🚀 Release Notes (v1.0)｜发布说明

Version 1.0 – Initial Release

Content included:
	•	Full MATLAB simulation framework
	•	Both analytical and numerical IK implementations
	•	Trajectory planning: jtraj & ctraj
	•	Cartesian / joint / animation visualization
	•	Chinese & English documentation folders
	•	Error comparison and Sim2Real analysis

适合用于：
	•	课程作业
	•	科研仿真
	•	Dobot Magician 教学
	•	轨迹规划算法对比
