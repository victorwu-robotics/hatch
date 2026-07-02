# Hatch (孵) 🐣

> 机器人的想法孵化器。  
> 一台机器人。一份 URDF。无轮询。

![Python](https://img.shields.io/badge/Python-3.11-blue?style=for-the-badge&logo=python&logoColor=white)
![Ubuntu](https://img.shields.io/badge/Ubuntu-20.04-orange?style=for-the-badge&logo=ubuntu&logoColor=white)
![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg?style=for-the-badge)
![Status](https://img.shields.io/badge/Status-Active-brightgreen?style=for-the-badge)

---

##  Hatch 是什么？

Hatch 是一个单进程、事件驱动的机器人臂应用开发平台。  
加载任意 URDF，实时求解逆运动学，并控制真实硬件——无需轮询循环、无需 ROS，并拥有直接 VTK 可视化。

**它有什么不同？**
- **一切皆事件** — 无轮询，无忙等待。
- **URDF 即场景** — 无独立世界文件，无启动文件。
- **架构是衍生出来的** — 每个组件都因某个需求而存在。

> *“一个替用户做决定的工具，并不智能——它是不服从的。”*  
> — 顽固的学生，[哲学](philosophy.md)

---

## 它看起来像什么

![主窗口，加载了 UR10](images/main_window_ur10_loaded.png)

---

## 快速开始（2 分钟）

```bash
git clone https://github.com/victorwu-robotics/hatch.git
cd hatch
python -m venv venv
source venv/bin/activate
pip install -r requirements.txt
python -m ui.main_window