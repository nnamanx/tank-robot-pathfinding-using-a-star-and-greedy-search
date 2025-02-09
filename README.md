# 🚀 Tank Robot Pathfinding Using A* and Greedy Best-First Search  

## 🤖 Project Overview  
This project showcases a **tank robot** that can autonomously navigate a **5x5 grid**, avoid obstacles, and determine the most **optimal path** using two different search algorithms:  

- **A* Algorithm** – Balances actual movement cost and heuristic estimates to find the shortest path.  
- **Greedy Best-First Search** – Focuses solely on the heuristic, making fast but sometimes suboptimal decisions.  

From just an **Arduino, a gyroscope, and a handful of wires**, we built this smart navigation system from scratch! 🚀  

---

## 📜 Features  
✔️ **Autonomous Pathfinding** – The tank finds its way from start to goal.  
✔️ **Obstacle Avoidance** – Detects and maneuvers around obstacles.  
✔️ **Algorithm Comparison** – Tests A* vs. Greedy Best-First Search for efficiency.  
✔️ **Arduino-Controlled** – Powered by an **Arduino Uno** and sensors.  

---

## 🛠️ Components Used  
| Component  | Purpose |
|------------|---------|
| **Arduino Uno**  | Main controller for the robot.  |
| **Gyroscope (IMU Sensor)**  | Tracks orientation and movement.  |
| **Motor Driver**  | Controls the tank wheels.  |
| **Ultrasonic Sensors**  | Detects obstacles in the path.  |
| **LiPo Battery**  | Powers the system.  |
| **Wheels & Chassis**  | Forms the base of the tank.  |

---

## 🔧 Setup & Installation  

### 1️⃣ Clone the Repository  
```bash
git clone https://github.com/nmananx/tank-robot-pathfinding-using-a-star-and-greedy-search.git
cd tank-robot-pathfinding-using-a-star-and-greedy-search

### 2️⃣ Upload Code to Arduino
Open a_star.ino in the Arduino IDE and upload to test A*.
Open greedy.ino to test Greedy Best-First Search.

### 3️⃣ Run the Robot 
Place the tank on a 5x5 grid with obstacles.
Run the code and watch it navigate!
