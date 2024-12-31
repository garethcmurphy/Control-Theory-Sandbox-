# Control Theory Sandbox 🎛️⚙️  

**A Python sandbox for testing control theory concepts and demonstrating feedback loop systems. Ideal for learning, experimentation, and visualization of control strategies.**

---

## Overview 🌟  

This sandbox provides tools for exploring fundamental control theory principles, such as:  
- Feedback loops  
- Stability analysis  
- PID controllers  
- Open-loop and closed-loop systems  

The project includes interactive scripts and visualizations to help users understand and analyze system behavior.

---

## Features ✨  

- 🧪 Experiment with control strategies and tune parameters interactively.  
- 📈 Visualize system responses with time-series plots, Bode plots, and root locus diagrams.  
- 🎛️ Explore PID controllers and tune proportional, integral, and derivative gains.  
- 🔄 Simulate open-loop and closed-loop feedback systems.  
- 🔧 Define custom dynamic models or use prebuilt demos for rapid experimentation.  

---

## Getting Started 🚀  

### Prerequisites 🛠️  

- Python 3.8+  
- Required Python libraries: `numpy`, `scipy`, and `matplotlib`.  

Install dependencies:  
`pip install numpy scipy matplotlib`

---

### Installation  

1. Clone the repository:  
`git clone https://github.com/your-username/control-theory-sandbox.git`  
`cd control-theory-sandbox`

2. Run a demo script:  
`python feedback_demo.py`

---

## Usage 🔧  

### Predefined Models  

- Explore included demos like `pid_controller_demo.py` or `root_locus_demo.py`.  
- Adjust parameters directly in the scripts to see the impact on system behavior.

### Custom Models  

Define your own transfer functions or dynamic systems:  
```python
from scipy import signal
my_system = signal.TransferFunction([1], [1, 2, 1])
