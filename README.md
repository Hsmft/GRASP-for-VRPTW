# GRASP for the Vehicle Routing Problem with Time Windows (VRPTW)

![Language](https://img.shields.io/badge/language-C%2B%2B-blue.svg)
[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](https://opensource.org/licenses/MIT)

This repository contains a C++ implementation of the **GRASP (Greedy Randomized Adaptive Search Procedure)** metaheuristic to solve the **Vehicle Routing Problem with Time Windows (VRPTW)**. This project aims to find high-quality, near-optimal solutions by minimizing the number of vehicles and the total travel distance.

---

## 📋 Project Overview

The Vehicle Routing Problem with Time Windows (VRPTW) is a classic NP-hard optimization problem. This solver finds effective solutions using the GRASP methodology, which balances greedy construction with randomization to explore a diverse set of high-quality starting points for local optimization.

The solver implements the two main phases of GRASP:

1.  **Construction Phase:** A feasible solution is built iteratively. At each step, a **Restricted Candidate List (RCL)** is created containing the best candidates (e.g., the cheapest customer insertions). A candidate is then selected randomly from this list, introducing a strategic element of randomness to the greedy construction.
2.  **Local Search Phase:** Once a solution is constructed, a **Local Search** algorithm is applied to it. This phase uses neighborhood operators (like 2-Opt and Relocate) to improve the solution until a local optimum is reached.

This entire process is repeated for a set number of iterations, and the best solution found across all iterations is reported as the final result.

---

## 🛠️ Technologies Used

* **Language:** C++ (utilizing C++11 features like `<chrono>` and `<random>`)
* **Libraries:** C++ Standard Library only. No external optimization libraries were used.

---

## 🚀 How to Compile and Run

### Compilation
You can compile the source code using a standard C++ compiler like g++.

```bash
g++ -std=c++11 -o solver GRASP.cpp
```

### Execution
The program is run from the command line with the following arguments:

```bash
./solver [instance-file-path] [max-execution-time] [max-evaluations]
```
* `instance-file-path`: The path to the problem instance file (e.g., `instances/C101.txt`).
* `max-execution-time`: The maximum run time in seconds. Use `0` for no time limit.
* `max-evaluations`: The maximum number of objective function evaluations. Use `0` for no limit.

**Example:**
```bash
./solver instances/C101.txt 60 0
```
This command runs the solver on the `C101.txt` instance for a maximum of 60 seconds.

---

## 📄 License
This project is licensed under the MIT License.
