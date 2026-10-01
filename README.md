# Dynamic Programming for Control Co-Design

Dynamic-programming optimal control applied to **control co-design (CCD)** — jointly considering plant and controller — on a single-link manipulator, with vectorized Simulink batch simulation and an HPC-ready workflow. This is the computational backbone of my research on dynamic programming and model predictive control.

## Overview

Control co-design optimizes the physical system and its controller together rather than in sequence. Here, DP provides the globally optimal control benchmark against which co-design solutions are evaluated. The code is written for **batch simulation** — sweeping many configurations efficiently — and to scale onto high-performance computing (HPC) resources.

## What's inside

| Component | Purpose |
|-----------|---------|
| `single-link-codesign/` | DP + control co-design optimization on the single-link manipulator |
| `single-link-batch/` | Vectorized batch-simulation variant of the single-link DP study |
| `hpc-massdamper/` | HPC-oriented, vectorized DP batch simulation on a mass–damper system |
| Simulink models (`.slx`) | Plant models driven by the DP-optimal control for validation |
| `simulink_state_update_fn.m`, `trace_state_update_fn.m` | State-update hooks linking the DP solver to Simulink |

## Research context

This is part of my research on **dynamic programming and model predictive control** for closed-loop control systems — benchmarking globally optimal DP solutions against predictive control in a co-design setting.

See also the companion repos **dynamic-programming-optimal-control** and **nonlinear-mpc-pendulum**.

## Requirements

MATLAB with **Simulink**, plus the open-source **yadpf** framework on your path (see Attribution). HPC batch runs assume a standard MATLAB parallel/cluster setup.

## Attribution

DP solver utilities are from **yadpf** by Auralius Manurung ([GitHub](https://github.com/auralius/yadpf)), used under its original license. Co-design formulation, batch/HPC workflow, and analysis are my own.

## Author

**Oluwaseun A. Adekoya** — Robotics Engineer & PhD Candidate, University of Cincinnati. License: MIT (my code).
