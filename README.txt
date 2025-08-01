PID Tuning for Buck Converter Using GA and PSO
================================================

This repository contains MATLAB scripts for demonstrating and comparing Genetic Algorithm (GA)
and Particle Swarm Optimization (PSO) for PID controller tuning of a buck converter.

Demo videos are available here:
https://drive.google.com/drive/folders/1gmMOhIkDKeodmUT6TvkEYC6hIP737xoM?usp=sharing

------------------------------------------------
Repository Main Contents
------------------------------------------------
1. run_ga_composite_verbose.m – Runs the GA-based PID tuning.
2. PSO.m – Runs the PSO-based PID tuning.
3. COMPARISON_MANUAL_GA_PSO_OVERLAP.m – Compares the best results from manual tunning, GA and PSO.

------------------------------------------------
How to Run the Codes
------------------------------------------------

1. Setup
--------
1. Download this repository as a ZIP file.
2. Extract it to your preferred working directory.
3. Open MATLAB and set the extracted folder as the current directory.

2. Running GA
--------------
1. Open run_ga_composite_verbose.m in MATLAB.
2. Click Run.
3. Wait for the optimization results to display.

3. Running PSO
---------------
1. Open PSO.m in MATLAB.
2. Click Run.
3. Wait for the optimization results to display.

4. Comparing Results
---------------------
1. Open COMPARISON_MANUAL_GA_PSO_OVERLAP.m in MATLAB.
2. Update the script with the best Kp, Ki, Kd values obtained from GA and PSO.
3. Click Run.
4. Observe the comparative performance plots.

------------------------------------------------
Notes
------------------------------------------------
- Ensure all scripts are in the same folder before running.
- Results may vary due to the stochastic nature of GA and PSO.