# Vehicle Power Management Simulation Report

**Modeling and Analysis of Power Consumption in Electric Vehicle Systems with Uncertain Road Conditions**

By Josué Dazogbo, Computer Engineering Student at the University of Ottawa  
Date: 10 April 2025

## Overview

This repository contains a simulation-based study of vehicle power management systems, developed using a combination of CarSim for high-fidelity vehicle dynamics simulation and MATLAB/Simulink's Vehicle Dynamics Blockset. The focus is on modeling the electro-mechanical dynamics of Electric Vehicles (EVs) to develop adaptive controllers capable of leveraging existing knoweledge of the environment to optimise the system's energy expenditure.

<div align="center">
  <img src="Ressources/Images/BlockDiagram.PNG" alt="Vehicle Block Diagram" style="width:70%;" />
  <p><em>Figure 1: Block diagram of the vehicle simulation environment in simulink</em></p>
</div>

<div align="center">
  <img src="Ressources/Images/DriveCycle.png" alt="Drive Cycle Profile" style="width:70%;" />
  <p><em>Figure 2: Example Drive cycle used to assess the performance of a controller</em></p>
</div>

<div align="center">
  <img src="Ressources/Images/MMAEWetTarmac.png" alt="Wet Tarmac Estimation" style="width:70%;" />
  <p><em>Figure 3: Acceleration profile of the vehicle and the corresponding acceleration estimations by road condition</em></p>
</div>

## Repository Structure

The [`EVLQR`](./EVLQR) directory contains all the materials related to the Linear Quadratic Regulator (LQR)-based energy management controller. Inside, you'll find scripts such as `LQRSolver.m` for solving the LQR control problem, as well as the main Simulink model [`VehicleEnergyManagementSystem.slx`](./EVLQR/VehicleEnergyManagementSystem.slx), which implements the electric vehicle control architecture.

The [`MMAE`](./MMAE) folder includes tools and files for Multi-Model Adaptive Estimator. Of particular importance is the Simulink toolbox [`MultiModelToolbox.slx`](./MMAE/MultiModelToolbox.slx), which enables dynamic model switching and parameter estimation within the simulation environment.

The [`Ressources`](./Ressources) folder contains supporting materials for the project. Within it, the [`Images`](./Ressources/Images) subfolder holds all figures and visual assets used in the report. The [`Derivations`](./Ressources/Derivations) subfolder includes the [Vehicle and Road Friction State Space Model Derivation](Ressources/Derivations/VehicleRoadStateSpace.pdf) upon which the control problem is vehicle dynamics based.
