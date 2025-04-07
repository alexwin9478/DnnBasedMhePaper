# Deep Neural Network-Based MPC and MHE in MiL Simulation Using acados

## Overview
This repository contains the simulation framework for a Deep Neural Network (DNN)-based Model Predictive Control (MPC) and Moving Horizon Estimation (MHE) system, integrated with acados for Model-in-the-Loop (MiL) simulations. The following guide outlines the folder structure and the steps required to set up and execute the simulation.

---

## Getting Started
Follow these steps to set up and run the MiL simulation.

### **Step 1: Install acados**
After cloning this repository, ensure that **acados** is installed in a directory parallel to the `DnnBasedMpcMhePmsm` folder. This is required for proper compilation and execution of the MPC and MHE modules.

### **Step 2: Set Up the Simulation Environment**
Run the script [`setpath.m`](https://git-ce.rwth-aachen.de/alexander.winkler/dnnbasedmpcmhepmsm/-/blob/main/setpath.m?ref_type=heads) from the root folder. This script configures the necessary folder paths and dependencies required for the simulation.

### **Step 3: Run the Simulation**
Execute the main simulation script:

- **Run [`MAIN_SIM.m`](https://git-ce.rwth-aachen.de/alexander.winkler/dnnbasedmpcmhepmsm/-/blob/main/a_Scripts/MAIN_SIM.m?ref_type=heads)** – This file initializes environment variables and compiles two separate instances of acados:
  - [`Generate_MPC.m`](https://git-ce.rwth-aachen.de/alexander.winkler/dnnbasedmpcmhepmsm/-/blob/main/a_Scripts/Generate_MPC.m?ref_type=heads) for MPC generation.
  - [`Generate_MHE.m`](https://git-ce.rwth-aachen.de/alexander.winkler/dnnbasedmpcmhepmsm/-/blob/main/a_Scripts/Generate_MHE.m?ref_type=heads) for MHE generation.

- **Run [`start_sim_offline.m`](https://git-ce.rwth-aachen.de/alexander.winkler/dnnbasedmpcmhepmsm/-/blob/main/a_Scripts/start_sim_offline.m?ref_type=heads)** – This script sets up and executes the simulation using the Simulink model [`Offline_MiL_Sim.slx`](https://git-ce.rwth-aachen.de/alexander.winkler/dnnbasedmpcmhepmsm/-/blob/main/e_Sim/Offline_MiL_Sim.slx).

---

## Folder Structure
The repository is structured as follows:

- [**`a_Scripts/`**](https://git-ce.rwth-aachen.de/alexander.winkler/dnnbasedmpcmhepmsm/-/tree/main/a_Scripts?ref_type=heads) – Contains core scripts for initializing and running the simulation.
- [**`b_Models/`**](https://git-ce.rwth-aachen.de/alexander.winkler/dnnbasedmpcmhepmsm/-/tree/main/b_model?ref_type=heads) – Houses the model definitions used in MPC and MHE.
- [**`c_MPC/`**](https://git-ce.rwth-aachen.de/alexander.winkler/dnnbasedmpcmhepmsm/-/tree/main/c_MPC?ref_type=heads) – Contains files related to the Model Predictive Controller (MPC).
- [**`d_MHE/`**](https://git-ce.rwth-aachen.de/alexander.winkler/dnnbasedmpcmhepmsm/-/tree/main/d_MHE?ref_type=heads) – Contains files for the Moving Horizon Estimation (MHE) module.
- [**`e_Sim/`**](https://git-ce.rwth-aachen.de/alexander.winkler/dnnbasedmpcmhepmsm/-/tree/main/e_Sim?ref_type=heads) – Includes Simulink models for MiL simulation.

All essential initialization scripts, parameter definitions, and model functions for both MPC and MHE are organized within their respective folders.

---

## Viewing Simulation Results

- **Simulation Performance Metrics:** The simulation prints real-time results in the MATLAB command window.
- **Visualization:** The main Simulink scope in [`Offline_MiL_Sim.slx`](https://git-ce.rwth-aachen.de/alexander.winkler/dnnbasedmpcmhepmsm/-/blob/main/e_Sim/Offline_MiL_Sim.slx) provides graphical visualization of the system states and control actions.
- **Data Logging:** Simulation data is stored in the MATLAB workspace for further analysis.

---

## Additional Notes
- Ensure acados and casadi are installed and configured correctly.
- If acados is not installed, refer to the [official installation guide](https://github.com/acados/acados) for setup instructions.
- There are multiple options available for the acados OCP and solver settings, feel free to play around. be careful when transferring t´his code to different projects with different specs and dynamics. Other options might be better suited.
- If any dependencies are missing, update your MATLAB path using [`setpath.m`](https://git-ce.rwth-aachen.de/alexander.winkler/dnnbasedmpcmhepmsm/-/blob/main/setpath.m?ref_type=heads).

## Authors
- **Alexander Winkler** (winkler_a@mmp.rwth-aachen.de)
- **Pranav Shah** (shah@mmp.rwth-aachen.de)

For any issues or contributions, feel free to raise an issue or submit a pull request.
Happy coding! 🚀