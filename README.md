# SoC_project

This is the repository connected to the final project of the course SoC design. All C-code for programming the SoC and other relevant files is to be uploaded here such that everyone has the opportunity to see the progress and coordinate the code.

## Code overview

| File | Owner | What it does |
|---|---|---|
| `model/Maxom_fcs_noncascaded.slx`, `model/PMSMpara.m` | Dayana | Simulink reference (controller + Simscape motor). Exports golden traces to `traces/*.csv`. |
| `hls/fcs_mpc.c`, `hls/fcs_mpc.h` | Anastasiia | FCS-MPC controller. Input: phase currents, rotor angle, speed, speed reference and motor parameters. Output: 3-bit switching state (Sa, Sb, Sc). The same source is synthesized with Vitis HLS (FPGA) and compiled with gcc for the ARM (CPU baseline). |
| `arm/motor.c`, `arm/motor.h` | Eskil | Discrete PMSM model. Input: 3-bit switching state and load torque. Output: phase currents, rotor angle and speed. Keeps one state per motor, so N motors can be simulated. |
| `vivado/block_design.tcl` | Even | Block design: Zynq PS, AXI DMA, controller IP and cycle counter. Produces the bitstream (`.bit`, `.hwh`). |
| `arm/closed_loop.c` | Even | Closed-loop driver on the board: steps the motor models, sends their states to the controller, receives the decisions and logs results to `results/`. |

## Maybe files?
| File | Owner | What it does |
|---|---|---|
| `arm/sil_main.c` | Eskil | Closed loop on the PC (motor model + float controller) without the board. Reference for the board results. |
| `scripts/compare.py` | Eskil | Compares results with the golden traces (RMS error, plots). |