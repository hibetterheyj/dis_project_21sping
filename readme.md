# DIS project code

## Teamwork

* Group ID: 13

- Localization: Nicola Santacroce (286331)
- High-level control
  - Flocking: Qi Yan (307457)
  - Formation: Yujie He (321657)
  - PSO: Jianhao Zheng (323146)

## Getting started

* `gsl` library is required for matrix computation. To install at Ubuntu:

  ```bash
  sudo apt-get install libgsl-dev
  ```

* Code structure

  ```bash
  ├── controller_world						# Main code
  │   ├── controllers							# Controllers
  │   │   ├── flocking_crossing_controller	
  │   │   ├── flocking_obstacle_controller
  │   │   └── supervisor						
  │   └── worlds								# Default testing world
  ├── localization_library					# Shared localization utilities
  ├── Matlab									# Visualization utilities
  └── supplemental							# Supplemental code, *not* necessary for metrics evaluation
      ├── controllers
      │   ├── crossing_pso_controller			# PSO
      │   ├── obstacle_pso_controller			# PSO
      │   ├── pso_crossing_supervisor			# PSO
      │   ├── pso_obstacle_supervisor			# PSO
      │   ├── localization_controller			# Localization
      │   └── localization_supervisor			# Localization
      └── worlds								# Ad-hoc worlds
  ```

## Results

* Qualitative results:
