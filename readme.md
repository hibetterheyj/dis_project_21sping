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
  ├── controller_world                        # Main code
  │   ├── controllers                         # Controllers
  │   │   ├── flocking_crossing_controller	
  │   │   ├── flocking_obstacle_controller
  │   │   └── supervisor						
  │   └── worlds                              # Default testing world
  ├── localization_library                    # Shared localization utilities
  ├── Matlab                                  # Visualization utilities
  │   ├── localization                        # Localization result visualization
  │   └── metric_computation                  # Flocking result visualization
  └── supplemental                            # Supplemental code, *not* necessary for metrics evaluation
      ├── controllers
      │   ├── crossing_pso_controller         # PSO
      │   ├── obstacle_pso_controller         # PSO
      │   ├── pso_crossing_supervisor         # PSO
      │   ├── pso_obstacle_supervisor         # PSO
      │   ├── localization_controller         # Localization
      │   └── localization_supervisor         # Localization
      └── worlds                              # Ad-hoc worlds
  ```

### Modification of webots world
* `/controller_world/worlds/test_obstacles.wbt`: The supervisor name need to be modified to `super0`
* `/controller_world/worlds/test_crossing.wbt`: None

### Run supervisor
* set controller of each supervisor to be `supervisor` (same for both two scenarios)
* metric at each time step will be saved in `flocking_metrics0.csv` (the number depends on which group)
* copy the data and run `Matlab/metric_computation/cal_single_avg_metric.m` in MATLAB will compute the average metric along the duration time.

### Flocking controllers

* `/controller_world/worlds/test_obstacles.wbt`: set *all* robot controllers to be `flocking_obstacle_controller`. 
* `/controller_world/worlds/test_crossing.wbt`: set *all* robot controllers to be `flocking_crossing_controller`.


## Results

* Qualitative results:
