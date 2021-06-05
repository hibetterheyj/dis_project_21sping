# formation_graph_crossing_controller

```c
// 10-robot crossing
// first five robots from -0.1 -> -2.9 = -2.8
// last five robots from -2.9 -> -0.1 = 2.8
float goal_distance[2] = {2.8, 0.0};

// 10-robot crossing
// initial position of 10 robots
float init_x[10] = {-0.1, -0.1, -0.1, -0.1, -0.1, -2.9, -2.9, -2.9, -2.9, -2.9};
float init_y[10] = {0.0,  -0.1,   0.1, -0.2,  0.2,   0.0,   0.1, -0.1, 0.2, -0.2};

// bias vector of each 5-robot group
float bias_x[5] = {0, 0, 0, 0, 0};
float bias_y[5] = {0.0, 0.1, -0.1, 0.2, -0.2};

// initial_pos
		// 10-robot crossing
		if (robot_id < 5){
		    // goal distance [-2.8, 0]
		    goal_pos[0] = my_position[0] - goal_distance[0];
		    goal_pos[1] = my_position[1] - goal_distance[1];
		} else {
		    // goal distance [2.8, 0]
		    goal_pos[0] = my_position[0] + goal_distance[0];
		    goal_pos[1] = my_position[1] + goal_distance[1];
		}
		// 10-robot crossing

```

- small bugs in original test_crossing.wbt

    `epuck8 robot` not in [-2.9, 0.2] correctly!
