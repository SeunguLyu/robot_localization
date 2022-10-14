# Robot Localization

## Run

- If running Neato in a simulation or real environment, run the code below:
```
ros2 launch robot_localization test_pf.py map_yaml:=path-to-your-yaml-file
```

- If you are testing particle filter with a bag file, run the code below:
```
ros2 launch robot_localization test_pf.py map_yaml:=path-to-your-yaml-file use_sim_time:=false
```

## Overview
Robot localization is one of the building blocks of achieving successful robot navigation (perception, localization, cognition, motor control); necessary for making decisions about its future actions. The objective of this project was to use a particle filter to determine where our Neato is located and how it is oriented with respect to its environment, which was a pre-built map of the first floor of the MAC in our case. The particle filter is first initialized with a random distribution of particles, where each particle is a representation of the Neato's potential location and orientation in the map. Once the initialization is done, the following steps are repeated:

1. Update the particles using data from odometry
2. Reweight the particles based on their compatibility with the laser scan
3. Update your estimate of the robot’s pose given the new particles. Update the map to Odom transform.
4. Resample with replacement a new set of particles with probability proportional to their weights.

Below is a demonstration of a simulation of the Neato determining its position and orientation in the MAC.

![](images/particle_filter_demo_1.gif)
![](images/particle_filter_demo_2.gif)

| Bag File | Based On | Demo |
| ------------- | ------------- | ----- |
| [macfirst_take_2](bags/macfirst_take_2) | [macfirst_floor_take_1](bags/macfirst_floor_take_1) | [particle_filter_demo_1](images/particle_filter_demo_1.gif)|
| [mac_take_2](bags/mac_take_2) | [macfirst_floor_take_2](bags/macfirst_floor_take_2) | [particle_filter_demo_2](images/particle_filter_demo_2.gif)|

## Initialize Particles

Before anything, we must initialize particles within the map. Each particle is a representation of the Neato's potential location and orientation on the map. Initialization creates N particles within the defined bounding area from the center given by the 2D Pose Estimate. An example of particle initialization with 14 particles and a bounding box unit of 4 is seen below:

![](images/particle_init.png)

## Update Particle Positions

Once the Neato starts moving around, we must also update the positions of the particles relative to the Neato's movement. To do so, we follow the steps below for every particle in the particle cloud:

1. Compute the change in x, y, and theta between the robot's current and previous state in the Odom frame
2. Compute the distance and angle of rotation towards the new state in the Odom frame.
3. Compute the change in x, y, theta (rotation 1) of particles in the map frame using the angle of rotation and distance calculated above.
4. Compute the final change in rotation (rotation2) so that particles are headed toward the correct direction.

A visualization of this process is shown in the picture below:

![](images/particle_update.png)

## Update Particle Weights

As the robot navigates around its environment, it receives laser scan data that consists of 1) distance readings to nearby obstacles and 2) angles relative to the robot frame for each corresponding distance reading. Using this obstacle data over 360 degrees, the weight of each particle in the particle cloud is recalculated. The greater the mean obstacle distance over 360 degrees of a particular particle, the less likely the particle is to be an accurate representation of the Neato in the map. The relationship between the particle weight and the mean obstacle distance reading is represented as an inversely proportional relationship. In case the obstacle location either returns to be a `NaN` or does not fall within the valid boundary of the map, we punish this behavior by assigning a high obstacle distance value, so that the particle weight will decrease. 

![](images/particle_weight_measure.png)

## Update Robot Position On Map

Once all the particles are re-weighted, the mean of all the particles' x, y, and theta relative to the map frame is calculated. This mean is set to be the next position of the Neato on the map. An example of this step can be seen below, where the robot position is updated based on the location and orientation of three particles.

![](images/robot_position.png)

## Resample Particles

Once the robot position on the map is updated, the weight values of each particle are normalized with standard normal distribution so that we can have better results during the resampling step. During this normalization step, we make sure that the weights sum up to 1.0 so that this can be used as an array of probability. For example, an array of weights will look like the below after the normalization step:
```
[0.1, 0.2, 0.1, 0.3, 0.3]
```
The higher the weight of the particle, the more likely the particle is to be a representation of the Neato in the map, and vice versa. Using this probability array, we draw a random sample of particles using `draw_random_sample()`, which most likely results in particles that are more likely to be accurate representations of the Neato in the map. After this step, we add a little bit of noise to the x, y, and theta of the particles. The distribution of particles before and after resampling is shown in the picture below.

![](images/resample_particle.png)

## Conclusion

We found 300 particles to be sufficient to determine the Neato's location in the provided map of the MAC first floor. 

### Challenges

The biggest challenge was figuring out `update_particles_with_odom()`. We had an issue implementing a transformation matrix to achieve this task, so we thought of an alternative way which was calculating the rotation and distance toward the new position. Another challenge was trying to figure out the way we normalized the weights, as the result of resampling depended a lot on how we defined the weights. Eventually, we found out that normalizing weights with standard normal distribution works well.

### Improvements

Our current implementation of `update_particles_with_odom()` is a rather crude approach that breaks down frame translation and rotation into smaller steps, resulting in a longer run-time. However, this can be optimized with a more elegant approach of using a combination of rotation matrixes and the Numpy library. This way, the frame translation, and rotation can be easily represented as matrix multiplications. 

There is a lot of room for improvement in how we calculate the weights and how we normalize them. Due to time constraints, we only had the chance to test only some ways of normalization but all of them showed clear performance improvement. Testing more options might result in better performance both in accuracy and time. 

Also, the way we add noise at this point is by adding random values to the x, y, and theta of particles based on their weights. There are many different options for adding noises that we have not explored.

### Lessons

One of the big takeaways from working on this project was to test every block (e.g. `update_particles_with_odom()`, `update_robot_pose()`, `resample_particles()`, `update_particles_with_laser()`, `initialize_particle_cloud()`) consisting robot localization. This meant after writing the functionality for each function, not only resolving errors but most importantly testing that the simulation in RViz exhibits the expected behavior. This helped us to guarantee which blocks of the project were working properly and reduced the time narrowing down the areas that would produce undesired behaviors. 