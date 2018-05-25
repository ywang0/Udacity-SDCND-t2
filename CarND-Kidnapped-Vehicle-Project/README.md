## Vehicle Localization with Particle Filters
Use Particle Filter to localize a vehicle which has been placed in an unknown location.

The landmarks' locations in map coordinate frame are in `data/map_data.txt`, and a set of observed landmarks' locations in particle coordinate frame is provided by the simulator in each time step.

The Particle Filter has `num_particles` variable specified in `particle_filter.cpp`. Using more particles yields more accurate state estimations, but the computational time also increases. Setting the number to be 5 is good enough to pass the evaluation.

In each time step,  
the filter perform the following to each particle:
1. Use motion model with Gaussian distributed noises to predict particle's next state.
2. Calculate particle's posterior (i.e., how likely the particle's state is the actual vehicle's) given the observed landmarks' locations. We use Multivariate Gaussian distribution to calculate the posterior, which will be used in weighted resampling process.  

the filter then resamples the new `num_particles` particles based on the newly calculated particle weights.


---

### Refer to [project starter code](https://github.com/udacity/CarND-Kidnapped-Vehicle-Project) for
1. Environment set up
2. [uWebSocketIO](https://github.com/uNetworking/uWebSockets) installation


### Run the code
From the project top directory,

```console
$ ./clean.sh
$ ./build.sh
$ ./run.sh
```

### Visualize

In simulator, the blue circle represents the most probable particle whose state matches the vehicle. If the transition calculation is done correctly, the vehicle should sit at the center of the circle all the time.

[//]: # (Image References)
[image1]: ./simulator.png "simulator screenshot"
![alt text][image1]
