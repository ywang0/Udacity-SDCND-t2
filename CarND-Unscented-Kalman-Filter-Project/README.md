## CarND-Unscented-Kalman-Filter-Project
Use Unscented Kalman Filter to estimate the state of a moving object of interest with noisy lidar and radar measurements

---

### Refer to [project starter code](https://github.com/udacity/CarND-Unscented-Kalman-Filter-Project) for
1. Environment set up
2. [uWebSocketIO](https://github.com/uNetworking/uWebSockets) installation


### Build
From the project top directory,

```console
$ mkdir build
$ cd build
$ cmake .. && make
```
### Run it
To visualize the performance, we need to save the state estimates, measurements, ground truth, NIS values and RMSE values to an output file.

Save output to the default location/file name `./data/obj_pose-laser-radar-ukf-output.txt`
```console
$ ./UnscentedKF
```
Save output to a different location and/or file name
```console
$ ./UnscentedKF <your-file-path>
```

For consecutive runs, remember to exit (`ctrl-c`) the program to refresh RMSE values, and click `Restart` button on the simulator.  

Check `UKF_visualization.ipynb` for the filter performance visualization.

### Compare performance: UKF with CTRV (Constant Turn Rate Velocity) model v.s. [EKF with constant velocity model](https://github.com/ywang0/Udacity-SDCND-t2/tree/master/CarND-Extended-Kalman-Filter-Project)

Below UKF RMSE results were using both laser/radar readings and run with:

```
// initial state covariance matrix
P_ << 0.01, 0, 0, 0, 0,
      0, 0.01, 0, 0, 0,
      0, 0, 3, 0, 0,
      0, 0, 0, 1, 0,
      0, 0, 0, 0, 1;

// process longitudinal acceleration noise std
std_a_ = 1;

// process yaw acceleration noise std
std_yawdd_ = 0.6;
```  

| | EKF(constant V) | UKF (CTRV) | Improvement |
| --- | --- | --- | --- |
| RMSE(px)| 0.0959 | 0.0664 | 31% |
| RMSE(py)| 0.0849 | 0.0809 | 5% |
| RMSE(vx)| 0.3881 | 0.2583 | 33% |
| RMSE(vy)| 0.4289 | 0.2207 | 49% |

The non-linear prediction model and non-linear measurement mapping used in UKF have much improved the tracking results of a moving object of interest.
