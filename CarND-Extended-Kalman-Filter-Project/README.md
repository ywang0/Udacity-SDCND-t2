## CarND-Extended-Kalman-Filter-Project
Use Extended Kalman Filter to estimate the state of a moving object of interest with noisy lidar and radar measurements

---

### Refer to [project starter code](https://github.com/udacity/CarND-Extended-Kalman-Filter-Project) for
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
Use both laser and radar sensors readings
```console
$ ./ExtendedKF
```
Use only laser sensor readings
```console
$ ./ExtendedKF L
```
Use only radar sensor readings
```console
$ ./ExtendedKF R
```

For consecutive runs, remember to exit (`ctrl-c`) the program to refresh RMSE values, and click `Restart` button on the simulator.

### How does fusing the two sensors' data improve the tracking results?

Below RMSE results were run with:

```
// state covariance matrix
P_ << 0.01, 0, 0, 0,
      0, 0.01, 0, 0,
      0, 0, 5, 0,
      0, 0, 0, 5;

// noise variances for process noise covariance matrix Q
noise_ax = 9;
noise_ay = 9;
```  

| Sensor type| laser  | radar  | laser&radar |
| ---------- | ------ | ------ | ----------- | 
| RMSE(px)   | 0.1226 | 0.1869 | 0.0959      |
| RMSE(py)   | 0.0980 | 0.2795 | 0.0849      | 
| RMSE(vx)   | 0.5632 | 0.5228 | 0.3881      |
| RMSE(vy)   | 0.4433 | 0.6599 | 0.4289      |

In general, using only laser sensor readings provides more accurate estimations than using only radar sensor readings. Fusing the two sensors' readings has much improved the accuracies on position estimations and velocity estimations.
