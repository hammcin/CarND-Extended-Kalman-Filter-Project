# Extended Kalman Filter Project

In this project you will utilize a kalman filter to estimate the state of a moving object of interest with noisy lidar and radar measurements. Passing the project requires obtaining RMSE values that are lower than the tolerance outlined in the project rubric.

### Accuracy

#### 1. px, py, vx, vy output coordinates must have an RMSE <= [.11, .11, 0.52, 0.52] when using the file: "obj_pose-laser-radar-synthetic-input.txt" which is the same data file the simulator uses for Dataset 1.

The accuracy of the Kalman Filter predictions is assessed using mean squared error as a metric (tools.cpp, Tools::CalculateRMSE, lines 17-61).  The Kalman Filter predictions are compared to the ground truth position and velocity of the moving object of interest to calculate the mean squared error.

Here's a [link to my video result](./ekf_video.mp4).

### Follows the Correct Algorithm

#### 1. Your Sensor Fusion algorithm follows the general processing flow as taught in the preceding lessons.

#### 2. Your Kalman Filter algorithm handles the first measurements appropriately.

My Kalman Filter processes the first measurement observed in FusionEKF.cpp, FusionEKF::ProcessMeasurement, lines 58-111.  The first measurement is used to initialize the state vector for a given radar measurement (FusionEKF.cpp, FusionEKF::ProcessMeasurement, lines 70-86) and for a given laser measurement (FusionEKF.cpp, FusionEKF::ProcessMeasurement, lines 87-98).  The state covariance matrix is also initialized at this time (FusionEKF.cpp, FusionEKF::ProcessMeasurement, lines 100-105).  Radar measurements must be converted from polar to cartesian coordinates before they are used to initialize the state vector (FusionEKF.cpp, FusionEKF::ProcessMeasurement, lines 77-78).

#### 3. Your Kalman Filter algorithm first predicts then updates.



#### 4. Your Kalman Filter can handle radar and lidar measurements.



### Code Efficiency

#### 1. Your algorithm should avoid unnecessary calculations.
