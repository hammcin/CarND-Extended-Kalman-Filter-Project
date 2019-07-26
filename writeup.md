# Extended Kalman Filter Project

In this project you will utilize a kalman filter to estimate the state of a moving object of interest with noisy lidar and radar measurements. Passing the project requires obtaining RMSE values that are lower than the tolerance outlined in the project rubric.

### Accuracy

#### 1. px, py, vx, vy output coordinates must have an RMSE <= [.11, .11, 0.52, 0.52] when using the file: "obj_pose-laser-radar-synthetic-input.txt" which is the same data file the simulator uses for Dataset 1.

The accuracy of the Kalman Filter predictions is assessed using mean squared error as a metric (tools.cpp, Tools::CalculateRMSE, lines 17-61).  The Kalman Filter predictions are compared to the ground truth position and velocity of the moving object of interest to calculate the mean squared error.

Here's a [link to my video result](./ekf_video.mp4).

### Follows the Correct Algorithm

#### 1. Your Sensor Fusion algorithm follows the general processing flow as taught in the preceding lessons.

#### 2. Your Kalman Filter algorithm handles the first measurements appropriately.



#### 3. Your Kalman Filter algorithm first predicts then updates.



#### 4. Your Kalman Filter can handle radar and lidar measurements.



### Code Efficiency

#### 1. Your algorithm should avoid unnecessary calculations.
