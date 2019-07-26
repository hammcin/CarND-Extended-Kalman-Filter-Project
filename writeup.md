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

After initializing the state and covariance matrices with the first measurement, my Kalman Filter then predicts the state and covariance matrices from their previous values on subsequent iterations (FusionEKF.cpp, FusionEKF::ProcessMeasurement, lines 113-148).  Prediction is then followed by a measurement update step (FusionEKF.cpp, FusionEKF::ProcessMeasurement, lines 150-177).

To perform the prediction step, it is necessary to update the state transition and process noise covariance matrices according to the time elapsed between each iteration (FusionEKF.cpp, FusionEKF::ProcessMeasurement, lines 126-146).  The updated state transition and process noise covariance matrices can then be used to predict the new state and covariance matrices (kalman_filter.cpp, KalmanFilter::Predict, line 29-40).

#### 4. Your Kalman Filter can handle radar and lidar measurements.

The measurement update step must be handled differently for radar (kalman_filter.cpp, KalmanFilter::Update, lines 42-63) and laser measurements (kalman_filter.cpp, KalmanFilter::UpdateEKF, lines 65-116).  Because the measurement function for radar measurements is nonlinear, it is necessary to linearize the radar measurement function by calculating its Jacobian matrix (tools.cpp, Tools::CalculateJacobian, lines 63-106).  A potential problem with calculating the Jacobian matrix is division by zero.  This situation is handled by dividing by a small number (0.001) instead (tools.cpp, Tools::CalculateJacobian, lines 77-84).  This situation also arises when converting from cartesian to polar coordinates, as is necessary when performing the radar measurement update (kalman_filter.cpp, KalmanFilter::UpdateEKF, lines 76-89).  Additionally, it is necessary to normalize the polar coordinate phi in the state vector for the radar measurement update to have a value less than or equal to pi and greater than or equal to -pi (kalman_filter.cpp, KalmanFilter::UpdateEKF, lines 91-100).

### Code Efficiency

#### 1. Your algorithm should avoid unnecessary calculations.
