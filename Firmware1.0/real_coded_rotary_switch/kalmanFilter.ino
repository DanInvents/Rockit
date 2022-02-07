// This program performs a Kalman filter of the flight data. It smoothens the data and ignores transitory events.

// Q = process noise covariance
// R = measurement noise covariance. Larger R means large measurement uncertainty. Larger Q means larger estimation uncertainty. Thus increasing Q corrects more.
// Xpe0 = prior estimation of signal X at time t=0 (current state)
// Xe1 = estimation of X at time t=1 (previous state)
// Ppe0 = prior estimation of "error covariance" at t=0,  
// P1 = error covariance at t=1, P0 = error covariance at t=0
// K = Kalman gain, Xe0 = estimation of signal at t=0, Z = measured signal at t=0;

float kalmanFilter(float Z){
    Xpe0 = Xe1; // Assumption of prediction 1
    Ppe0 = P1 + Q; // Update of prior estimation of "error covariance"
    K = Ppe0/(Ppe0 + R); // Measurement update or correction of "Kalman gain"
    Xe0 = Xpe0 + K * (Z - Xpe0); // Measurement update or correction of "estimated signal"
    P0 = (1 - K) * Ppe0; // Measurement update or correction of "error covariance";
    Xe1 = Xe0;
    P1 = P0;
  return Xe0;
}
