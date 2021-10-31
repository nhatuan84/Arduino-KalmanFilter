# Arduino-KalmanFilter
Arduino-KalmanFilter

Based on https://github.com/simondlevy/TinyEKF

This is a matrix version of Kalman Filter for ESP8266/ESP32/MCUs. 

It can be applied to Arduino, ESP8266/ESP32/MCUs.

There are 3 files that you need to include in your project (ESP8266/ESP32/MCUs): 

- ArduiKalman.cpp
- ArduiKalman.h
- mat.h

![alt text](https://github.com/nhatuan84/Arduino-KalmanFilter/blob/main/Kalman-filter-equations-and-instruction.png)

n: number of states

m: number of measurement values

xc[n]:   correct state vector 

xp[n]:   predict state vector 

A(n, n): System dynamics matrix

H(m, n): Measurement matrix

Q(n, n): Process noise covariance

R(m, m): Measurement noise covariance

P(n, n): Estimate error covariance

Predict step:

   float *predict = m_kf.predict()
   
Correct step:

   float *correct = m_kf.correct(measurement)

An example when n=4, m=2

  A[0][0] = 1.0f;
  
  A[1][1] = 1.0f;
  
  A[2][2] = 1.0f;
  
  A[3][3] = 1.0f;

  H[0][0] = 1.0f;
  
  H[1][1] = 1.0f;

  Q[0][0] = 0.01f;
  
  Q[1][1] = 0.01f;
  
  Q[2][2] = 0.01f;
  
  Q[3][3] = 0.01f;

  R[0][0] = 0.1f;
  
  R[1][1] = 0.1f;

  P[0][0] = 1.0f;
  
  P[1][1] = 1.0f;
  
  P[2][2] = 1.0f;
  
  P[3][3] = 1.0f;
  
  xc[0] = 0.01f;
  
  xc[1] = 0.01f;
  
  xc[2] = 0.01f;
  
  xc[3] = 0.01f;


![alt text](https://github.com/nhatuan84/Arduino-KalmanFilter/blob/main/kalman1.png)

