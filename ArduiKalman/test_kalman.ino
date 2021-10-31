#include "ArduiKalman.h"

#define ADC_PIN 34
#define UPDATE_TIME  100
long current; 

int stateNum = 1;
int measureNum = 1;

//init matrix for Kalman
float xc[1];        // correct state vector 
float xp[1];        // predict state vector 
float A[1][1];      // prediction error covariance 
float Q[1][1];      // process noise covariance 
float R[1][1];      // measurement error covariance
float H[1][1];      // Measurement model
float P[1][1];      // Post-prediction, pre-update
  
KalmanFilter m_kf;

void setup() {
  Serial.begin(115200);
  Serial.print(0);  // To freeze the lower limit
  Serial.print(" ");
  Serial.print(3.3);  // To freeze the upper limit
  Serial.print(" ");
  
  // put your setup code here, to run once:
  m_kf.init(stateNum, measureNum, &A[0][0], &P[0][0], &Q[0][0], &H[0][0], &R[0][0], &xp[0], &xc[0]);
  m_kf.zeros();
  A[0][0] = 1.0f;
  H[0][0] = 1.0f;
  Q[0][0] = 0.01f;
  R[0][0] = 100.0f;
  P[0][0] = 1.0f;

  xc[0] = analogRead(ADC_PIN)/4095.0 * 3.3;
}

void loop() {
  // predict
  float *predict = m_kf.predict();
  // correct
  float rand_noise = random(-100,100)/100.0;
  float measured_value = analogRead(ADC_PIN)/4095.0 * 3.3 + rand_noise;
  float measurement[measureNum];
  measurement[0] = measured_value;
  float *correct = m_kf.correct(measurement);
  float estimated_value = correct[0];
  
  //update plotter
  if (millis() > current) {
    Serial.print(measured_value);
    Serial.print(",");
    Serial.print(estimated_value);
    Serial.println();
    
    current = millis() + UPDATE_TIME;
  }

}
