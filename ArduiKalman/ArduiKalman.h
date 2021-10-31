#ifndef __KALMAN_H__
#define __KALMAN_H__

class KalmanFilter 
{
public:
  float* A;
  float* P;
  float* Q;
  float* H;
  float* R;
  float* xp;
  float* xc;

  KalmanFilter();
  
  void init(int nostates, int nobsers, float *A, float *P, float *Q, float *H, float *R, float *xp, float *xc);
  void zeros();
  float *predict();
  float *correct(float *z);


private:
  int m_States;       
  int m_Obsers;
};

#endif