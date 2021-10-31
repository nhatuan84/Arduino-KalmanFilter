#include "ArduiKalman.h"
#include <math.h>
#include <stdio.h>
#include "mat.h" 

KalmanFilter::KalmanFilter()
{}

void KalmanFilter::init(int nostates, int nobsers, \
                        float *A, float *P, float *Q, float *H, float *R, float *xp, float *xc)
{
  this->m_States = nostates;
  this->m_Obsers = nobsers;
  this->A = A;
  this->P = P;
  this->Q = Q;
  this->H = H;
  this->R = R;
  this->xp = xp;
  this->xc = xc;
}
void KalmanFilter::zeros()
{
  for(int i=0; i<m_States*m_States; i++)
  {
    this->A[i] = 0.0f;
    this->Q[i] = 0.0f;
    this->P[i] = 0.0f;
  }
  for(int i=0; i<m_Obsers*m_States; i++)
  {
    this->H[i] = 0.0f;
  }
  for(int i=0; i<m_Obsers*m_Obsers; i++)
  {
    this->R[i] = 0.0f;
  }
  for(int i=0; i<m_States; i++)
  {
    this->xp[i] = 0.0f;
    this->xc[i] = 0.0f;
  }
}

float *KalmanFilter::predict()
{
  mulmat(A, xc, xp, m_States, m_States, 1);
  return xp;
}

float *KalmanFilter::correct(float *z)
{  
  float tmp0[m_States*m_States];
  float tmp1[m_States*m_Obsers];
  float tmp2[m_Obsers*m_States];
  float tmp3[m_Obsers*m_Obsers];
  float tmp4[m_Obsers*m_Obsers];
  float tmp5[m_Obsers*m_Obsers];
  float tmp6[m_Obsers]; 
  float Ht[m_States*m_Obsers]; 
  float At[m_States*m_States]; 
  float Pp[m_States*m_States];
  float K[m_States*m_Obsers];

  /* P_k = F_{k-1} P_{k-1} F^T_{k-1} + Q_{k-1} */
  mulmat(A, P, tmp0, m_States, m_States, m_States);
  transpose(A, At, m_States, m_States);
  mulmat(tmp0, At, Pp, m_States, m_States, m_States);
  accum(Pp, Q, m_States, m_States);

  /* G_k = P_k H^T_k (H_k P_k H^T_k + R)^{-1} */
  transpose(H, Ht, m_Obsers, m_States);
  mulmat(Pp, Ht, tmp1, m_States, m_States, m_Obsers);

  mulmat(H, Pp, tmp2, m_Obsers, m_States, m_States);
  mulmat(tmp2, Ht, tmp3, m_Obsers, m_States, m_Obsers);
  accum(tmp3, R, m_Obsers, m_Obsers);

  if (cholsl(tmp3, tmp4, tmp5, m_Obsers)) 
  {
    return NULL;
  }
  mulmat(tmp1, tmp4, K, m_States, m_Obsers, m_Obsers);

  /* \hat{x}_k = \hat{x_k} + G_k(z_k - h(\hat{x}_k)) */
  mulmat(H, xp, tmp6, m_Obsers, m_States, m_States);
  sub(z, tmp6, tmp5, m_Obsers);
  mulvec(K, tmp5, tmp2, m_States, m_Obsers);
  add(xp, tmp2, xc, m_States);

  /* P_k = (I - G_k H_k) P_k */
  mulmat(K, H, tmp0, m_States, m_Obsers, m_States);
  negate(tmp0, m_States, m_States);
  mat_addeye(tmp0, m_States);
  mulmat(tmp0, Pp, P, m_States, m_States, m_States);

  return xc;
}