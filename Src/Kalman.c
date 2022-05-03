#include "Kalman.h"
#include "stdint.h"
#include "math.h"

 float _err_measure_X;
 float _err_estimate_X;
 float _q_X;
 float _current_estimate_X;
 float _last_estimate_X;
 float _kalman_gain_X;
 
 float _err_measure_Y;
 float _err_estimate_Y;
 float _q_Y;
 float _current_estimate_Y;
 float _last_estimate_Y;
 float _kalman_gain_Y;

 void SimpleKalmanFilter_X(float mea_e_X, float est_e_X, float q_X)
 {
   _err_measure_X=mea_e_X;
   _err_estimate_X=est_e_X;
   _q_X = q_X;
 }
 void SimpleKalmanFilter_Y(float mea_e_Y, float est_e_Y, float q_Y)
 {
   _err_measure_Y=mea_e_Y;
   _err_estimate_Y=est_e_Y;
   _q_Y = q_Y;
 }

 float updateEstimate_X(float mea_X)
 {
   _kalman_gain_X = _err_estimate_X/(_err_estimate_X + _err_measure_X);
   _current_estimate_X = _last_estimate_X + _kalman_gain_X * (mea_X - _last_estimate_X);
   _err_estimate_X =  (1.0 - _kalman_gain_X)*_err_estimate_X + fabs(_last_estimate_X-_current_estimate_X)*_q_X;
   _last_estimate_X=_current_estimate_X;
 return _current_estimate_X;
 }
 
 float updateEstimate_Y(float mea_Y)
 {
   _kalman_gain_Y = _err_estimate_Y/(_err_estimate_Y + _err_measure_Y);
   _current_estimate_Y = _last_estimate_Y + _kalman_gain_Y * (mea_Y - _last_estimate_Y);
   _err_estimate_Y =  (1.0 - _kalman_gain_Y)*_err_estimate_Y + fabs(_last_estimate_Y-_current_estimate_Y)*_q_Y;
   _last_estimate_Y=_current_estimate_Y;
 return _current_estimate_Y;
 }

 void setMeasurementError_X(float mea_e_X)
 {
   _err_measure_X=mea_e_X;
 }

 void setEstimateError_X(float est_e_X)
 {
   _err_estimate_X=est_e_X;
 }

 void setProcessNoise_X(float q_X)
 {
   _q_X=q_X;
 }

 float getKalmanGain_X() {
   return _kalman_gain_X;
 }

 float getEstimateError_X() {
   return _err_estimate_X;
 }
 void setMeasurementError_Y(float mea_e_Y)
 {
   _err_measure_Y=mea_e_Y;
 }

 void setEstimateError_Y(float est_e_Y)
 {
   _err_estimate_Y=est_e_Y;
 }

 void setProcessNoise_Y(float q_Y)
 {
   _q_Y=q_Y;
 }

 float getKalmanGain_Y() {
   return _kalman_gain_Y;
 }

 float getEstimateError_Y() {
   return _err_estimate_Y;
 }