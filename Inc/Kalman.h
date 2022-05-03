/*
 SimpleKalmanFilter - a Kalman Filter implementation for single variable models.
 Created by Denys Sene, January, 1, 2017.
 Released under MIT License - see LICENSE file for details.
 */ 
#ifndef Kalman
#define Kalman
 // class SimpleKalmanFilter
	 void SimpleKalmanFilter_X(float mea_e_X, float est_e_X, float q_X);
   float updateEstimate_X(float mea_X);
   void setMeasurementError_X(float mea_e_X);
   void setEstimateError_X(float est_e_X);
   void setProcessNoise_X(float q_X);
   float getKalmanGain_X();
   float getEstimateError_X();
	 
	 void SimpleKalmanFilter_Y(float mea_e_Y, float est_e_Y, float q_Y);
   float updateEstimate_Y(float mea_Y);
   void setMeasurementError_Y(float mea_e_Y);
   void setEstimateError_Y(float est_e_Y);
   void setProcessNoise_Y(float q_Y);
   float getKalmanGain_Y();
   float getEstimateError_Y();
	
#endif
