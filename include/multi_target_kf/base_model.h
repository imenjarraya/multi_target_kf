/*
BSD 3-Clause License

Copyright (c) 2022, Mohamed Abdelkader Zahana
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this
   list of conditions and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice,
   this list of conditions and the following disclaimer in the documentation
   and/or other materials provided with the distribution.

3. Neither the name of the copyright holder nor the names of its
   contributors may be used to endorse or promote products derived from
   this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

#ifndef BASE_MODEL_H
#define BASE_MODEL_H

#include <ros/ros.h>
#include <math.h>  // log

#include <Eigen/Dense>



/**
 * Structure to store the current stamped KF prediction
 */
struct kf_state
{
   ros::Time time_stamp;
   Eigen::VectorXd x; // State estimate
   Eigen::MatrixXd P; // State estimate covariance
};

/**
 * Structure to store current stamped sensor measurement.
 */
struct sensor_measurement
{
   ros::Time time_stamp;
   unsigned int id; /**< OPtional. Associated measurement ID, e.g. Apriltag ID */
   Eigen::VectorXd z; /**< Measurements, e.g. 3D position, velocity, ... etc */
   Eigen::MatrixXd R; /* Measurement covariance matrix */
};


/**
 * @brief Base class that contains all common members and functions for all models.
 * @todo Needs implementation
 */
class BaseModel
{
protected:

Eigen::MatrixXd F_; /* State transition matrix */
Eigen::MatrixXd H_; /* Observation jacobian matrix */
Eigen::MatrixXd Q_; /** Process covariance matrix */
Eigen::MatrixXd P_; /* State covariance estimate */
Eigen::MatrixXd R_; /** Measurements covariance matrix */
Eigen::VectorXd x_; /* Current state (mean) vector [x, y, z, vx, vy, vz] */
Eigen::MatrixXd P1; /* State covariance estimate */
Eigen::MatrixXd P2; /** Measurements covariance matrix */
Eigen::MatrixXd P12; /** Measurements covariance matrix */
Eigen::MatrixXd K1; /** Measurements covariance matrix */
Eigen::VectorXd err; /* Current state (mean) vector [x, y, z, vx, vy, vz] */
Eigen::VectorXd squared_error;
Eigen::MatrixXd Xsig; /* sigma points around x */
Eigen::MatrixXd x1; /* */
Eigen::MatrixXd X1; /* */
Eigen::MatrixXd X2; /* X1(i,k) - x1(i,0); */
Eigen::MatrixXd Xcol; /* f(X)*/
Eigen::MatrixXd X1col; /* */
Eigen::MatrixXd z1; /* */
Eigen::MatrixXd Z1; /* */
Eigen::MatrixXd Z2; /* */
Eigen::MatrixXd Z1col; /* */
Eigen::MatrixXd Wm; /* weights for means */
Eigen::MatrixXd Wc; /* weights for covariance	 */
Eigen::MatrixXd diagWc; //
Eigen::MatrixXd diagWm; //
Eigen::MatrixXd L; //

const unsigned int NUM_STATES=8;// constant velocity model
const unsigned int NUM_MEASUREMENTS=3; // position \in R^3
const unsigned int n=NUM_STATES;// constant velocity model
const unsigned int m= NUM_MEASUREMENTS; // position \in R^3
const unsigned int L1= 2*NUM_STATES+1;// constant velocity model
const unsigned int beta= 2;// constant velocity model
const float alpha=0.21063; // 0.03
const float ki=0.91; // position \in R^3
const float lambda= (alpha*alpha)*(n+ki)-n; // position \in R^3
const float c= n+lambda; // position \in R^3
const float C=sqrt(c); // position \in R^3
double dt_; /* Prediction sampling time */
ros::Time current_t_; /* Current time stamp */

public:
BaseModel(){}
~BaseModel(){}

};

#endif //BASE_MODEL_H
