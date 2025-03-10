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

#ifndef CONSTANT_UKF_H
#define CONSTANT_UKF_H
#include <Eigen/Cholesky> 


#include "multi_target_kf/base_model.h"
#include<stdio.h>
#include<math.h>
 #include <iostream>  
// #include <ros/ros.h>
// #include <math.h>  // log

// #include <Eigen/Dense>


// -----------------------------------------------------------------------//
//------------------------- ConstantAccelModel --------------------------//
//-----------------------------------------------------------------------//

/**
 * @brief Constant velocity model
 */
class ConstantUkfModel
{
protected:

bool debug_; /* Print debug messages */

Eigen::MatrixXd F_; /* State transition matrix */
Eigen::MatrixXd H_; /* Observation jacobian matrix */
Eigen::MatrixXd Q_; /** Process covariance matrix */
Eigen::MatrixXd P_; /* State covariance estimate */
Eigen::MatrixXd Pest; /* State covariance estimate */
Eigen::MatrixXd P12; /* State covariance estimate */
Eigen::MatrixXd P1; /* State covariance estimate */
Eigen::MatrixXd R_; /** Measurements covariance matrix */
Eigen::MatrixXd P2; /** Measurements covariance matrix */
Eigen::MatrixXd K1; /** Measurements covariance matrix */
Eigen::VectorXd x_; /* Current state (mean) vector [x, y, z, vx, vy, vz] */
Eigen::VectorXd xest; /* Current state (mean) vector [x, y, z, vx, vy, vz] */
Eigen::VectorXd err; /* Current state (mean) vector [x, y, z, vx, vy, vz] */
Eigen::MatrixXd Au; /* /* Chol gives a lower (not upper) triangle matrix  */
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
Eigen::MatrixXd diagWc; 
Eigen::MatrixXd diagWm; 

const unsigned int NUM_STATES=6;// constant velocity model
const unsigned int NUM_MEASUREMENTS=3; // position \in R^3
const unsigned int n=6;// constant velocity model
const unsigned int m= NUM_MEASUREMENTS; // position \in R^3
const unsigned int L1= 2*NUM_STATES+1;// constant velocity model
const unsigned int beta= 2;// constant velocity model
const float alpha=0.1; // 0.03
const float ki=0.02; // position \in R^3
const float lambda= (alpha*alpha)*(n+ki)-n; // position \in R^3
const float c= n+lambda; // position \in R^3
const float C=sqrt(c); // position \in R^3

double dt_; /* Prediction sampling time */
ros::Time current_t_; /* Current time stamp */

Eigen::VectorXd acc_variance_; /* 3D vector for accelration variances (sigma^2) in x,y z */

double sigma_a_, sigma_p_, sigma_v_;

public:

void
debug(bool d){
    debug_ = d;
}

ConstantUkfModel():
dt_(0.01),
F_(Eigen::MatrixXd::Zero(NUM_STATES,NUM_STATES)),
H_(Eigen::MatrixXd::Zero(NUM_MEASUREMENTS,NUM_STATES)),
Q_(Eigen::MatrixXd::Zero(NUM_STATES,NUM_STATES)),
P_(Eigen::MatrixXd::Zero(NUM_STATES,NUM_STATES)),
P1(Eigen::MatrixXd::Zero(NUM_STATES,NUM_STATES)),
Pest(Eigen::MatrixXd::Zero(NUM_STATES,NUM_STATES)),
P12(Eigen::MatrixXd::Zero(NUM_STATES,NUM_STATES)),
K1(Eigen::MatrixXd::Zero(NUM_STATES,NUM_STATES)),
R_(Eigen::MatrixXd::Zero(NUM_MEASUREMENTS,NUM_MEASUREMENTS)),
P2(Eigen::MatrixXd::Zero(NUM_MEASUREMENTS,NUM_MEASUREMENTS)),
x_(Eigen::MatrixXd::Zero(NUM_STATES,1)),
err(Eigen::MatrixXd::Zero(NUM_STATES,1)),
xest(Eigen::MatrixXd::Zero(NUM_STATES,1)),
Au(Eigen::MatrixXd::Zero(NUM_STATES,NUM_STATES)),
Xsig(Eigen::MatrixXd::Zero(NUM_STATES,NUM_STATES*2+1)), //sigma matrix
x1(Eigen::MatrixXd::Zero(NUM_STATES,1)),
X1(Eigen::MatrixXd::Zero(NUM_STATES,NUM_STATES*2+1)),
X2(Eigen::MatrixXd::Zero(NUM_STATES,NUM_STATES*2+1)),
X1col(Eigen::MatrixXd::Zero(NUM_STATES,1)),
Xcol(Eigen::MatrixXd::Zero(NUM_STATES,1)),
Wm(Eigen::MatrixXd::Zero(1,NUM_STATES*2+1)),/* weights for means	 */
Wc(Eigen::MatrixXd::Zero(1,NUM_STATES*2+1)),/* weights for covariance	 */
diagWc(Eigen::MatrixXd::Zero(NUM_STATES*2+1,NUM_STATES*2+1)),
diagWm(Eigen::MatrixXd::Zero(NUM_STATES*2+1,NUM_STATES*2+1)),
Z2(Eigen::MatrixXd::Zero(NUM_MEASUREMENTS,NUM_STATES*2+1)),
Z1(Eigen::MatrixXd::Zero(NUM_MEASUREMENTS,NUM_STATES*2+1)),
z1(Eigen::MatrixXd::Zero(NUM_MEASUREMENTS,1)),
Z1col(Eigen::MatrixXd::Zero(NUM_MEASUREMENTS,1)),
acc_variance_(Eigen::VectorXd::Zero(3))
{}
~ConstantUkfModel(){}

unsigned int
numStates(){
    return NUM_STATES;
}


unsigned int
numMeasurements(){
    return NUM_MEASUREMENTS;
}

/**
 * @brief Sets the acceleration variance vector acc_variance_
 * @param a Eigen::VectorXd 3D vector contains [ax,ay,az]
 * @return bool True if size of a is 3. Otheriwse, returns false
*/
bool
setAccVar(Eigen::VectorXd a)
{
    if (a.size() != 3) return false;

    acc_variance_.resize(3); acc_variance_= a;
    return true;
}

bool
setSigmaA(double sigma_a)
{
    if (sigma_a<=0) return false;
    sigma_a_ = sigma_a;

    if (debug_) std::cout << " sigma_a: " << sigma_a_ << "\n";
    return true;
}

bool
setSigmaP(double sigma_p)
{
    if (sigma_p<=0) return false;
    sigma_p_ = sigma_p;
    if (debug_) std::cout << " sigma_p: " << sigma_p_ << "\n";
    return true;
}

bool
setSigmaV(double sigma_v)
{
    if (sigma_v<=0) return false;
    sigma_v_ = sigma_v;
    if (debug_) std::cout << " sigma_v: " << sigma_v_ << "\n";
    return true;
}

/**
 * @brief Computes the state prediction using internal state x_, and dt_
 * @param state vector
 * @return predicted state vector
 */
Eigen::VectorXd
f()
{
    auto new_x = f(x_, dt_);
    return new_x;
}

/**
 * @brief Computes the state prediction using model given state, and internal dt_
 * @param state vector
 * @return predicted state vector
 */
Eigen::VectorXd
f(Eigen::VectorXd x)
{
    Eigen::VectorXd new_x = f(x, dt_); // Updates x_
    return new_x;
}

/**
 * @brief Computes the state prediction using model f(x)
 * @param state vector
 * @param dt sampling time in seconds
 * @return predicted state vector
 */
Eigen::VectorXd
f(Eigen::VectorXd x, double dt)
{
    if(debug_)
        ROS_INFO("[ConstantUkfModel::f] Calculating f");

    if (dt <= 0){
        ROS_WARN("[ConstantUkfModel::f] dt is <= 0. Returning same x");
        return x;
    }

    Eigen::MatrixXd A = Eigen::MatrixXd::Identity(NUM_STATES,NUM_STATES);

    // constant velocity model
    // A(0,3) = dt; // x - vx
    // A(1,4) = dt; // y - vy
    // A(2,5) = dt; // z - vz

    // The following is based on this thesis:
   // (https://dspace.cvut.cz/bitstream/handle/10467/76157/F3-DP-2018-Hert-Daniel-thesis_hertdani.pdf?sequence=-1&isAllowed=y)
   A.block(0,3,3,3)= dt*Eigen::MatrixXd::Identity(3,3);
    
    return (A*x);
}

/**
 * @brief State transition matrix of a linear system
 * @param double Sampling time in seconds
 * @return MatrixXd State transition matrix
 */
Eigen::MatrixXd
F(double dt){
    if(debug_)
        ROS_INFO("[ConstantUkfModel::F] Calculating F");

    Eigen::MatrixXd A = Eigen::MatrixXd::Identity(NUM_STATES,NUM_STATES);

    // constant velocity model
    // A(0,3) = dt; // x - vx
    // A(1,4) = dt; // y - vy
    // A(2,5) = dt; // z - vz

    // The following is based on this thesis:
    // (https://dspace.cvut.cz/bitstream/handle/10467/76157/F3-DP-2018-Hert-Daniel-thesis_hertdani.pdf?sequence=-1&isAllowed=y)
    A.block(0,3,3,3)= dt*Eigen::MatrixXd::Identity(3,3);

    return A;
}

/**
 * @brief Computes the observation model h(x)
 * @param x state vector
 * @return observation z vector
 */
Eigen::VectorXd
h(Eigen::VectorXd x)
{
    if(debug_)
        ROS_INFO("[ConstantUkfModel::h] Calculating h");

    // The following is based on this thesis:
    // (https://dspace.cvut.cz/bitstream/handle/10467/76157/F3-DP-2018-Hert-Daniel-thesis_hertdani.pdf?sequence=-1&isAllowed=y)
    H_.resize(NUM_MEASUREMENTS, NUM_STATES);
    H_ = Eigen::MatrixXd::Zero(NUM_MEASUREMENTS,NUM_STATES);
    H_(0,0) = 1.0; // observing x
    H_(1,1) = 1.0; // observing y
    H_(2,2) = 1.0; // observing z

    Eigen::VectorXd z = H_*x;
    if(debug_)
        std::cout << "h(x) = \n" << z << "\n";
    return z;
}

/**
 * @brief Returns observation matrix H_
 */
Eigen::MatrixXd
H(void){
    if(debug_)
        ROS_INFO("[ConstantUkfModel::H] Returning H_");

    // The following is based on this thesis:
    // (https://dspace.cvut.cz/bitstream/handle/10467/76157/F3-DP-2018-Hert-Daniel-thesis_hertdani.pdf?sequence=-1&isAllowed=y)
    H_.resize(NUM_MEASUREMENTS, NUM_STATES);
    H_ = Eigen::MatrixXd::Zero(NUM_MEASUREMENTS,NUM_STATES);
    H_(0,0) = 1.0; // observing x
    H_(1,1) = 1.0; // observing y
    H_(2,2) = 1.0; // observing z
    return H_;
}

/**
 * @brief Sets observation matrix H_
 * @param M MatrixXd Input observation matrix
 * @return Bool True if dimensions are OK.
 */
bool
H(Eigen::MatrixXd M){
    if(debug_)
        ROS_INFO("[ConstantUkfModel::H] Setting H_");

    if(M.cols() == NUM_STATES && M.rows() == NUM_MEASUREMENTS){
        H_.resize(NUM_MEASUREMENTS, NUM_STATES);
        H_ = M;
        return true;
    }
    else
        return false;
}


/**
 * @brief Returns Q_
 */
Eigen::MatrixXd
Q(void)
{
    return Q_;
}

/**
 * @brief Sets the process covariance matrix Q. Used in the prediction step of the Kalman filter
 * @param dt [double] time step in seconds
 * @param sigma_a [double] Standard deviation of the process accelration noise
*/
bool
Q(double dt, double sigma_a){
    if (dt <= 0) return false;
    if (sigma_a <= 0) return false;


    // Initialize
    Q_ = Eigen::MatrixXd::Identity(NUM_STATES, NUM_STATES);

    // Construct the upper left block, Qpp
    Q_.block(0,0,3,3) = dt*dt*dt*dt/4*Eigen::MatrixXd::Identity(3,3);
    // Construct the upper right block, Qpv
    Q_.block(0,3,3,3) = dt*dt*dt/2*Eigen::MatrixXd::Identity(3,3);
    // Construct the lower left block, Qvp
    Q_.block(3,0,3,3) = dt*dt*dt/2*Eigen::MatrixXd::Identity(3,3);
    // Construct the lower right block, Qvv
    Q_.block(3,3,3,3) = dt*dt*Eigen::MatrixXd::Identity(3,3);

    Q_ = sigma_a*sigma_a*Q_;

    if(debug_) std::cout << "Q: \n" << Q_ << "\n";


    return true;
}

Eigen::MatrixXd
Q(double dt){
    if (dt <= 0) return Eigen::MatrixXd::Identity(NUM_STATES, NUM_STATES);;


    // Initialize
    Q_ = Eigen::MatrixXd::Identity(NUM_STATES, NUM_STATES);

    // Construct the upper left block, Qpp
    Q_.block(0,0,3,3) = dt*dt*dt*dt/4*Eigen::MatrixXd::Identity(3,3);
    // Construct the upper right block, Qpv
    Q_.block(0,3,3,3) = dt*dt*dt/2*Eigen::MatrixXd::Identity(3,3);
    // Construct the lower left block, Qvp
    Q_.block(3,0,3,3) = dt*dt*dt/2*Eigen::MatrixXd::Identity(3,3);
    // Construct the lower right block, Qvv
    Q_.block(3,3,3,3) = dt*dt*Eigen::MatrixXd::Identity(3,3);

    Q_ = sigma_a_*sigma_a_*Q_;


    return Q_;
}



/**
 * @brief Returns R_
 * @return MatrixXd Measurement covariance matrix R_
 */
Eigen::MatrixXd
R(void)
{
    return R_;
}

/**
 * @brief Sets R_
 * @param MatrixXd Input R matrix
 * @return Bool. True if dimensions are OK. 
 */
bool
R(Eigen::MatrixXd M)
{
    if(debug_)
        ROS_INFO("[ConstantUkfModel::R] Setting R_ from a matrix");

    if (M.rows() == NUM_MEASUREMENTS && M.cols() == NUM_MEASUREMENTS){
        R_.resize(NUM_MEASUREMENTS,NUM_MEASUREMENTS);
        R_ = M;
        return true;
    }
    else
        return false;
}

/**
 * @brief Initialize R_ as a diagonal matrix using a std::vector representing diagonal elements
 * @param v std::vector<double> Vector of diagonla elements.  Its size must be equal to NUM_MEASUREMENTS
 * @return Bool True if size of input vector = NUM_MEASUREMENTS
 */
bool
R(std::vector<double> v){
    if(debug_)
        ROS_INFO("[ConstantUkfModel::R] Setting diagonal R_ from a vector");

    if((unsigned int)v.size()!=NUM_MEASUREMENTS){
        ROS_ERROR("[ConstantUkfModel::R] Input vector size != NUM_MEASUREMENTS, v.size = %lu", v.size());
        return false;
    }
    Eigen::VectorXd temp = Eigen::MatrixXd::Zero(NUM_MEASUREMENTS,1);
    for (int i=0; i< NUM_MEASUREMENTS; i++) temp(i) = v[i];

    R_.resize(NUM_MEASUREMENTS, NUM_MEASUREMENTS);
    R_ = temp.asDiagonal();
    if(debug_)
        std::cout << "R_=" << std::endl << R_ << std::endl;
    return true;
}

/**
 * @brief Returns P_
 * @return MatrixXd State covariance matrix
 */
Eigen::MatrixXd
P(void)
{
    return P_;
}

/**
 * @brief Sets P_
 * @param MatrixXd input state covariance matrix
 * @return Bool. True if dimensions are OK.
 */
bool
P(Eigen::MatrixXd M)
{
    if(debug_){
        ROS_INFO("[ConstantUkfModel::P] Setting P from a matrix");
    }

    if (M.rows() == NUM_STATES && M.cols() == NUM_STATES){
        P_.resize(NUM_STATES,NUM_STATES);
        P_ = M;
        if(debug_)
            std::cout << "P: " << std::endl << P_<< std::endl;
        return true;
    }
    else
        return false;
}

/**
 * @brief Sets P_
 * @param sigma_p [double] Standard deviation of position
 * @param sigma_v [double] Standard deviation of velocity
 * @return Bool. True if sigma_p && and sigma_va are non-zero
 */
bool
P(double sigma_p, double sigma_v)
{
    if (sigma_p <=0 || sigma_v<=0) return false;
    
    if(debug_){
        ROS_INFO("[ConstantUkfModel::P] Setting P from standard deviations");
    }
    P_ = Eigen::MatrixXd::Identity(NUM_STATES, NUM_STATES);
    P_.block(0,0,3,3) = sigma_p*sigma_p*Eigen::MatrixXd::Identity(3,3);
    P_.block(3,3,3,3) = sigma_v*sigma_v*Eigen::MatrixXd::Identity(3,3);
    
    if (debug_) std::cout << "P: \n" << P_ << "\n";

    return true;
}

/**
 * @brief Returns current state vectror 
 * @return VectorXd State vector x_
 */
Eigen::VectorXd
getx(void){
    return x_;
}

/**
 * @brief Sets current state vectror x_
 * @param VectorXd Input state vector
 * @return Bool. True if dimensions are OK.
 */
bool
setx(Eigen::VectorXd v){
    if(v.cols() == 1 and v.rows()==NUM_STATES){
        x_ = v;
        return true;
    }
    else
        return false;
}



/**
 * @brief Prediciton step of a discrete KF using given state s and dt
 * @return kf_state current state (includes time, state, covariance)
 * @return kf_state Predicted state (includes time, state, covariance)
 */
kf_state
predictX(kf_state s, double dt){
    if(debug_)
        ROS_INFO("[ConstantUkfModel::predictX] Predicting x");

    if (dt <= 0){
        ROS_WARN("[ConstantUkfModel::predictX] dt = %f <= 0. Returning same state", dt);
        return s;
    }

    if(debug_)
        ROS_INFO("[ConstantUkfModel::predictX] det(P) of current state: %f", s.P.determinant());

    kf_state xx;
    xx.time_stamp = s.time_stamp + ros::Duration(dt);
    
    
      /* weight equations are found in the upper part of http://www.cslu.ogi.edu/nsel/ukf/node6.html */
   
    Wm(0,0) = lambda/c;
    Wc(0,0) = lambda/(c+(1-(alpha*alpha)+beta));
    for (unsigned int k=1; k<L1; k++)
    {
	      Wm(0,k) = 0.5/c;
	      Wc(0,k) = 0.5/c;
    }
    
    auto FF = F(dt);
    xx.P = FF*s.P*FF.transpose()+Q(dt);
    xx.x = f(s.x, dt);
    //std::cout << xx.x << std::endl;
    Eigen::MatrixXd L( s.P.llt().matrixL() );
   // std::cout << L << std::endl;

    Xsig = Eigen::MatrixXd::Zero(n,L1);
    unsigned int k=0;
	{
		for (unsigned int i=0; i<n; i++)
		{
			Xsig(i,k) = s.x(i,0);
		}
	}
	for(k=1; k<n+1; k++)
	{
		for (unsigned int i=0; i<n; i++)
		{
			Xsig(i,k) = s.x(i,0) + C*L(i,k-1);
		}
	}
	for(k=n+1; k<L1; k++)
	{
		for (unsigned int i=0; i<n; i++)
		{
			Xsig(i,k) = s.x(i,0) - C*L(i,k-1-n);
		}
	}
    //std::cout << Xsig << std::endl;
    Xcol = Eigen::MatrixXd::Zero(n,1);
    X1col = Eigen::MatrixXd::Zero(n,1);
    x1 = Eigen::MatrixXd::Zero(n,1);
    X1 = Eigen::MatrixXd::Zero(n,L1);
    for(unsigned int k=0; k<L1; k++)
	{

		  for (unsigned int i=0; i<n; i++)
		  {
			Xcol(i,0) = Xsig(i,k);	// take out a column so that state_function can take it
		  }	
   	
          auto FF = F(dt_); 

	      X1col = FF*Xcol;
		  for (unsigned int i=0; i<n; i++)
		  {
			x1(i,0) +=  Wm(0,k) * X1col(i,0);
		  }
		  for (unsigned int i=0; i<n; i++)
		  {
			X1(i,k) = X1col(i,0);	// put back the output column
		  }	
	}
    //std::cout << x1 << std::endl;
    X2= Eigen::MatrixXd::Zero(NUM_STATES,NUM_STATES*2+1);
    for (unsigned int k=0; k<L1; k++)
		for (unsigned int i=0; i<n; i++)
		{
			X2(i,k) = X1(i,k) - x1(i,0);	//X2.Column(k) = X1.Column(k) - x1;
		}
    //std::cout << X2 << std::endl;
	diagWm =Eigen::MatrixXd::Zero(L1,L1);

	for(unsigned int k=0; k<L1; k++)
		diagWm(k,k) = Wm(0,k);
    
    P12=Eigen::MatrixXd::Zero(NUM_STATES,NUM_MEASUREMENTS);
   
    P1=Eigen::MatrixXd::Zero(NUM_STATES,NUM_STATES);

	P1= X2 * diagWm * X2.transpose()+ Q_;	/* ~ means transpose */
    // std::cout << P1 << std::endl;
	/* unscented transformation (ut) of measurements */
    /* unscented transformation (ut) of measurements */
	
    Z1=Eigen::MatrixXd::Zero(NUM_MEASUREMENTS,NUM_STATES*2+1);
    z1=Eigen::MatrixXd::Zero(NUM_MEASUREMENTS,1);
    Z1col=Eigen::MatrixXd::Zero(NUM_MEASUREMENTS,L1);

	for(unsigned int k=0; k<L1; k++)
	{
		
		for (unsigned int i=0; i<m; i++)
		{
			X1col(i,0) = X1(i,k);	// take out a column so that measurement_function can take it
		}		
		
        Z1col = h(X1col);
        //std::cout << Z1col << std::endl;
		for (unsigned int i=0; i<m; i++)
		{
			z1(i,0) += Wm(0,k) * Z1col(i,0);
		}
		for (unsigned int i=0; i<m; i++)
		{
     		Z1(i,k) = Z1col(i,0);	// put back the output column
		}	
	}
    
    Z2=Eigen::MatrixXd::Zero(NUM_MEASUREMENTS,NUM_STATES*2+1);
    for (unsigned int k=0; k<L1; k++)
	{	
        for (unsigned int i=0; i<m; i++)
		{
			Z2(i,k) = Z1(i,k) - z1(i,0);	//Z2.Column(k) = Z1.Column(k) - z1;
	    }
    }
    
    diagWc =Eigen::MatrixXd::Zero(L1,L1);
    
	for(unsigned int k=0; k<L1; k++)
    {
		diagWc(k,k) = Wc(0,k);
    }

    P2= Eigen::MatrixXd::Zero(NUM_MEASUREMENTS,NUM_MEASUREMENTS);
    P2 = Z2 * diagWc * Z2.transpose() + R_;	
    
	P12 = X2 * diagWc * Z2.transpose();	//transformed cross-covariance
    K1 = Eigen::MatrixXd::Zero(NUM_STATES,NUM_STATES);
	K1 = P12 * P2.inverse();
    
	//cout << x << endl << K << endl;
    //Pest=Eigen::MatrixXd::Zero(NUM_STATES,NUM_STATES);
	//Pest = s.P - K1*P2.inverse()*K1.transpose();                                //covariance update
	//Pest = P1-K1*P12.inverse();       
   //std::cout << "Pest" <<std::endl;                  //covariance update
	//std::cout << Pest << std::endl;
   // std::cout << "P=" <<std::endl;                  //covariance update
	//std::cout << xx.P << std::endl;

    if(debug_){
        ROS_INFO("[ConstantUkfModel::predictX] ||x_new - x_old|| = %f", (xx.x - s.x).norm());
        ROS_INFO("[ConstantUkfModel::predictX] det(P) of predicted state: %f", xx.P.determinant());
    }

    if(debug_){
        std::cout << "[predictX] old P = \n" << s.P << "\n";
        std::cout << "[predictX] old x = \n" << s.x << "\n";
    
    
        std::cout << "[predictX] new P = \n" << xx.P << "\n";
        std::cout << "[predictX] new x = \n" << xx.x << "\n";
    }
    
    return K1, x1, z1, P1, P12, xx;
}

/**
 * @brief Update step of a discrete KF using given state x and dt, and measurements z
 * @param kf_state current state (includes time, state, covariance)
 * @return kf_state Predicted state (includes time, state, covariance)
 */
kf_state
updateX(sensor_measurement z, kf_state s){
    if(debug_)
        ROS_INFO("[ConstantUkfModel::updateX] Updating x");

    kf_state xx;
    xx.time_stamp = z.time_stamp; //z.time_stamp;

    Eigen::VectorXd y = z.z - h(s.x); // innovation
    Eigen::MatrixXd K = K1; // optimal Kalman gain
    xx.x = x1 +K1 *(z.z-z1);                         //state update
    xx.P= P1-K1*P12.transpose();                                //covariance update
	 //cout << P << endl << endl;

    if(debug_){
        ROS_INFO("[ConstantUkfModel::updateX] Done updating state");
        ROS_INFO("[ConstantUkfModel::updateX] Norm of innovation y = %f ", y.norm());
        std::cout << "[ConstantUkfModel::updateX] predicted state P: \n" << s.P << "\n";
        // std::cout << "[ConstantUkfModel::updateX] innovation covariance S: \n" << S << "\n";
        std::cout << "[ConstantUkfModel::updateX] Kalman gain: \n" << K << "\n";
        std::cout << "[ConstantUkfModel::updateX] uncorrected state: \n" << s.x << "\n";
        std::cout << "[ConstantUkfModel::updateX] measurement: \n" << z.z << "\n";
        std::cout << "[ConstantUkfModel::updateX] corrected state: \n" << xx.x << "\n";


    }

    return xx;
}

/**
 * @brief Computes log Likelihood between predicted state and a measurement
 * @param xx kf_state Predicted state (time, state, covariance)
 * @param z sensor_measurement measurement (time, state, covariance)
 * @return double logLikelihood
 */
double
logLikelihood(kf_state xx, sensor_measurement z){
    // if(debug_){
    //     ROS_INFO("[ConstantAccelModel::logLikelihood] Calculating logLikelihood");
    //     std::cout << "x: \n" << xx.x << "\n";
    //     std::cout << "z: \n" << z.z << "\n";
    // }

    Eigen::VectorXd y_hat; //z.z - h(xx.x); // innovation
    y_hat = z.z - h(xx.x); // innovation
    // if(debug_) std::cout << "y_hat: \n" << y_hat << "\n";

    Eigen::MatrixXd S;
    S = R_ + H()*xx.P*H().transpose(); // innovation covariance
    // if(debug_) std::cout << "S: \n" << S << "\n";

    auto S_inv = S.inverse();
    // if (debug_) std::cout << "S_inv \n" << S_inv << "\n";
    auto tmp = y_hat.transpose()*S_inv*y_hat;

    double LL = -0.5 * (tmp + log( std::fabs(S.determinant()) ) + 3.0*log(2*M_PI)); // log-likelihood

    if(debug_)
        ROS_INFO("[ConstantUkfModel::logLikelihood] Done computing logLikelihood. L= %f", LL);

    return LL;
}

kf_state
initStateFromMeasurements(sensor_measurement z){

    if(debug_)
        ROS_INFO("[ConstantUkfModel::initStateFromMeasurements] Initializing state from measurements");

    kf_state state;
    state.time_stamp = z.time_stamp;
    state.x = Eigen::MatrixXd::Zero(NUM_STATES,1);
    state.x(0) = z.z(0);
    state.x(1) = z.z(1);
    state.x(2) = z.z(2);
    
    state.P = Q_;

    return state;
}

double
computeDistance(kf_state xx, sensor_measurement zz){
    auto y = zz.z - h(xx.x);
    auto nrm = y.norm();
    if(debug_)
        ROS_INFO("[ConstantUkfModel::computeDistance] Distance between state and measurement = %f", nrm);
    return nrm;
}

};

#endif //CONSTANT_ACCEL_H
