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

#ifndef CONSTANT_VEL_UKF_H
#define CONSTANT_VEL_UKF_H

#include "multi_target_kf/structs.h"
#include <vector>


// -----------------------------------------------------------------------//
//------------------------- ConstantAccelModel --------------------------//
//-----------------------------------------------------------------------//

/**
 * @brief Constant velocity model
 */
class ConstantVelUkfModel
{
private:
    Eigen::MatrixXd F_; /* State transition jacobian matrix */
    Eigen::MatrixXd H_; /* Observation jacobian matrix */
    Eigen::MatrixXd Q_; /** Process covariance matrix */
    Eigen::MatrixXd P_; /* State covariance estimate */
    Eigen::MatrixXd R_; /** Measurements covariance matrix */
    Eigen::VectorXd x_; /* Current state vector [px, py, pz, theta, gamma, theta_dot, gamma_dot, speed] */
    Eigen::MatrixXd X1col; /* f(X)*/
    Eigen::MatrixXd P12; /* State covariance estimate P12*/
    Eigen::MatrixXd P1; /* State covariance estimate */
    Eigen::MatrixXd P2; /** Measurements covariance matrix */
    Eigen::MatrixXd K; /** Measurements covariance matrix */
    Eigen::MatrixXd Xsig; /* sigma points around x */
    Eigen::MatrixXd x1; /* */
    Eigen::MatrixXd X1; /* */
    Eigen::MatrixXd X2; /* X1(i,k) - x1(i,0); */
    Eigen::MatrixXd Xcol; /* f(X)*/
    Eigen::MatrixXd z1; /* */
    Eigen::MatrixXd Z1; /* */
    Eigen::MatrixXd Z2; /* */
    Eigen::MatrixXd Z1col; /* */
    Eigen::MatrixXd Wm; /* weights for means */
    Eigen::MatrixXd Wc; /* weights for covariance	 */
    Eigen::MatrixXd diagWc; 
    Eigen::MatrixXd diagWm; 
    unsigned int NUM_STATES=6; /* State dimension */
    unsigned int NUM_MEASUREMENTS=3; /* Measurements dimension */
    unsigned int nu=NUM_STATES;// constant velocity model
    unsigned int m= NUM_MEASUREMENTS; // position \in R^3
    unsigned int L1= 2*NUM_STATES+1;// constant velocity model
    unsigned int beta= 1.2;// constant velocity model
    float alpha=0.21063; // 0.03
    float ki=0.091; // position \in R^3
    float lambda= (alpha*alpha)*(nu+ki)-nu; // position \in R^3
    float c= nu+lambda; // position \in R^3
    float C=sqrt(c); // position \in R^3
    double dt_; /* Prediction sampling time */
    double current_t_; /* Current time stamp */

    bool debug_;

    Eigen::VectorXd acc_variance_; /* 3D vector for accelration variances (sigma^2) in x,y z */

    double sigma_a_, sigma_p_, sigma_v_;

public:

    /* Constructor */
    ConstantVelUkfModel()
    {
        init();
        acc_variance_ = Eigen::VectorXd::Zero(3);
    }
    ~ConstantVelUkfModel(){}

    bool
    init(void)
    {
        F_.resize(NUM_STATES,NUM_STATES);
        F_ = Eigen::MatrixXd::Zero(NUM_STATES,NUM_STATES);
        H_.resize(NUM_MEASUREMENTS,NUM_STATES);
        H_ = Eigen::MatrixXd::Zero(NUM_MEASUREMENTS,NUM_STATES);
        Q_.resize(NUM_STATES,NUM_STATES);
        Q_ = Eigen::MatrixXd::Zero(NUM_STATES,NUM_STATES);
        P_.resize(NUM_STATES,NUM_STATES);
        P_ = Eigen::MatrixXd::Zero(NUM_STATES,NUM_STATES);
        R_.resize(NUM_MEASUREMENTS,NUM_MEASUREMENTS);
        R_ = Eigen::MatrixXd::Zero(NUM_MEASUREMENTS,NUM_MEASUREMENTS);
        X1col.resize(NUM_STATES,1);
        X1col =Eigen::MatrixXd::Zero(NUM_STATES,1);
        P1.resize(NUM_STATES,NUM_STATES);
        P1 = Eigen::MatrixXd::Zero(NUM_STATES,NUM_STATES);
        P12.resize(NUM_STATES,NUM_STATES);
        P12 =Eigen::MatrixXd::Zero(NUM_STATES,NUM_STATES);
        K.resize(NUM_STATES,NUM_STATES);
        K=Eigen::MatrixXd::Zero(NUM_STATES,NUM_STATES);
        P2.resize(NUM_MEASUREMENTS,NUM_MEASUREMENTS);
        P2=Eigen::MatrixXd::Zero(NUM_MEASUREMENTS,NUM_MEASUREMENTS);
        Xsig.resize(NUM_STATES,NUM_STATES*2+1);
        Xsig=Eigen::MatrixXd::Zero(NUM_STATES,NUM_STATES*2+1);
        x1.resize(NUM_STATES,1);
        x1=Eigen::MatrixXd::Zero(NUM_STATES,1);
        X1.resize(NUM_STATES,NUM_STATES*2+1);
        X1=Eigen::MatrixXd::Zero(NUM_STATES,NUM_STATES*2+1);
        X2.resize(NUM_STATES,NUM_STATES*2+1);
        X2=Eigen::MatrixXd::Zero(NUM_STATES,NUM_STATES*2+1);
        Xcol.resize(NUM_STATES,1);
        Xcol=Eigen::MatrixXd::Zero(NUM_STATES,1);
        Wm.resize(1,NUM_STATES*2+1);/* weights for means	 */
        Wm=Eigen::MatrixXd::Zero(1,NUM_STATES*2+1),/* weights for means	 */
        Wc.resize(1,NUM_STATES*2+1);/* weights for covariance	 */
        Wc=Eigen::MatrixXd::Zero(1,NUM_STATES*2+1),/* weights for covariance	 */
        diagWc.resize(NUM_STATES*2+1,NUM_STATES*2+1);
        diagWc=Eigen::MatrixXd::Zero(NUM_STATES*2+1,NUM_STATES*2+1);
        diagWm.resize(NUM_STATES*2+1,NUM_STATES*2+1);
        diagWm=Eigen::MatrixXd::Zero(NUM_STATES*2+1,NUM_STATES*2+1);
        Z2.resize(NUM_MEASUREMENTS,NUM_STATES*2+1);
        Z2=Eigen::MatrixXd::Zero(NUM_MEASUREMENTS,NUM_STATES*2+1);
        Z1.resize(NUM_MEASUREMENTS,NUM_STATES*2+1);
        Z1=Eigen::MatrixXd::Zero(NUM_MEASUREMENTS,NUM_STATES*2+1);
        z1.resize(NUM_MEASUREMENTS,1);
        z1=Eigen::MatrixXd::Zero(NUM_MEASUREMENTS,1);
        x_.resize(NUM_STATES,1);
        x_ = Eigen::MatrixXd::Zero(NUM_STATES,1);
        debug_ = false;
        dt_=0.01;
        current_t_ = 0.0;

        return(true);

    }
    unsigned int
    numStates(){
        return NUM_STATES;
    }

    bool
    debug(void)
    {
        return debug_;
    }

    void
    debug(bool d)
    {
        debug_ = d; 
    }

    bool
    setNumStates(unsigned int n){
        if(n<1) return false;
        NUM_STATES = n;
        return true;
    }

    unsigned int
    numMeasurements(){
        return NUM_MEASUREMENTS;
    }

    bool
    setNumMeasurements(unsigned int n){
        if(n<1) return false;
        NUM_MEASUREMENTS = n;
        return true;
    }

    void
    setCurrentTimeStamp(double t){
        current_t_ = t;
    }

    bool
    setDt(double dt){
        if (dt < 0){
            printf("WARN [DubinModel::setDt] dt < 0 \n");
            return false;
        }
        dt_ = dt;
        return true;
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
        if (sigma_a<=0)
        {
            printf("ERROR [ConstVelUkf::setSigmaA] sigma_a <=0 \n");
            return false;
        }
        sigma_a_ = sigma_a;

        if (debug()) std::cout << " sigma_a: " << sigma_a_ << "\n";
        return true;
    }

    bool
    setSigmaP(double sigma_p)
    {
        if (sigma_p<=0)
        {
            printf("ERROR [ConstVelUkf::setSigmaP] sigma_p <= 0 \n");
            return false;
        }
        sigma_p_ = sigma_p;
        if (debug()) std::cout << " sigma_p: " << sigma_p_ << "\n";
        return true;
    }

    bool
    setSigmaV(double sigma_v)
    {
        if (sigma_v<=0)
        {
            printf("ERROR [setSigmaV] seigma_v <= 0 \n");
            return false;
        }
        sigma_v_ = sigma_v;
        if (debug()) std::cout << " sigma_v: " << sigma_v_ << "\n";
        return true;
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
        if(debug())
            printf("[ConstantVelUkfModel::f] Calculating f \n");

        if (dt <= 0){
            printf("[ConstantVelUkfModel::f] dt is <= 0. Returning same x \n");
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
    * @brief Computes the state transition jacobian matrix F
    * @param x state vector
    * @param dt sampling time in seconds
    * @return jacobain F matrix
    */
//    Eigen::MatrixXd
//    F(Eigen::VectorXd x, double dt)
//    {
//       // x = [px, py, pz, theta, gamma, theta_dot, gamma_dot, speed]
//       //      0    1   2   3      4      5           6          7
//       // F_.resize(NUM_STATES,NUM_STATES);
//       F_ = Eigen::MatrixXd::Identity(NUM_STATES,NUM_STATES);
      

//       /** @todo Needs specific implementation */

//       return F_;
//    }

    /**
     * @brief State transition matrix of a linear system
     * @param double Sampling time in seconds
     * @return MatrixXd State transition matrix
     */
    Eigen::MatrixXd
    F(double dt)
    {
        if(debug())
            printf("[ConstantVelUkfModel::F] Calculating F \n");

        // constant velocity model
        // A(0,3) = dt; // x - vx
        // A(1,4) = dt; // y - vy
        // A(2,5) = dt; // z - vz

        // The following is based on this thesis:
        // (https://dspace.cvut.cz/bitstream/handle/10467/76157/F3-DP-2018-Hert-Daniel-thesis_hertdani.pdf?sequence=-1&isAllowed=y)
        F_.block(0,3,3,3)= dt*Eigen::MatrixXd::Identity(3,3);

        return F_;
    }

    /**
    * @brief Computes the state transition jacobian matrix F
    * @param x state vector
    * @return jacobain F matrix
    */
    // Eigen::MatrixXd
    // F(Eigen::VectorXd x)
    // {
    //     F_ = F(x,dt_);
    //     return F_;
    // }

    /**
    * @brief Sets transition jacobian matrix F_
    * @param M Matrix of size (NUM_STATES, NUM_STATES)
    */
    bool
    F(Eigen::MatrixXd M){
        if (M.cols()==NUM_STATES && M.rows()==NUM_STATES){
            F_.resize(NUM_STATES,NUM_STATES);
            F_=M;
            return true;
        }
        else    return false;
        
    }

    /**
    * @brief Returns transition jacobian matrix F_
    */
    Eigen::MatrixXd
    F()
    {
        return F_;
    }

    /**
     * @brief Computes the observation model h(x)
     * @param x state vector
     * @return observation z vector
     */
    Eigen::VectorXd
    h(Eigen::VectorXd x)
    {
        if(debug())
            printf("[ConstantVelUkfModel::h] Calculating h \n");

        // The following is based on this thesis:
        // (https://dspace.cvut.cz/bitstream/handle/10467/76157/F3-DP-2018-Hert-Daniel-thesis_hertdani.pdf?sequence=-1&isAllowed=y)
        H_.resize(NUM_MEASUREMENTS, NUM_STATES);
        H_ = Eigen::MatrixXd::Zero(NUM_MEASUREMENTS,NUM_STATES);
        H_(0,0) = 1.0; // observing x
        H_(1,1) = 1.0; // observing y
        H_(2,2) = 1.0; // observing z

        Eigen::VectorXd z = H_*x;
        if(debug())
            std::cout << "h(x) = \n" << z << "\n";
        return z;
    }

    /**
     * @brief Returns observation matrix H_
     */
    Eigen::MatrixXd
    H(void)
    {
        if(debug())
            printf("[ConstantVelUkfModel::H] Returning H_ \n");

        // The following is based on this thesis:
        // (https://dspace.cvut.cz/bitstream/handle/10467/76157/F3-DP-2018-Hert-Daniel-thesis_hertdani.pdf?sequence=-1&isAllowed=y)
        Eigen::MatrixXd T = Eigen::MatrixXd::Zero(NUM_MEASUREMENTS,NUM_STATES);
        
        H_(0,0) = 1.0; // observing x
        H_(1,1) = 1.0; // observing y
        H_(2,2) = 1.0; // observing z

        return H_;
    }

    /**
    * @brief Sets Jacobain observation matrix H_
    * @param M MatrixXd Input observation matrix
    * @return Bool True if dimensions are OK.
    */
    bool
    H(Eigen::MatrixXd M){
        if(debug_)
            printf("[ConstVelUkf::H] Setting H_ \n");

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
    Q(double dt, double sigma_a)
    {
        if (dt <= 0)
        {
            printf("ERORR [ConstVelUkf::Q] dt<=0 \n");
            return false;
        }
        if (sigma_a <= 0)
        {
            printf("ERROR [ConstVelUkf::Q] sigma_a <=0 \n");
            return false;
        }


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

        if(debug()) std::cout << "Q: \n" << Q_ << "\n";


        return true;
    }

    Eigen::MatrixXd
    Q(double dt)
    {
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
    * @brief Sets Q_ from Eigen::MatrixXd
    * @param M Eigen::MatrixXd
    */
    bool
    Q(Eigen::MatrixXd M)
    {
        if (M.cols() == NUM_STATES && M.rows()==NUM_STATES){
            Q_.resize(NUM_STATES,NUM_STATES);
            Q_=M;
            return true;
        }
        else
            return false;
    }

    /**
    * @brief Initialize Q_ as a diagonal matrix using a std::vector representing diagonal elements
    * @param v std::vector<double> Vector of diagonla elements.  Its size must be equal to NUM_STATES
    * @return Bool True if size of input vector = NUM_STATES
    */
    bool
    Q(std::vector<double> v){
        if(v.size()!=NUM_STATES){
            printf("ERROR [ConstVelUkf::Q] Input vector size != NUM_STATES. %lu != %u  \n", v.size(), NUM_STATES);
            return false;
        }
        Eigen::VectorXd temp = Eigen::MatrixXd::Zero(NUM_STATES,1);
        for (unsigned int i=0; i< NUM_STATES; i++) temp(i) = v[i];

        Q_ = temp.asDiagonal();
        return true;
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
    * @brief Sets R_ using MatrixXd
    * @param MatrixXd Input R matrix
    * @return Bool. True if dimensions are OK. 
    */
    bool
    R(Eigen::MatrixXd M)
    {
        if(debug_)
            printf("[ConstVelUkf::R] Setting R_ from a matrix \n");

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
            printf("[ConstVelUkf::R] Setting diagonal R_ from a vector \n");

        if((unsigned int)v.size()!=NUM_MEASUREMENTS){
            printf("ERROR [ConstVelUkf::R] Input vector size != NUM_MEASUREMENTS, v.size = %lu != %u \n", v.size(), NUM_MEASUREMENTS);
            return false;
        }
        Eigen::VectorXd temp = Eigen::MatrixXd::Zero(NUM_MEASUREMENTS,1);
        for (unsigned int i=0; i< NUM_MEASUREMENTS; i++) temp(i) = v[i];

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
     * @param sigma_p [double] Standard deviation of position
     * @param sigma_v [double] Standard deviation of velocity
     * @return Bool. True if sigma_p && and sigma_va are non-zero
     */
    bool
    P(double sigma_p, double sigma_v)
    {
        if (sigma_p <=0 || sigma_v<=0)
        {
            printf("ERROR [ConstVelUkf::P] sigma_p or sigma_v <=0 \n");
            return false;
        }
        
        if(debug()){
            printf("[ConstantVelUkfModel::P] Setting P from standard deviations \n");
        }
        P_ = Eigen::MatrixXd::Identity(NUM_STATES, NUM_STATES);
        P_.block(0,0,3,3) = sigma_p*sigma_p*Eigen::MatrixXd::Identity(3,3);
        P_.block(3,3,3,3) = sigma_v*sigma_v*Eigen::MatrixXd::Identity(3,3);

        if (debug()) std::cout << "P: \n" << P_ << "\n";

        return true;
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
            printf("[ConstVelUkf::P] Setting P from a matrix \n");
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
            printf("[ConstVelUkf::predictX] Predicting x \n");

        if (dt <= 0){
            printf("WARN [ConstVelUkf::predictX] dt = %f <= 0. Returning same state \n", dt);
            return s;
        }

        if(debug_)
            printf("[ConstVelUkf::predictX] det(P) of current state: %f \n", s.P.determinant());

        kf_state xx;
        xx.time_stamp = s.time_stamp + dt;

        auto FF = F(dt);
        xx.P = FF*s.P*FF.transpose()+Q(dt);
        xx.x = f(s.x, dt);
        Wm(0,0) = lambda/c;
        Wc(0,0) = lambda/(c+(1-(alpha*alpha)+beta));
        for (unsigned int k=1; k<L1; k++)
        {
	         Wm(0,k) = 0.5/c;
	         Wc(0,k) = 0.5/c;
        }
        Eigen::MatrixXd L( s.P.llt().matrixL() );
        // std::cout << L << std::endl;
        Xsig = Eigen::MatrixXd::Zero(nu,L1);
        unsigned int k=0;
	    {
		   for (unsigned int i=0; i<nu; i++)
		   {
			   Xsig(i,k) = s.x(i,0);
           }
	    }

	    for(k=1; k<nu+1; k++)
	    {
		   for (unsigned int i=0; i<nu; i++)
		   {
			   Xsig(i,k) = s.x(i,0) + C*L(i,k-1);
		   }
	    }
	    for(k=nu+1; k<L1; k++)
	    {
		   for (unsigned int i=0; i<nu; i++)
		    {
			    Xsig(i,k) = s.x(i,0) - C*L(i,k-1-nu);
		    }
	    }
        //std::cout << Xsig << std::endl;
        Xcol = Eigen::MatrixXd::Zero(nu,1);
        X1col = Eigen::MatrixXd::Zero(nu,1);
        x1 = Eigen::MatrixXd::Zero(nu,1);
        X1 = Eigen::MatrixXd::Zero(nu,L1);
        for(unsigned int k=0; k<L1; k++)
	    {

		    for (unsigned int i=0; i<nu; i++)
		    {
			    Xcol(i,0) = Xsig(i,k);	// take out a column so that state_function can take it
		    }	
   	

	        X1col = f(Xcol,dt);
		    for (unsigned int i=0; i<nu; i++)
		    {
			    x1(i,0) +=  Wm(0,k) * X1col(i,0);
		    }
		    for (unsigned int i=0; i<nu; i++)
		    {
			    X1(i,k) = X1col(i,0);	// put back the output column
		    }	
	    }
        //std::cout << x1 << std::endl;
        X2= Eigen::MatrixXd::Zero(NUM_STATES,NUM_STATES*2+1);
        for (unsigned int k=0; k<L1; k++)
		    for (unsigned int i=0; i<nu; i++)
		    {
			   X2(i,k) = X1(i,k) - x1(i,0);	//X2.Column(k) = X1.Column(k) - x1;
		    }
        s.x=x1;
	    diagWm =Eigen::MatrixXd::Zero(L1,L1);

	    for(unsigned int k=0; k<L1; k++)
		    diagWm(k,k) = Wm(0,k);
        
        P12=Eigen::MatrixXd::Zero(NUM_STATES,NUM_MEASUREMENTS);
   
        P1=Eigen::MatrixXd::Zero(NUM_STATES,NUM_STATES);

	    P1= X2 * diagWm * X2.transpose()+ Q_;	/* ~ means transpose */
        xx.P=P1;
        // std::cout << P1 << std::endl;
	    /* unscented transformation (ut) of measurements */
        /* unscented transformation (ut) of measurements */
	    //std::cout << "Q=" <<std::endl;                  //covariance update
	    //std::cout << Q_ << std::endl;
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
        if(debug_){
            printf("[ConstVelUkf::predictX] ||x_new - x_old|| = %f \n", (xx.x - s.x).norm());
            printf("[ConstVelUkf::predictX] det(P) of predicted state: %f \n", xx.P.determinant());
        }

        if(debug_){
            std::cout << "[predictX] old P = \n" << s.P << "\n";
            std::cout << "[predictX] old x = \n" << s.x << "\n";

            std::cout << "[predictX] new P = \n" << xx.P << "\n";
            std::cout << "[predictX] new x = \n" << xx.x << "\n";
        }
        
        return xx;
    }

    /**
    * @brief Update step of a discrete KF using given state x and dt, and measurements z
    * @param kf_state current state (includes time, state, covariance)
    * @return kf_state Predicted state (includes time, state, covariance)
    */
    kf_state
    updateX(sensor_measurement z, kf_state s){
        if(debug_)
            printf("[ConstVelUkf::updateX] Updating x \n");

        kf_state xx;
        xx.time_stamp = z.time_stamp; //z.time_stamp;
        K = P12 * P2.inverse();
        xx.P = P1-K*P12.transpose();
        Eigen::VectorXd y = z.z - h(s.x); // innovation
        xx.x = s.x+ K*(z.z-z1); 
       

        if(debug_){
            printf("[ConstVelUkf::updateX] Done updating state \n");
            printf("[ConstVelUkf::updateX] Norm of innovation y = %f \n", y.norm());
            std::cout << "[ConstVelUkf::updateX] predicted state P: \n" << s.P << "\n";
            // std::cout << "[ConstVelUkf::updateX] innovation covariance S: \n" << S << "\n";
            std::cout << "[ConstVelUkf::updateX] Kalman gain: \n" << K << "\n";
            std::cout << "[ConstVelUkf::updateX] uncorrected state: \n" << s.x << "\n";
            std::cout << "[ConstVelUkf::updateX] measurement: \n" << z.z << "\n";
            std::cout << "[ConstVelUkf::updateX] corrected state: \n" << xx.x << "\n";
        }

        return xx;
    }

    /**
     * dt double time difference between last measurement and current measurement.
     *  This is used to scale the R_ matrix according to measurment's sampling time
    */
    kf_state
    updateX(sensor_measurement z, kf_state s, double dt){
        if(debug_)
            printf("[ConstVelUkf::updateX] Updating x \n");

        kf_state xx;
        xx.time_stamp = z.time_stamp; //z.time_stamp;

        Eigen::VectorXd y = z.z - h(s.x); // innovation
        Eigen::MatrixXd S = H()*s.P*H().transpose() + dt*dt*R_; // innovation covariance
        Eigen::MatrixXd K = K; // optimal Kalman gain
        xx.x = s.x+ K *(z.z-z1); 
        xx.P =  P1-K*P12.transpose();

        if(debug_){
            printf("[ConstVelUkf::updateX] Done updating state \n");
            printf("[ConstVelUkf::updateX] Norm of innovation y = %f \n", y.norm());
            std::cout << "[ConstVelUkf::updateX] predicted state P: \n" << s.P << "\n";
            // std::cout << "[ConstVelUkf::updateX] innovation covariance S: \n" << S << "\n";
            std::cout << "[ConstVelUkf::updateX] Kalman gain: \n" << K << "\n";
            std::cout << "[ConstVelUkf::updateX] uncorrected state: \n" << s.x << "\n";
            std::cout << "[ConstVelUkf::updateX] measurement: \n" << z.z << "\n";
            std::cout << "[ConstVelUkf::updateX] corrected state: \n" << xx.x << "\n";
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
        //     printf("[ConstantAccelModel::logLikelihood] Calculating logLikelihood");
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
            printf("[ConstVelUkf::logLikelihood] Done computing logLikelihood. L= %f \n", LL);

        return LL;
    }

    kf_state
    initStateFromMeasurements(sensor_measurement z){

        if(debug_)
            printf("[ConstVelUkf::initStateFromMeasurements] Initializing state from measurements \n");

        kf_state state;
        state.time_stamp = z.time_stamp;
        state.x = Eigen::MatrixXd::Zero(NUM_STATES,1);
        state.x(0) = z.z(0);
        state.x(1) = z.z(1);
        state.x(2) = z.z(2);

        state.P = Q_;

        return state;
    }

    /**
    * @brief Computes ecluian distance between two states
    */
    double
    computeDistance(kf_state xx, sensor_measurement zz){
        auto y = zz.z - h(xx.x); // assumes length of xx.x = length of zz.z
        auto nrm = y.norm();
        if(debug_)
            printf("[ConstVelUkf::computeDistance] Distance between state and measurement = %f \n", nrm);
        return nrm;
    }

};

#endif //CONSTANT_VEL_UKF_H