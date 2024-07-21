#include <iostream>
#include "kalman_filter.hpp"

namespace KalmanFilter{

    void KalmanFilter::init(Vector x, Matrix A, Matrix P, Matrix Q, Matrix H, Matrix R){
        x_ = x;
        A_ = A;
        P_ = P;
        Q_ = Q;
        H_ = H;
        R_ = R;

        Matrix I(n_,n_);

        //creating the init matrix. Find a way to add this to the matrix_lib as a method and not allow the user to assign a new value to it.
        
        vector <float> identity;
        for(int row = 0; row < n_; row++){
          for(int clm = 0; clm < n_; clm++){
            if (clm == row){
              identity.push_back(1);
            }
            else{
              identity.push_back(0);
            }
          }
        }

        I.set_matrix(identity);

        Matrix J(n_,n_);

        //creating the init matrix. Find a way to add this to the matrix_lib as a method and not allow the user to assign a new value to it.
        
        vector <float> anti_diagnonal;
        for(int row = 0; row < n_; row++){
          for(int clm = 0; clm < n_; clm++){
            if (n_ - 1 - clm == row){
              anti_diagnonal.push_back(1);
            }
            else{
              anti_diagnonal.push_back(0);
            }
          }
        }

        J.set_matrix(anti_diagnonal);

        J_ = J;

        std::cout << J_;

    }

    KalmanFilter::KalmanFilter(uint8_t n, uint8_t m, uint8_t k)
        : n_(n)
        , m_(m)
        , k_(k){
    }

    

    void KalmanFilter::set_initial(Vector init){
        x_ =  init;
    }

    Vector KalmanFilter::get_state(){
        return x_;
    }

    Matrix KalmanFilter::get_covariance(){
        return P_;
    }

    Vector KalmanFilter::get_measurement(){
        return z_;
    }

    void KalmanFilter::update_sensor_noise(Matrix R){

        //some sort of calclulation takes place here.

        R_ = R;
    }

    void KalmanFilter::filter(float dt, Vector s, Vector z){
        
        KalmanFilter::update_state_transition(dt);

        KalmanFilter::predict_state();
        KalmanFilter::predict_covariance();

        Matrix K(n_, n_);
        K = KalmanFilter::kalman_gain();

        // KalmanFilter::get_data(); // manipulate sensor data and set measurement vector

        KalmanFilter::estimate_state(K,z);
        
        KalmanFilter::estimate_covariance(K);

    }

    void KalmanFilter::predict_state(){
        x_ = A_ * x_;
    }
    
    void KalmanFilter::predict_covariance(){
        P_ = A_ * P_ * A_.transpose() + Q_;
    }

    Matrix KalmanFilter::kalman_gain(){
        Matrix K(n_, n_);
        K = (P_ * H_.transpose()) * (H_ * P_ * H_.transpose() + R_).inverse();
        return K;
    }

    void KalmanFilter::estimate_state(Matrix K, Vector z){
        // z = {ax,ay,az}
        // z = {xx,xy,xz, vx,vy,vz, ax,ay,az}
        x_ = x_ + K * (z - H_ * x_);
    }

    void KalmanFilter::estimate_covariance(Matrix K){
        P_ = (I_ - K * H_) * P_;
    }

    void KalmanFilter::update_state_transition(float dt){
        Matrix A(n_, n_);

        Vector s (3);
        s = {1.0, dt, dt*dt/2}; // {displacement, velocity, acceleration placeholders}

        for(int i = 0; i < n_; i++)
        {
            for(int j = i; j < n_; j++)
            {
                if (j - i % m_ == 0){
                    A.entries_[j + i*n_] = s.entries_[(j - i)/m_];
                }
            }
        }
        A_ = A;
    }

    // VectorXf KalmanFilter::get_data(){

    //     Eigen::Vector3f z(0.0,0.0,0.0); // placeholder

    //     // read data from sensor
    //     // manipulate data - make any calculations needed
    //     // store data in a VectorXf format

    //     // output data
    //     set_measurement(z);

    //     return z;
    // }

    void KalmanFilter::set_measurement(Vector z){
        z_ = z;
    }

}




// #endif  // KALMAN_FILTER_HPP