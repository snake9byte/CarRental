#ifndef KALMAN_FILTER_HPP
#define KALMAN_FILTER_HPP
#include <iostream>
#include "matrix_lib.hpp"

//!!! General comment: fewer return statements, work more with member variables

namespace KalmanFilter{

  class KalmanFilter {
    public:
      /*
      * Constructor call for a kalman filter
      * n : dimension of the state
      * m : dimension of the measurement
      * k : dimension of control   NOTE: will not be used in this implementation
      */
      KalmanFilter(uint8_t n, uint8_t m, uint8_t k = 0);

      /*
      * Constructor call for a kalman filter
      * x : state
      * A : Transition Matrix
      * P : Covariance
      * Q : Random uncertainty covariacne
      * H : Measurement Matrix
      * R : Sensor Noise Covariance
      */
      void init(Vector x, Matrix A, Matrix P, Matrix Q, Matrix H, Matrix R);

      // set initial state and covariance
      void set_initial(Vector init);  //! Add initial covariance matrix

      // get state
      Vector get_state();

      // get covariacne
      Matrix get_covariance();

      // filter using 5 KF equations
      /*
      * dt : Time interval
      * s  : state
      * z  : sensor measurement vector
      */
      void filter(float dt, Vector s, Vector z);  //!! Two variables missing, update this! (real life values) - fixed

    public:
      // f1 : predict state
      void predict_state();
      // f2 : estimate state (update based on measurement)
      void estimate_state(Matrix K, Vector z);
      // f3 : kalman gain
      Matrix kalman_gain();
      // f4 : predict state covariance
      void predict_covariance();
      // f5 : estimate state covariance
      void estimate_covariance(Matrix K);
      // f10 : update state transition;
      void update_state_transition(float Dt);  //! lowercase d
      // f11 : update state transition;
      void update_sensor_noise(Matrix R); // Find out - how to update R?
      // f12 : get data from sensors (may have parameters according to sensors spec)
      //       Will set the measurement vector
      // Vector get_data();
      // f13 : get measurement
      void set_measurement(Vector z);
      // f14 : get measurement
      Vector get_measurement();

      // these guys throw a warning. On the Hyped 2019 github code, they dont. Need to further invastigate why...
      static constexpr float kStateCovarianceNoise = 0.01;
      static constexpr float kMeasurementNoise = 0.01;

      uint8_t n_;
      uint8_t m_;
      uint8_t k_;

      Vector x_;  // current state (n x 1)
      Vector z_;  // measurement (m x 1)
      Matrix A_;  // state transition matrix (n x n)
      Matrix P_;  // state covariance matrix (n x n)
      Matrix Q_;  // state covariance noise matrix (n x n)
      Matrix H_;  // dimension change matrix (m x n)
      Matrix R_;  // measurement noise matrix (m x m)
      Matrix I_;  // identity matrix (n x n)
      Matrix J_;  // anti-diagnonal matrix (J)

  };

}
#endif  // KALMAN_FILTER_HPP
