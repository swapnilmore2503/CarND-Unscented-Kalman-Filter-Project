#include "ukf.h"
#include "Eigen/Dense"
#include <iostream>

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::vector;

/**
 * Initializes Unscented Kalman filter
 * This is scaffolding, do not modify
 */
UKF::UKF() {
    // if this is false, laser measurements will be ignored (except during init)
    use_laser_ = true;

    // if this is false, radar measurements will be ignored (except during init)
    use_radar_ = true;

    // initial state vector
    x_ = VectorXd(5);

    // initial covariance matrix
    P_ = MatrixXd(5, 5);

    // Process noise standard deviation longitudinal acceleration in m/s^2
    std_a_ = 1.5; // Can be tuned

    // Process noise standard deviation yaw acceleration in rad/s^2
    std_yawdd_ = 0.6; // Can be tuned
  
    //DO NOT MODIFY measurement noise values below these are provided by the sensor manufacturer.
    // Laser measurement noise standard deviation position1 in m
    std_laspx_ = 0.15;

    // Laser measurement noise standard deviation position2 in m
    std_laspy_ = 0.15;

    // Radar measurement noise standard deviation radius in m
    std_radr_ = 0.3;

    // Radar measurement noise standard deviation angle in rad
    std_radphi_ = 0.03;

    // Radar measurement noise standard deviation radius change in m/s
    std_radrd_ = 0.3;
  //DO NOT MODIFY measurement noise values above these are provided by the sensor manufacturer.
  
  /**
  TODO:

  Complete the initialization. See ukf.h for other member properties.

  Hint: one or more values initialized above might be wildly off...
  */
    
    // Initialization is False
    is_initialized_ = false;
    
    // State Dimension
    n_x_ = 5;
  
    // Augmented Dimensions
    n_aug_ = 7;

    // Set lambda
    lambda_ = 0;
    
    // Sigma point Matrix
    Xsig_pred_ = MatrixXd(n_x_, 2 * n_aug_ + 1);
    
    // Weights Vectors
    weights_ = VectorXd(2 * n_aug_ + 1);
    
    //Noise Matrices
    R_radar = MatrixXd(3,3);
    R_laser = MatrixXd(2,2);
    
    // Update Noise Matrix - Radar
    R_radar << std_radr_*std_radr_, 0, 0,
               0, std_radphi_*std_radphi_, 0,
               0, 0, std_radrd_*std_radrd_;
    
    // Update Noise Matrix - Laser
    R_laser << std_laspx_*std_laspx_, 0,
               0, std_laspy_*std_laspy_;
    
    // Current Time
    time_us_ = 0;
    
    NIS_radar_ = 0;
    NIS_laser_ = 0;
    
}

UKF::~UKF() {}

/**
 * @param {MeasurementPackage} meas_package The latest measurement data of
 * either radar or laser.
 */
void UKF::ProcessMeasurement(MeasurementPackage meas_package) {
  /**
  TODO:

  Complete this function! Make sure you switch between lidar and radar
  measurements.
  */
    if (!is_initialized_) {
        P_ << 1, 0, 0, 0, 0,
              0, 1, 0, 0, 0,
              0, 0, 1, 0, 0,
              0, 0, 0, 1, 0,
              0, 0, 0, 0, 1;
        if (meas_package.sensor_type_ == MeasurementPackage::RADAR) {
            double rho = meas_package.raw_measurements_(0);
            double phi = meas_package.raw_measurements_(1);
            double rhodot = meas_package.raw_measurements_(2);
            
            // Polar to Cartesian convertion
            /**** Last 3 Values can be tuned ****/
            x_ << rho * cos(phi), rho * sin(phi), rhodot, 0.0, 0.0;
        }
        else if (meas_package.sensor_type_ == MeasurementPackage::LASER) {
            // Set State Initial Values
            /**** Last 3 Values can be tuned ****/
            x_ << meas_package.raw_measurements_(0), meas_package.raw_measurements_(1), 0.0, 0.0, 0.0;
        }
        
        is_initialized_ = true;
        time_us_ = meas_package.timestamp_;
        return;
    }
    
    // delta_t
    double delta_t = (meas_package.timestamp_ - time_us_) / 1000000.0;
    time_us_ = meas_package.timestamp_;
    
    // Predict
    Prediction(delta_t);
    
    // Measurement Updates based on Sensor Type
    if (meas_package.sensor_type_ == MeasurementPackage::RADAR) {
        UpdateRadar(meas_package);
    } else {
        UpdateLidar(meas_package);
    }
}

/**
 * Predicts sigma points, the state, and the state covariance matrix.
 * @param {double} delta_t the change in time (in seconds) between the last
 * measurement and this one.
 */
void UKF::Prediction(double delta_t) {
  /**
  TODO:

  Complete this function! Estimate the object's location. Modify the state
  vector, x_. Predict sigma points, the state, and the state covariance matrix.
  */
    
    // Set lambda
    lambda_ = 3 - n_x_;
    
    // Sigma Point matrix
    MatrixXd Xsig_ = MatrixXd(n_x_, 2 * n_x_ + 1);
    
    // Square root of P
    MatrixXd A_ = P_.llt().matrixL();
    
    // Calculate Sigma points and re-arrange as column vector
    Xsig_.col(0) = x_;
    for (int i = 0; i < n_x_; i++) {
        Xsig_.col(i+1) = x_ + std::sqrt(lambda_ + n_x_) * A_.col(i);
        Xsig_.col(i+1+n_x_) = x_ - std::sqrt(lambda_+n_x_) * A_.col(i);
    }
    
    // Define lambda for augmentation
    lambda_ = 3 - n_aug_;
    
    // Mean Vector
    VectorXd x_aug_ = VectorXd(7);
    
    // Augmented State Covariance Matrix
    MatrixXd P_aug_ = MatrixXd(7, 7);
    
    // Sigma point matrix
    MatrixXd Xsig_aug_ = MatrixXd(n_aug_, 2 * n_aug_ + 1);
    
    // Augmented Mean State
    x_aug_.head(5) = x_;
    x_aug_(5) = 0;
    x_aug_(6) = 0;
    
    // Augmented Covariance Matrix
    MatrixXd Q_ = MatrixXd(2,2);
    Q_ << std_a_*std_a_, 0,
          0, std_yawdd_*std_yawdd_;
    
    P_aug_.fill(0.0);
    P_aug_.topLeftCorner(5, 5) = P_;
    P_aug_.bottomRightCorner(2,2) = Q_;
    
    // Calculate square root of Augmented P
    MatrixXd A_aug = P_aug_.llt().matrixL();
    
    // Augmented Sigma Points
    Xsig_aug_.col(0) = x_aug_;
    for (int i = 0; i < n_aug_; i++) {
        Xsig_aug_.col(i+1) = x_aug_ + std::sqrt(lambda_ + n_aug_) * A_aug.col(i);
        Xsig_aug_.col(i+1+n_aug_) = x_aug_ - std::sqrt(lambda_ + n_aug_) * A_aug.col(i);
    }
    
    // Predict Sigma Points
    VectorXd x1 = VectorXd(5);
    VectorXd x2 = VectorXd(5);
    
    for (int i = 0; i < 2 * n_aug_ + 1; i++) {
        VectorXd Xvec = Xsig_aug_.col(i);
        double px = Xvec(0);
        double py = Xvec(1);
        double v = Xvec(2);
        double yaw = Xvec(3);
        double yawd = Xvec(4);
        double v_aug = Xvec(5);
        double v_yawdd = Xvec(6);
        
        VectorXd xk_ = Xvec.head(5);
        
        // Calculate integral 2nd term
        if (yawd > 0.001) {
            // If the yaw rate is not zero
            x1 << (v/yawd)*(sin(yaw+yawd*delta_t) - sin(yaw)),
                  (v/yawd)*(-cos(yaw+yawd*delta_t) + cos(yaw)),
                  0,
                  yawd * delta_t,
                  0;
        } else {
            // If the yaw rate is zero
            x1 << v*cos(yaw)*delta_t,
                  v*sin(yaw)*delta_t,
                  0,
                  yawd*delta_t,
                  0;
        }
        
        // Calculate CTRV Process Noise Vector
        x2 << .5*delta_t*delta_t*cos(yaw)*v_aug,
              .5*delta_t*delta_t*sin(yaw)*v_aug,
              delta_t*v_aug,
              .5*delta_t*delta_t*v_yawdd,
              delta_t*v_yawdd;
        
        // Predict the sigma points for x_k+1
        Xsig_pred_.col(i) = xk_ + x1 + x2;
    }
    
    // Define Predicted State Vector
    VectorXd x_pred = VectorXd(n_x_);
    
    // Define Covariance Matrix for Prediction
    MatrixXd P_pred = MatrixXd(n_x_, n_x_);
    
    x_pred.fill(0.0);
    P_pred.fill(0.0);
    
    for  (int i = 0; i < 2 * n_aug_ + 1; i++) {
        
        // Calculate Weights
        if (i == 0) {
            weights_(i) = lambda_ / (lambda_ + n_aug_);
        } else {
            weights_(i) = 0.5 / (lambda_ + n_aug_);
        }
        
        // State Mean
        x_pred += weights_(i) * Xsig_pred_.col(i);
    }
    
    // State Covariance Matrix Prediction
    for (int i = 0; i < 2 * n_aug_ + 1; i++) {
        
        VectorXd x_diff = Xsig_pred_.col(i) - x_pred;
        
        // Calibrate angles between -PI to PI
        if (x_diff(3) > M_PI) {
            while (x_diff(3) > M_PI) {x_diff(3) -= 2. * M_PI;}
        } else if (x_diff(3) < -M_PI) {
            while (x_diff(3) < -M_PI) {x_diff(3) += 2. * M_PI;}
        }
        P_pred += weights_(i) * x_diff * x_diff.transpose();
    }
    x_ = x_pred;
    P_ = P_pred;
}

/**
 * Updates the state and the state covariance matrix using a laser measurement.
 * @param {MeasurementPackage} meas_package
 */
void UKF::UpdateLidar(MeasurementPackage meas_package) {
  /**
  TODO:

  Complete this function! Use lidar data to update the belief about the object's
  position. Modify the state vector, x_, and covariance, P_.

  You'll also need to calculate the lidar NIS.
  */
    
    // Measurement dimensions
    int n_z = 2;
    
    // Define Sigma point matrix in measurement space
    MatrixXd Zsig = MatrixXd(n_z, 2 * n_aug_ + 1);
    
    // Mean Predicted Measurement
    VectorXd z_pred = VectorXd(n_z);
    
    // Measurement Covariance Matrix S
    MatrixXd S = MatrixXd(n_z, n_z);
    
    Zsig.fill(0.0);
    z_pred.fill(0.0);
    S.fill(0.0);
    
    for (int i = 0; i < 2 * n_aug_ + 1; i++) {
        // Transform Sigma points to Measurements Space
        VectorXd x_vec = Xsig_pred_.col(i);
        double px = x_vec(0);
        double py = x_vec(1);
        
        Zsig.col(i) << px,
                       py;
        
        // Calculate Mean Predicted Measurement
        z_pred += weights_(i) * Zsig.col(i);
    }
    
    // Calculate Measurement Covariance Matrix S
    for (int i = 0; i < 2 * n_aug_ + 1; i++) {
        VectorXd z_diff = Zsig.col(i) - z_pred;
        S += weights_(i) * z_diff * z_diff.transpose();
    }
    
    // Add the Measurement Noise R to S
    S += R_laser;
    
    // Incoming Radar Measurement
    VectorXd z = VectorXd(n_z);
    
    z << meas_package.raw_measurements_(0),
         meas_package.raw_measurements_(1);
    
    // Cross Correlation Matrix Tc
    MatrixXd Tc = MatrixXd(n_x_, n_z);
    Tc.fill(0.0);
    
    // Calculate Tc
    for (int i = 0; i < 2 * n_aug_ + 1; i++) {
        VectorXd x_diff = Xsig_pred_.col(i) - x_;
        
        // Calibrate angles between -PI to PI
        if (x_diff(3) > M_PI) {
            while (x_diff(3) > M_PI) {x_diff(3) -= 2. * M_PI;}
        } else if (x_diff(3) < -M_PI) {
            while (x_diff(3) < -M_PI) {x_diff(3) += 2. * M_PI;}
        }
        
        VectorXd z_diff = Zsig.col(i) - z_pred;
        
        Tc += weights_(i) * x_diff * z_diff.transpose();
    }
    
    // Calculate Residual
    VectorXd z_diff = z - z_pred;
    
    // Calculate NIS
    NIS_laser_ = z_diff.transpose() * S.inverse() * z_diff;
    
    // Calculate Kalman Gain K
    MatrixXd K = Tc * S.inverse();
    
    // Update State Mean and Covariance Matrix to k+1
    x_ += K * z_diff;
    P_ -= K * S * K.transpose();
}

/**
 * Updates the state and the state covariance matrix using a radar measurement.
 * @param {MeasurementPackage} meas_package
 */
void UKF::UpdateRadar(MeasurementPackage meas_package) {
  /**
  TODO:

  Complete this function! Use radar data to update the belief about the object's
  position. Modify the state vector, x_, and covariance, P_.

  You'll also need to calculate the radar NIS.
  */
    
    // Measurement Dimensions
    int n_z = 3;
    
    // Sigma Point Matrix in Measurement Space
    MatrixXd Zsig = MatrixXd(n_z, 2 * n_aug_ + 1);
    
    // Mean Predicted Measurement
    VectorXd z_pred = VectorXd(n_z);
    
    // Measurement Covariance Matrix S
    MatrixXd S = MatrixXd(n_z, n_z);
    
    Zsig.fill(0.0);
    z_pred.fill(0.0);
    S.fill(0.0);
    double rho = 0;
    double phi = 0;
    double rhodot = 0;
    
    for (int i = 0; i < 2 * n_aug_ + 1; i++) {
        // Transform Sigma Points into Measurement Space
        VectorXd x_vec = Xsig_pred_.col(i);
        double px = x_vec(0);
        double py = x_vec(1);
        double v = x_vec(2);
        double yaw = x_vec(3);
        double yawd = x_vec(4);
        
        rho = sqrt(px*px + py*py);
        phi = atan2(py, px);
        rhodot = (px*cos(yaw)*v+py*sin(yaw)*v) / rho;
        
        Zsig.col(i) << rho,
                       phi,
                       rhodot;
        
        // Calculate Mean Predicted Measurement
        z_pred += weights_(i) * Zsig.col(i);
    }
    
    // Calculate Measurement Covariance Matrix S
    for (int i = 0; i < 2 * n_aug_ + 1; i++) {
        VectorXd z_diff = Zsig.col(i) - z_pred;
        
        // Calibrate angles between -PI to PI
        if (z_diff(1) > M_PI) {
            while (z_diff(1) > M_PI) {z_diff(1) -= 2. * M_PI;}
        } else if (z_diff(1) < -M_PI) {
            while (z_diff(1) < -M_PI) {z_diff(1) += 2. * M_PI;}
        }
        S += weights_(i) * z_diff * z_diff.transpose();
    }
    
    // Add Measurement Noise R to S
    S += R_radar;
    
    // Vector for Radar Meeasurement
    VectorXd z = VectorXd(n_z);
    
    z << meas_package.raw_measurements_(0),
         meas_package.raw_measurements_(1),
         meas_package.raw_measurements_(2);
    
    // Define Cross-Correlation Matrix Tc
    MatrixXd Tc = MatrixXd(n_x_, n_z);
    Tc.fill(0.0);
    
    // Calculate Cross-Correlation Matrix
    for (int i = 0; i < 2 * n_aug_ + 1; i++) {
        
        VectorXd x_diff = Xsig_pred_.col(i) - x_;
        
        // Calibrate angles between -PI to PI
        if (x_diff(3) > M_PI) {
            while (x_diff(3) > M_PI) {x_diff(3) -= 2. * M_PI;}
        } else if (x_diff(3) < -M_PI) {
            while (x_diff(3) < -M_PI) {x_diff(3) += 2. * M_PI;}
        }
        
        VectorXd z_diff = Zsig.col(i) - z_pred;
        
        // Calibrate angles between -PI to PI
        if (z_diff(1) > M_PI) {
            while (z_diff(1) > M_PI) {z_diff(1) -= 2. * M_PI;}
        } else if (z_diff(1) < -M_PI) {
            while (z_diff(1) < -M_PI) {z_diff(1) += 2. * M_PI;}
        }
        
        Tc += weights_(i) * x_diff * z_diff.transpose();
    }
    
    // Residual
    VectorXd z_diff = z - z_pred;
    
    // Calibrate angles between -PI to PI
    if (z_diff(1) > M_PI) {
        while (z_diff(1) > M_PI) {z_diff(1) -= 2. * M_PI;}
    } else if (z_diff(1) < -M_PI) {
        while (z_diff(1) < -M_PI) {z_diff(1) += 2. * M_PI;}
    }
    
    // Calculate NIS
    NIS_radar_ = z_diff.transpose() * S.inverse() * z_diff;
    
    // Calculate Kalman Gain K
    MatrixXd K = Tc * S.inverse();
    
    // Update State Mean and Covariance Matrix
    x_ += K * z_diff;
    P_ -= K * S * K.transpose();
    
}
