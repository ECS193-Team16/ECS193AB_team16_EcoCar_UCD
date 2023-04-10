/* \author Aaron Brown */
// Create simple 3d highway enviroment using PCL
// for exploring self-driving car sensors

#include "Eigen/Dense"
#include <iostream>

#include "types.h"
#include "../ukf.h"

using Eigen::VectorXd;

VectorXd getEstimate(UKF ukf){
	VectorXd estimate(4);
	double v  = ukf.x_(2);
	double yaw = ukf.x_(3);
	estimate <<
		ukf.x_[0],
		ukf.x_[1],
		cos(yaw)*v,
		sin(yaw)*v;
	return estimate;
}

MeasurementPackage meas_package_radar(
	double rho, double phi, double rho_dot,
	long long timestamp
){
	rmarker marker = rmarker(rho, phi, rho_dot);

	MeasurementPackage meas_package;
	meas_package.sensor_type_ = MeasurementPackage::RADAR;
    meas_package.raw_measurements_ = VectorXd(3);
    meas_package.raw_measurements_ << marker.rho, marker.phi, marker.rho_dot;
    meas_package.timestamp_ = timestamp;

	return meas_package;
}

int main(int argc, char** argv)
{
	UKF ukf;

	double rho, phi, rho_dot;
	long long timestamp;

	while (1){

		std::cin >> rho >> phi >> rho_dot >> timestamp;
		meas_package_radar(rho, phi, rho_dot, timestamp);

		VectorXd estimate= getEstimate(ukf);

        std::cout << estimate.transpose().format(Eigen::IOFormat(Eigen::FullPrecision, Eigen::DontAlignCols, " ", " ")) << std::endl;
	}

}


