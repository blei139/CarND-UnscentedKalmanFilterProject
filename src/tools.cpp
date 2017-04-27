#include <iostream>
#include "tools.h"

using Eigen::VectorXd;
using Eigen::MatrixXd;
using std::vector;

Tools::Tools() {}

Tools::~Tools() {}

VectorXd Tools::CalculateRMSE(const vector<VectorXd> &estimations,
                              const vector<VectorXd> &ground_truth) {

	VectorXd rmse(4);
	rmse << 0,0,0,0;

  /**
  TODO:
    * Calculate the RMSE here.
  */
	// check the validity of the following inputs:
	//  * the estimation vector size should not be zero
	if (estimations.size() == 0) {
	   	std::cout << "RMSE - Error - estimation vector is Zero" << std::endl;
		return rmse;
	}
	//  * the estimation vector size should equal ground truth vector size
	// ... your code here
    	else if (estimations.size() != ground_truth.size()) {
        		std::cout << "RMSE - Error - estimation size is equal to ground truch size" << std::endl;
    	}
	//accumulate squared residuals
	for(int i=0; i < estimations.size(); ++i){
        		// ... your code here
        		VectorXd res = estimations[i] - ground_truth[i];
        		//coefficient-wise multiplication
		res = res.array()*res.array();
		rmse += res;
		
	}


	//calculate the mean
	// ... your code here
    	int n = estimations.size();
    	rmse = rmse/n;
    
	//calculate the squared root
	// ... your code here
    	rmse = rmse.array().sqrt();
	//return the result
	return rmse;

}

