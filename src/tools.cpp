#include <iostream>
#include "tools.h"

using Eigen::VectorXd;
using Eigen::MatrixXd;
using std::vector;
using namespace std;

Tools::Tools() {}

Tools::~Tools() {}

VectorXd Tools::CalculateRMSE(const vector<VectorXd> &estimations,
                              const vector<VectorXd> &ground_truth) {

    VectorXd rmse(4);
    rmse << 0, 0, 0, 0;

    if (estimations.size() != ground_truth.size()
        || estimations.size() == 0) {
        cout << "Invalid estimation or ground_truth data" << endl;
        return rmse;
    }

    //accumulate squared residuals
    for (unsigned int i = 0; i < estimations.size(); ++i) {

        VectorXd residual = estimations[i] - ground_truth[i];

        //coefficient-wise multiplication
        residual = residual.array() * residual.array();
        rmse += residual;
    }

    //calculate the mean
    rmse = rmse / estimations.size();

    //calculate the squared root
    rmse = rmse.array().sqrt();

    //return the result
    return rmse;


}

MatrixXd Tools::CalculateJacobian(const VectorXd &x_state) {
    // initialize Jacobian matrix
    MatrixXd Hj(3, 4);
    float px = x_state(0);
    float py = x_state(1);
    float vx = x_state(2);
    float vy = x_state(3);

    // First we compute some terms for later use
    float t1 = px * px + py * py;
    float t2 = sqrt(t1);
    float t3 = t2 * t2;
    float t4 = vx * py - vy * px;

    // check division by zero exception
    if (fabs(t1) < 0.0001) {
        cout << "CalculateJacobian() division by Zero " << endl;
        return Hj;
    }

    Hj << px / t2, py / t2, 0, 0,
            -py / t1, px / t1, 0, 0,
            py * (t4) / t3, px * (-1 * t4) / t3, px / t2, py / t2;
    return Hj;
}
