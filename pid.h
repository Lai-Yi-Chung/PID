#ifndef PID_H
#define PID_H
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Core>
#include <iostream>

class pid
{
public:
    pid();
    Eigen::VectorXd x, u, B;
    Eigen::MatrixXd A;
    double dt;
    double input;
    double nature_freq, kusi;
    double run();
    void set(double a);
    void reset();


};

#endif // PID_H
