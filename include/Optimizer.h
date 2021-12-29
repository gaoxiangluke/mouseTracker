#ifndef E09B6F1C_BD8B_4143_A73E_501AA2F2108C
#define E09B6F1C_BD8B_4143_A73E_501AA2F2108C
#include "common_include.h"
#include "g2o_header.h"
#include "utility.h"
using namespace std;
class Optimizer{
public:
	Optimizer(double w_sigma = 1.44);
	SE2 start_optimizer(const vector<Point>& kalman_pose,const vector<Point>& meas_pose);
public:
	g2o::SparseOptimizer _optimizer;
	double _w_sigma;
};

#endif /* E09B6F1C_BD8B_4143_A73E_501AA2F2108C */
