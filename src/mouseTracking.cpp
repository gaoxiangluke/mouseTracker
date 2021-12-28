#include "opencv2/video/tracking.hpp"
#include <iostream>
#include <stdio.h>
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <g2o/core/g2o_core_api.h>
#include <g2o/core/base_vertex.h>
#include <g2o/core/base_unary_edge.h>
#include <g2o/core/block_solver.h>
#include <g2o/core/optimization_algorithm_levenberg.h>
#include <g2o/core/optimization_algorithm_gauss_newton.h>
#include <g2o/core/optimization_algorithm_dogleg.h>
#include <g2o/solvers/dense/linear_solver_dense.h>
#include <Eigen/Core>
#include <cmath>
#include <chrono>
using namespace cv;
using namespace std;

// state vertex x,y
class StateVertex : public g2o::BaseVertex<2,Eigen::Vector2d > {
   public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

    virtual void setToOriginImpl() override {
    _estimate << 0, 0;
  }

    /// left multiplication on SE3
    virtual void oplusImpl(const double *update) override {
       _estimate += Eigen::Vector2d(update);
    }

    virtual bool read(std::istream &in) override { return true; }

    virtual bool write(std::ostream &out) const override { return true; }
};


// 误差模型 模板参数：观测值维度，类型，连接顶点类型
class PoseEdge : public g2o::BaseUnaryEdge<2,Eigen::Vector2d,StateVertex> {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  PoseEdge(Eigen::Vector2d pos) :_pos2d(pos){}

  // 计算曲线模型误差
  virtual void computeError() override {
    const  StateVertex *v = static_cast<const  StateVertex *> (_vertices[0]);
    const Eigen::Vector2d dT = v->estimate();
	Eigen::Vector2d current;
	current[0] = _pos2d[0] + dT[0];
	current[1] = _pos2d[1] + dT[1];
    //_error(0, 0) = pow(_measurement[0] - current[0],2) + pow(_measurement[1] - current[1],2);
	_error(0, 0)= abs(_measurement[0] - current[0]);
	_error(1, 0)= abs(_measurement[1] - current[1]);
  }
  virtual bool read(istream &in) {}

  virtual bool write(ostream &out) const {}

public:
  Eigen::Vector2d _pos2d;  // x 值， y 值为 _measurement
};

inline void drawCross(Mat display_image,Point center, Scalar color, int d )
{
   line( display_image, Point( center.x - d, center.y - d ), Point( center.x + d, center.y + d ), color, 2, LINE_AA, 0);
   line( display_image, Point( center.x + d, center.y - d ), Point( center.x - d, center.y + d ), color, 2, LINE_AA, 0 );
}  


void on_mouse(int e, int x, int y, int d, void *ptr)
{
	Point*p = (Point*)ptr;
	p->x = x;
	p->y = y;
}
Eigen::Vector2d point_to_vector2d(Point p){
	Eigen::Vector2d result;
	result[0] = p.x;
	result[1] = p.y;
	return result;
}

int main()
{
	// >>> Kalman Filter Initialization
	int stateSize = 4;  // [x, y, d_x, d_y]
	int measSize = 2;   // [z_x, z_y] // we will only measure mouse cursor x and y
	int contrSize = 0;  // no control input1

	unsigned int F_type = CV_32F;

	// initiation of OpenCV Kalman Filter
	cv::KalmanFilter KF(stateSize, measSize, contrSize, F_type);

	// creating state vector
	cv::Mat state(stateSize, 1, F_type);  // [x, y, d_x, d_y] // column Matrix

										  // creating measurement vector
	cv::Mat meas_true(measSize, 1, F_type);    // [z_x, z_y] // column matrix

										  // Transition state matrix A
										  // Note: set dT at each processing step!
										  // X_k = A*X_k-1
										  // X_k = current state := x_k, y_k, v_x_k
										  // X_k-1 = previous state
										  // A =
										  // [1 0 1 0]
										  // [0 1 0 1]
										  // [0 0 1  0]
										  // [0 0 0  1]
										  // observe it is an identity matrix with dT inputs that we will provide later
	cv::Mat meas(measSize, 1, F_type); 

	cv::setIdentity(KF.transitionMatrix);
    double dt = 2;
    KF.transitionMatrix.at<float>(0,2) = 1;
    KF.transitionMatrix.at<float>(1,3) = 1;

	//set up random noise 
	cv::RNG rng;
	// Measurement Matrix (This is C or H matrix)
	// size of C is measSize x stateSize
	// only those values will set which we can get as measurement in a state vector
	// here out of [x, y, v_x and v_y] we can only measure x, y of the mouse cursor coordianates
	// so we set only element "0" and "5".
	// [1 0 0 0]
	// [0 1 0 0]

	// Process Noise Covariance Matrix := stateSize x stateSize
	//  [Ex 0  0    0]
	//  [0 Ey  0    0]
	//  [0 0 E_v_x  0]
	//  [0 0  0  E_v_y]

	double w_sigma = 1.44;
	setIdentity(KF.measurementMatrix);
	setIdentity(KF.processNoiseCov, Scalar::all(1e-4));
	setIdentity(KF.measurementNoiseCov, Scalar::all(w_sigma*w_sigma));
	setIdentity(KF.errorCovPost, Scalar::all(.1));

	// Measure Noise Covariance Matrix
	//cv::setIdentity(KF.measurementNoiseCov, cv::Scalar(1e-1));

	// <<< Kalman Filter initializationOnThread

	Mat display_image(600, 800, CV_8UC3);
	namedWindow("Mouse Kalman with Velocity");

	char ch = 0;
	double ticks = 0;
	Point mousePos;
	vector<Point> mousev, kalmanv,graphv,truev;
	


	// g2o releated	
	// set up graph optimization, start with block and linear solver 
	typedef g2o::BlockSolver<g2o::BlockSolverTraits<2, 2>> BlockSolverType;  // every vertex dim is 2, error is 2
  	typedef g2o::LinearSolverDense<BlockSolverType::PoseMatrixType> LinearSolverType; // type of linear solve

	// gradient descent use GaussNewton
  	auto solver = new g2o::OptimizationAlgorithmGaussNewton(
    g2o::make_unique<BlockSolverType>(g2o::make_unique<LinearSolverType>()));
  	g2o::SparseOptimizer optimizer;     // optimizer
  	optimizer.setAlgorithm(solver);   // set up solver
  	optimizer.setVerbose(true);       // enable messages






	while (ch != 'q' && ch != 'Q')
	{
		state = KF.predict(); // First predict, to update the internal statePre variable
		//std::cout << "State post: " << state << std::endl;

		Point predictPt(state.at<float>(0), state.at<float>(1));
		// <<< Kalman Prediction

		// >>> Get Mouse Point
		setMouseCallback("Mouse Kalman with Velocity", on_mouse, &mousePos);
		//std::cout << "Mouse Position is: " << mousePos << std::endl;
		// <<< Get Mouse Point

		// >>> Passing the measured values to the measurement vector with some noise 
		meas_true.at<float>(0) = mousePos.x;
		meas_true.at<float>(1) = mousePos.y;
		meas.at<float>(0) = mousePos.x + rng.gaussian(w_sigma*w_sigma);
		meas.at<float>(1) = mousePos.y + rng.gaussian(w_sigma*w_sigma);
		// >>> Kalman Update Phase
		Mat estimated = KF.correct(meas);

		Point statePt(estimated.at<float>(0), estimated.at<float>(1));
		Point measPt(meas.at<float>(0), meas.at<float>(1));
		Point truePt(meas_true.at<float>(0), meas_true.at<float>(1));
		// <<< Kalman Update Phase

		cv::imshow("Mouse Kalman with Velocity", display_image);
		display_image = Scalar::all(0);
		mousev.push_back(measPt);
		kalmanv.push_back(statePt);
		graphv.push_back(statePt);
		truev.push_back(truePt);

		Point graphPt = graphv.back();
		drawCross(display_image,statePt, Scalar(255, 255, 255), 5);
		drawCross(display_image,measPt, Scalar(0, 0, 255), 5);
		drawCross(display_image,graphPt, Scalar(0, 255, 0), 5);
		drawCross(display_image,truePt, Scalar(0, 255, 255), 5);

		for (int i = 0; i < mousev.size() - 1; i++)
			line(display_image, mousev[i], mousev[i + 1], Scalar(0, 0, 255), 1);
		for (int i = 0; i < truev.size() - 1; i++)
			line(display_image, truev[i], truev[i + 1], Scalar(0, 255, 255), 1);
		for (int i = 0; i < kalmanv.size() - 1; i++)
			line(display_image, kalmanv[i], kalmanv[i + 1], Scalar(255, 0, 0), 1);
		for (int i = 0; i < graphv.size() - 1; i++)
			line(display_image, graphv[i], graphv[i + 1], Scalar(0, 255, 0), 1);
        cv::putText(display_image, //target image
            "Prediction-KF", //text
            cv::Point(10, 20), //top-left position
            cv::FONT_HERSHEY_DUPLEX,
            1,
            CV_RGB(255, 0, 0), //font color
            1);
         cv::putText(display_image, //target image
            "Noisy Measurement", //text
            cv::Point(10, 50), //top-left position
            cv::FONT_HERSHEY_DUPLEX,
            1,
            CV_RGB(0, 0, 255), //font color
            1);
		 cv::putText(display_image, //target image
            "optimized graph", //text
            cv::Point(10, 100), //top-left position
            cv::FONT_HERSHEY_DUPLEX,
            1,
            CV_RGB(0, 255, 0), //font color
            1);
		char ch = cv::waitKey(10);
		if(ch == 27){
            cout <<"end program" << endl;
            return 0; 
        }else if (ch == 103){
			//start graph optimization
			optimizer.clear();
			//add vertices
			std::vector<StateVertex*> vertices;
			// for(int index = 0; index < graphv.size(); index ++ ){
			// 	StateVertex* vertex_state = new StateVertex();  // camera vertex_pose
        	// 	vertex_state->setId(index);
			// 	Eigen::Vector2d estimate = {0,0};
        	// 	vertex_state->setEstimate(estimate);
        	// 	optimizer.addVertex(vertex_state);
        	// 	vertices.push_back(vertex_state);	
			// }
			StateVertex* vertex_state = new StateVertex();  // camera vertex_pose
        	vertex_state->setId(0);
			Eigen::Vector2d estimate = {0,0};
        	vertex_state->setEstimate(estimate);
        	optimizer.addVertex(vertex_state);
        	//vertices.push_back(vertex_state);	
			int N = mousev.size();
			for (int i = 0; i < N; i++) {
    			PoseEdge *edge = new PoseEdge(point_to_vector2d(kalmanv[i]));
    			edge->setId(i);
    			edge->setVertex(0, vertex_state);                // 设置连接的顶点
    			edge->setMeasurement(point_to_vector2d(mousev[i]));      // 观测数值
    			edge->setInformation(Eigen::Matrix<double, 2, 2>::Identity() * 1 / (w_sigma * w_sigma)); // 信息矩阵：协方差矩阵之逆
    			optimizer.addEdge(edge);
  			}

			// 执行优化
  			cout << "start optimization" << endl;
  			chrono::steady_clock::time_point t1 = chrono::steady_clock::now();
  			optimizer.initializeOptimization();
  			optimizer.optimize(10);
  			chrono::steady_clock::time_point t2 = chrono::steady_clock::now();
  			chrono::duration<double> time_used = chrono::duration_cast<chrono::duration<double>>(t2 - t1);
  			cout << "solve time cost = " << time_used.count() << " seconds. " << endl;

  			// 输出优化值
  			Eigen::Vector2d displacement_estimate =  vertex_state->estimate();
  			cout << "estimated model: " << displacement_estimate.transpose() << endl;
			//add edges
			graphv = kalmanv;
			for (int i = 0; i < graphv.size(); i++){
				graphv[i].x += displacement_estimate[0];
				graphv[i].y += displacement_estimate[1];
			}
				
				
		}

	}
}
