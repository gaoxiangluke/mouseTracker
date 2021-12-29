#include "Optimizer.h"

Optimizer::Optimizer(double w_sigma){
    // g2o releated	
		// set up graph optimization, start with block and linear solver 
		typedef g2o::BlockSolver<g2o::BlockSolverTraits<3, 2>> BlockSolverType;  // every vertex dim is 2, error is 2
  		typedef g2o::LinearSolverDense<BlockSolverType::PoseMatrixType> LinearSolverType; // type of linear solve

		// gradient descent use GaussNewton
  		auto solver = new g2o::OptimizationAlgorithmGaussNewton(
    	g2o::make_unique<BlockSolverType>(g2o::make_unique<LinearSolverType>()));
  		g2o::SparseOptimizer optimizer;     // optimizer
  		_optimizer.setAlgorithm(solver);   // set up solver
  		_optimizer.setVerbose(true);       // enable messages
		_w_sigma = w_sigma;
}
SE2 Optimizer::start_optimizer(const vector<Point>& kalman_pose,const vector<Point>& meas_pose){
    //start graph optimization
			_optimizer.clear();
			//add vertices
			std::vector<StateVertex*> vertices;
			StateVertex* vertex_state = new StateVertex();  // camera vertex_pose
        	vertex_state->setId(0);
			SE2 estimate = SE2();
        	vertex_state->setEstimate(estimate);
        	_optimizer.addVertex(vertex_state);
        	//vertices.push_back(vertex_state);	
			int N =  meas_pose.size();
			for (int i = 0; i < N; i++) {
    			PoseEdge *edge = new PoseEdge(point_to_vector2d(kalman_pose[i]));
    			edge->setId(i);
    			edge->setVertex(0, vertex_state);                // set up connected vertex
    			edge->setMeasurement(point_to_vector2d(meas_pose[i]));      // set measurement 
    			edge->setInformation(Eigen::Matrix<double, 2, 2>::Identity() * 1 / (_w_sigma * _w_sigma)); // information matrix
    			_optimizer.addEdge(edge);
  			}

			// execute Optimization
  			_optimizer.initializeOptimization();
  			_optimizer.optimize(6);
  			// retrun esitimation 
  			SE2 T =  vertex_state->estimate();
  			cout << "estimated model: " << T.matrix()<< endl;
			return T;
}