#ifndef B25EECF3_5685_48A4_BC2F_BDAF44F6404C
#define B25EECF3_5685_48A4_BC2F_BDAF44F6404C
class MouseFilter{
public:
	MouseFilter(double dt = 2,double w_sigma = 1.44,unsigned int type = CV_32F){
		//set up KF releated state
		// initiation of OpenCV Kalman Filter
	 	// [x, y, d_x, d_y] state size 4
		// [z_x, z_y] // measurement size 2. we will only measure mouse cursor x and y
		//no control
		_KF = cv::KalmanFilter(4, 2, 0, type);
		// X_k+1 = A*X_k
		// A =
		// [1 0 1 0]
		// [0 1 0 1]
		// [0 0 1  0]
		// [0 0 0  1]
		cv::setIdentity(_KF.transitionMatrix);
		_KF.transitionMatrix.at<float>(0,2) = 1;
    	_KF.transitionMatrix.at<float>(1,3) = 1;
		setIdentity(_KF.measurementMatrix); //C
		setIdentity(_KF.processNoiseCov, Scalar::all(1e-4)); //R
		setIdentity(_KF.measurementNoiseCov, Scalar::all(w_sigma*w_sigma)); //Q
		setIdentity(_KF.errorCovPost, Scalar::all(.1)); //P
	}
	cv::Mat prediction(cv::Mat& measurement){
		//first predict current state
		cv::Mat state(4, 1, CV_32F);  // [x, y, d_x, d_y] 
		state = _KF.predict();
		// then update with measurement
		Mat estimated = _KF.correct(measurement);

		return estimated;
	}
public:
	cv::KalmanFilter _KF;
	double _w_sigma;
	//state vector [x,y,d_x,d_y]'
	double dt;

};

#endif /* B25EECF3_5685_48A4_BC2F_BDAF44F6404C */
