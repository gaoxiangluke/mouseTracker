#include "common_include.h"
#include "utility.h"
#include "Optimizer.h"
#include "MouseFilter.h"

int main()
{

	//set up random noise 
	cv::RNG rng;
	double w_sigma = 1.44;

	//initialize Optimizer
	Optimizer optimizer;
	//initilize kf
	MouseFilter kf;
	unsigned int type = CV_32F;
	// creating state vector
	cv::Mat measurement_true(2, 1, type);    // [z_x, z_y] 
	cv::Mat measurement_noisy(2, 1, type); 


	Mat display_image(600, 800, CV_8UC3);
	namedWindow("Predict Mouse trajectory");

	Point mousePos;
	vector<Point> measurement_p, kalman_p,graph_p,true_p;


	while (1)
	{
		
		//read from mouse input as measurement
		setMouseCallback("Predict Mouse trajectory", on_mouse, &mousePos);
		
		//passing measurement and add some noise
		measurement_true.at<float>(0) = mousePos.x;
		measurement_true.at<float>(1) = mousePos.y;
		measurement_noisy.at<float>(0) = mousePos.x + rng.gaussian(w_sigma*w_sigma);
		measurement_noisy.at<float>(1) = mousePos.y + rng.gaussian(w_sigma*w_sigma);

		Mat estimated = kf.prediction(measurement_noisy);

		Point statePt(estimated.at<float>(0), estimated.at<float>(1));
		Point measPt(measurement_noisy.at<float>(0),measurement_noisy.at<float>(1));
		Point truePt(measurement_true.at<float>(0), measurement_true.at<float>(1));

		cv::imshow("Predict Mouse trajectory", display_image);
		display_image = Scalar::all(0);
		measurement_p.push_back(measPt);
		kalman_p.push_back(statePt);
		graph_p.push_back(statePt);
		true_p.push_back(truePt);

		Point graphPt = graph_p.back();
		drawCross(display_image,statePt, Scalar(255, 255, 255), 5);
		drawCross(display_image,measPt, Scalar(0, 0, 255), 5);
		drawCross(display_image,graphPt, Scalar(0, 255, 0), 5);
		drawCross(display_image,truePt, Scalar(0, 255, 255), 5);

		for (int i = 0; i < measurement_p.size() - 1; i++)
			line(display_image, measurement_p[i], measurement_p[i + 1], Scalar(0, 0, 255), 1);
		for (int i = 0; i < true_p.size() - 1; i++)
			line(display_image, true_p[i], true_p[i + 1], Scalar(0, 255, 255), 1);
		for (int i = 0; i < graph_p.size() - 1; i++)
			line(display_image, graph_p[i], graph_p[i + 1], Scalar(0, 255, 0), 1);
		for (int i = 0; i < kalman_p.size() - 1; i++)
			line(display_image, kalman_p[i], kalman_p[i + 1], Scalar(255, 0, 0), 1);
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
			SE2 T = optimizer.start_optimizer(kalman_p,measurement_p);
			//add edges
			graph_p = kalman_p;
			for (int i = 0; i < graph_p.size(); i++){
				Eigen::Vector2d point = point_to_vector2d(graph_p[i]);
				point = T*point;
				graph_p[i].x = point[0];
				graph_p[i].y = point[1];
			}

		}
	}
}
