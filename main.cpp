/*#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui.hpp>
#include <iostream>
using namespace cv;*/
/////image///////
/*int main()
{
    cv::Mat img = cv::imread("C:/Users/sharm/OneDrive/Desktop/adriuno/codeing open cv/setup/resource/harshit.jpg");
    namedWindow("First OpenCV Application", WINDOW_AUTOSIZE);
    cv::imshow("First OpenCV Application", img);
    cv::moveWindow("First OpenCV Application", 0, 45);
    cv::waitKey(0);
    cv::destroyAllWindows();



    return 0;
}*/
//////////////////////video/////////////////////
/*
int main() {
	cv::VideoCapture cap("C:/Users/sharm/OneDrive/Desktop/adriuno/codeing open cv/setup/resource/vid.mp4");
	if (cap.isOpened() == false) {
		std::cout << "Cannot open the video file" << std::endl;
		std::cin.get(); //wait for any key press
		return -1;
	}
	double fps = cap.get(cv::CAP_PROP_FPS);
	cv::String window_name = "My First Video";
	namedWindow(window_name, cv::WINDOW_NORMAL);
	while (true) {
		cv::Mat frame;
		bool bSuccess = cap.read(frame);
		if (bSuccess == false) {
			std::cout << "The video has ended" << std::endl;
			break;
		}
		cv::imshow(window_name, frame);
		if (cv::waitKey(10) == 27) {
			std::cout << "Esc key is pressed by the user. Stopping the video" << std::endl;
			break;
		}
	}
	return 0;
}*/
///////////Webcam////////////////
/*
int main() {
	cv::VideoCapture cap(0); //ID number 0->one camera, 1->second camera
	if (cap.isOpened() == false) {
		std::cout << "Cannot open the video camera" << std::endl;
		std::cin.get(); //wait for any key press
		return -1;
	}
	double dWidth = cap.get(cv::CAP_PROP_FRAME_WIDTH);
	double dHeight = cap.get(cv::CAP_PROP_FRAME_HEIGHT);
	std::cout << "Resolution of the video : " << dWidth << " x " << dHeight << std::endl;
	cv::String window_name = "My Camera Feed";
	namedWindow(window_name, cv::WINDOW_NORMAL);
	while (true) {
		cv::Mat frame;
		bool bSuccess = cap.read(frame);
		if (bSuccess == false) {
			std::cout << "The camera is disconnected" << std::endl;
			std::cin.get(); //Wait for any key press
			break;
		}
		cv::imshow(window_name, frame);
		if (cv::waitKey(10) == 27) {
			std::cout << "Esc key is pressed by the user. Stopping the video" << std::endl;
			break;
		}
	}
	return 0;
}
*/
//chapter 2//
////iamge/////
/*

#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui.hpp>
#include <iostream>
using namespace cv;
int main() {
	cv::Mat img = cv::imread("C:/Users/sharm/OneDrive/Desktop/adriuno/codeing open cv/setup/resource/harshit.jpg");
	if (img.empty()) {
		std::cout << "Could not read the image: harshit.jpg" << std::endl;
		return 1;
	}

	cv::Mat imgGray, imgBlur, imgCanny;
	cv::cvtColor(img, imgGray, cv::COLOR_BGR2GRAY); // Use COLOR_BGR2GRAY for standard grayscale conversion
	if (imgGray.empty()) {
		std::cout << "Color conversion failed." << std::endl;
		return 1;
	}

	cv::GaussianBlur(imgGray, imgBlur, cv::Size(3, 3), 5, 0);
	if (imgBlur.empty()) {
		std::cout << "GaussianBlur failed." << std::endl;
		return 1;
	}

	cv::Canny(imgBlur, imgCanny, 10, 10); // Adjust thresholds as needed
	if (imgCanny.empty()) {
		std::cout << "CannyFail" << std::endl;
	}
	//cv::imshow("Original Image", img);
	//cv::imshow("Grayscale Image", imgGray);
	//cv::imshow("Blurred Image", imgBlur);
	cv::imshow("Canny Edge Detected", imgCanny);
	waitKey(0);

	return 0;
}
*/

/**

#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui.hpp>
#include <iostream>
using namespace cv;
int main() {
	cv::Mat img = cv::imread("C:/Users/sharm/OneDrive/Desktop/adriuno/codeing open cv/setup/resource/harshit.jpg");
	if (img.empty()) {
		std::cout << "Could not read the image: harshit.jpg" << std::endl;
		return 1;
	}
	Mat resize1;
	std::cout << img.size() << std::endl;
	//resize(img, resize1, cv::Size(640, 480));
	//imshow("resize img", resize1);
	//Crop of img

	Mat crop = img(cv::Rect(100, 100, 300, 300));
	imshow("crop img", crop);
	waitKey(0);
	return 0;
}
*/

//chapter 4//
//Drawing shapes on image//
/*
#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui.hpp>
#include <iostream>
int main() {
	//make img//
	cv::Mat img(512, 512, CV_8UC3, cv::Scalar(255, 0, 255));//Cv 8 bit unsigned char 3 channel(BGR)//
	//Draw a circle//
	cv::circle(img, cv::Point(256, 256), 155, cv::Scalar(0, 69, 255), cv::FILLED);//FILLED or THICKNESS//
	cv::rectangle(img, cv::Point(130, 226), cv::Point(382, 326), cv::Scalar(0, 255, 0), 2); // Thickness of 2 pixels
	cv::line(img, cv::Point(130, 226), cv::Point(382, 326), cv::Scalar(255, 255, 255), 2);
	cv::putText(img, "Workshop practice", cv::Point(137, 262), cv::FONT_HERSHEY_DUPLEX, 1, cv::Scalar(7, 69, 0), 2);
	cv::imshow("img", img);
	cv::waitKey(0);
}*/

//chapter 5//

#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui.hpp>
#include <iostream>
int main() {
	float w = 250, h = 350; // Width and height of the warped output
	cv::Mat matrix, imgWarp;

	// Load the image
	cv::Mat img = cv::imread("C:/Users/sharm/OneDrive/Desktop/adriuno/codeing open cv/setup/resource/card.jpg");
	if (img.empty()) {
		std::cout << "Could not read the image: card.jpg" << std::endl;
		return 1;
	}

	// Define source points from the input image
	cv::Point2f src[4] = { {529, 142}, {771, 190}, {405, 395}, {674, 457} };

	// Define destination points for the output image
	cv::Point2f dst[4] = { {0.0f, 0.0f}, {w, 0.0f}, {0.0f, h}, {w, h} };

	// Get the perspective transform matrix
	matrix = cv::getPerspectiveTransform(src, dst);

	// Perform the perspective warp
	cv::warpPerspective(img, imgWarp, matrix, cv::Size(w, h));

	// Display the images
	cv::imshow("Original Image", img);
	cv::imshow("Warped Image", imgWarp);

	cv::waitKey(0);
	return 0;
}
