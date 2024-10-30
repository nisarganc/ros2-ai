#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>
#include <cmath>
#include <iostream>

int main(){

    cv::VideoCapture inputVideo(0);
    cv::Ptr<cv::aruco::Dictionary> dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_6X6_50);

    int image_width = 1920;
    int image_height = 1080;
    float fov = 95;
    float fovx = 47.5;
    float fovy = 47.5;
    float fx = image_width / (2.0f * std::tan(fovx * M_PI / 360.0f));
    float cx = image_width / 2.0f;
    float fy = image_height / (2.0f * std::tan(fovy * M_PI / 360.0f));
    float cy = image_height / 2.0f;
    cv::Mat cameraMatrix, distCoeffs;
    cameraMatrix = (cv::Mat_<float>(3,3) << fx, 0.0, cx, 0.0, fy, cy, 0.0, 0.0, 1.0);
    distCoeffs = (cv::Mat_<float>(1,5) < 0, 0, 0, 0, 0); // << rd1, rd2, td1, td2, rd3);

    while (inputVideo.grab()) 
    {
        cv::Mat inputImage, outputImage;
        inputVideo.retrieve(inputImage);
        inputImage.copyTo(outputImage);

        // Detection
        std::vector<int> markerIds;
        std::vector<std::vector<cv::Point2f>> markerCorners;
        cv::aruco::detectMarkers(inputImage, dictionary, markerCorners, markerIds);
        
        //estimation
        std::vector<cv::Vec3d> rvecs, tvecs;

        if (markerIds.size() > 0)
        {
            std::cout << "Detected " << markerIds.size() << " markers." << std::endl;
            cv::aruco::drawDetectedMarkers(outputImage, markerCorners, markerIds);
            cv::aruco::estimatePoseSingleMarkers(markerCorners, 0.05, cameraMatrix, distCoeffs, rvecs, tvecs);
            for(int i=0; i<markerIds.size(); i++)
                cv::drawFrameAxes(outputImage, cameraMatrix, distCoeffs, rvecs[i], tvecs[i], 0.1);
        }

        cv::imshow("outputImage", outputImage);

        char key = (char)cv::waitKey(10);
        if (key == 'q' || key == 27) {
            break;
        }
    }

    // Release the video and destroy all windows
    inputVideo.release();
    cv::destroyAllWindows();
    return 0;
}