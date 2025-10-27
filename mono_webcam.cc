// mono_webcam.cc
#include <System.h>
#include <opencv2/opencv.hpp>
#include <chrono>
#include <iostream>

int main(int argc, char **argv) {
    if (argc != 3) {
        std::cerr << "Usage: ./mono_webcam path_to_vocabulary path_to_settings" << std::endl;
        return -1;
    }

    ORB_SLAM3::System SLAM(argv[1], argv[2], ORB_SLAM3::System::MONOCULAR, true);

    cv::VideoCapture cap(0);  // open default camera
    if (!cap.isOpened()) {
        std::cerr << "Cannot open webcam" << std::endl;
        return -1;
    }
    // Set camera resolution to 640x480
    cap.set(cv::CAP_PROP_FRAME_WIDTH, 640);
    cap.set(cv::CAP_PROP_FRAME_HEIGHT, 480);
    cap.set(cv::CAP_PROP_FPS, 20);


    while (true) {
        cv::Mat frame;
        cap >> frame;
        if (frame.empty()) break;
        auto t_start = std::chrono::steady_clock::now();
        SLAM.TrackMonocular(frame, cv::getTickCount() / cv::getTickFrequency());
        auto t_end = std::chrono::steady_clock::now();

        double fps = 1.0 / std::chrono::duration<double>(t_end - t_start).count();
        std::cout << "FPS: " << fps << "\r" << std::flush;

        char key = cv::waitKey(1);
        if (key == 27) break; // ESC to quit
    }
    // Before SLAM.Shutdown():
//    SLAM.SaveMap("MyMap.osa");  // Saves the map to a binary file
    SLAM.SaveTrajectoryTUM("CameraTrajectory.txt");
    SLAM.SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory.txt");

    SLAM.Shutdown();
    return 0;
}


//#include <System.h>
//#include <opencv2/opencv.hpp>
//#include <chrono>
//#include <iostream>
//#include <thread>  // for std::this_thread::sleep_for

//int main(int argc, char **argv) {
    //if (argc != 3) {
        //std::cerr << "Usage: ./mono_webcam path_to_vocabulary path_to_settings" << std::endl;
        //return -1;
    //}

    //ORB_SLAM3::System SLAM(argv[1], argv[2], ORB_SLAM3::System::MONOCULAR, true);

    //cv::VideoCapture cap(0);  // Open default camera
    //if (!cap.isOpened()) {
        //std::cerr << "Cannot open webcam" << std::endl;
        //return -1;
    //}

    //// Set camera to 30 fps if possible
    //cap.set(cv::CAP_PROP_FPS, 30);

    //const double frame_interval_ms = 1000.0 / 30.0;  // ~33.33 ms per frame

    //while (true) {
        //auto t_start = std::chrono::steady_clock::now();

        //cv::Mat frame;
        //cap >> frame;
        //if (frame.empty()) break;

        //cv::resize(frame, frame, cv::Size(640, 480));

        //double timestamp = cv::getTickCount() / cv::getTickFrequency();  // in seconds
        //SLAM.TrackMonocular(frame, timestamp);

        //auto t_end = std::chrono::steady_clock::now();
        //double elapsed_ms = std::chrono::duration<double, std::milli>(t_end - t_start).count();

        //double delay_ms = frame_interval_ms - elapsed_ms;
        //if (delay_ms > 0) {
            //std::this_thread::sleep_for(std::chrono::milliseconds(static_cast<int>(delay_ms)));
        //}

        //double actual_fps = 1000.0 / std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t_start).count();
        //std::cout << "FPS: " << actual_fps << "\r" << std::flush;

        //char key = cv::waitKey(1);
        //if (key == 27) break; // ESC to quit
    //}
    
    //SLAM.SaveTrajectoryTUM("CameraTrajectory.txt");
    //SLAM.SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory.txt");

    //SLAM.Shutdown();
    //return 0;
//}
