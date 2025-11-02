#include <iostream>
#include <algorithm>
#include <fstream>
#include <chrono>
#include <vector>
#include <queue>
#include <thread>
#include <mutex>

#include <rclcpp/rclcpp.hpp>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/image.hpp>

#include <opencv2/core/core.hpp>

#include "System.h"      // Correct
#include "ImuTypes.h"    // adjust path

using namespace std;

class ImuGrabber
{
public:
    ImuGrabber() = default;

    void GrabImu(const sensor_msgs::msg::Imu::SharedPtr imu_msg)
    {
        std::lock_guard<std::mutex> lock(mBufMutex);
        imuBuf.push(imu_msg);
    }

    queue<sensor_msgs::msg::Imu::SharedPtr> imuBuf;
    std::mutex mBufMutex;
};

class ImageGrabber : public rclcpp::Node
{
public:
    ImageGrabber(ORB_SLAM3::System* pSLAM, ImuGrabber* pImuGb, const bool bClahe)
        : Node("mono_inertial_slam"), mpSLAM(pSLAM), mpImuGb(pImuGb), mbClahe(bClahe)
    {
        // Subscribers
        sub_img0_ = this->create_subscription<sensor_msgs::msg::Image>(
            "/camera/image_raw", 100,
            std::bind(&ImageGrabber::GrabImage, this, std::placeholders::_1));

        sub_imu_ = this->create_subscription<sensor_msgs::msg::Imu>(
            "/imu/data_raw", 1000,
            std::bind(&ImuGrabber::GrabImu, mpImuGb, std::placeholders::_1));

        // Launch sync thread
        sync_thread_ = std::thread(&ImageGrabber::SyncWithImu, this);
    }

    ~ImageGrabber()
    {
        if (sync_thread_.joinable())
            sync_thread_.join();
    }

    void GrabImage(const sensor_msgs::msg::Image::SharedPtr img_msg)
    {
        std::lock_guard<std::mutex> lock(mBufMutex);
        if (!img0Buf.empty())
            img0Buf.pop();
        img0Buf.push(img_msg);
    }

    cv::Mat GetImage(const sensor_msgs::msg::Image::SharedPtr& img_msg)
    {
        cv_bridge::CvImageConstPtr cv_ptr;
        try
        {
            cv_ptr = cv_bridge::toCvShare(img_msg, sensor_msgs::image_encodings::MONO8);
        }
        catch (cv_bridge::Exception& e)
        {
            RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
            return cv::Mat();
        }

        if (cv_ptr->image.type() == 0)
            return cv_ptr->image.clone();
        else
        {
            RCLCPP_WARN(this->get_logger(), "Unexpected image type, returning clone anyway.");
            return cv_ptr->image.clone();
        }
    }

    //void SyncWithImu()
    //{
        //while (rclcpp::ok())
        //{
            //cv::Mat im;
            //double tIm = 2.0;

            //if (!img0Buf.empty() && !mpImuGb->imuBuf.empty())
            //{
                //tIm = img0Buf.front()->header.stamp.sec +
                      //img0Buf.front()->header.stamp.nanosec * 1e-9;

                //if (tIm > (mpImuGb->imuBuf.back()->header.stamp.sec +
                           //mpImuGb->imuBuf.back()->header.stamp.nanosec * 1e-9))
                    //continue;

                //{
                    //std::lock_guard<std::mutex> lock(mBufMutex);
                    //im = GetImage(img0Buf.front());
                    //img0Buf.pop();
                //}

                //vector<ORB_SLAM3::IMU::Point> vImuMeas;
                //{
                    //std::lock_guard<std::mutex> lock(mpImuGb->mBufMutex);
                    //while (!mpImuGb->imuBuf.empty())
                    //{
                        //double t = mpImuGb->imuBuf.front()->header.stamp.sec +
                                   //mpImuGb->imuBuf.front()->header.stamp.nanosec * 1e-9;

                        //if (t > tIm) break;

                        //auto imu = mpImuGb->imuBuf.front();
                        //cv::Point3f acc(imu->linear_acceleration.x,
                                        //imu->linear_acceleration.y,
                                        //imu->linear_acceleration.z);
                        //cv::Point3f gyr(imu->angular_velocity.x,
                                        //imu->angular_velocity.y,
                                        //imu->angular_velocity.z);
                        //vImuMeas.emplace_back(acc, gyr, t);

                        //mpImuGb->imuBuf.pop();
                    //}
                //}

                //if (mbClahe)
                    //mClahe->apply(im, im);
                

                //mpSLAM->TrackMonocular(im, tIm, vImuMeas);
            //}

            //std::this_thread::sleep_for(std::chrono::milliseconds(1));
        //}
    //}
    void SyncWithImu()
{
    const double CAM_IMU_TIME_OFFSET = 0.1; // Positive: camera is ahead of IMU by 0.1s

    while (rclcpp::ok())
    {
        cv::Mat im;
        double tIm = 2.0;

        if (!img0Buf.empty() && !mpImuGb->imuBuf.empty())
        {
            // -----------------------------
            // 1. Adjust camera timestamp
            // -----------------------------
            tIm = img0Buf.front()->header.stamp.sec +
                  img0Buf.front()->header.stamp.nanosec * 1e-9;

            // Apply offset measured by Kalibr
            tIm += CAM_IMU_TIME_OFFSET;

            // ------------------------------------------------------
            // 2. If adjusted image timestamp is ahead of latest IMU
            //    wait for more IMU data
            // ------------------------------------------------------
            double latestImuTime = mpImuGb->imuBuf.back()->header.stamp.sec +
                                   mpImuGb->imuBuf.back()->header.stamp.nanosec * 1e-9;

            if (tIm > latestImuTime)
                continue;

            // -----------------------------
            // 3. Extract image safely
            // -----------------------------
            {
                std::lock_guard<std::mutex> lock(mBufMutex);
                im = GetImage(img0Buf.front());
                img0Buf.pop();
            }

            // -----------------------------
            // 4. Collect IMU measurements
            // -----------------------------
            vector<ORB_SLAM3::IMU::Point> vImuMeas;
            {
                std::lock_guard<std::mutex> lock(mpImuGb->mBufMutex);

                while (!mpImuGb->imuBuf.empty())
                {
                    double t = mpImuGb->imuBuf.front()->header.stamp.sec +
                               mpImuGb->imuBuf.front()->header.stamp.nanosec * 1e-9;

                    // Stop when IMU timestamp exceeds adjusted image timestamp
                    if (t > tIm) break;

                    auto imu = mpImuGb->imuBuf.front();

                    cv::Point3f acc(imu->linear_acceleration.x,
                                    imu->linear_acceleration.y,
                                    imu->linear_acceleration.z);

                    cv::Point3f gyr(imu->angular_velocity.x,
                                    imu->angular_velocity.y,
                                    imu->angular_velocity.z);

                    vImuMeas.emplace_back(acc, gyr, t);

                    mpImuGb->imuBuf.pop();
                }
            }

            // -----------------------------
            // 5. Optional CLAHE preprocessing
            // -----------------------------
            if (mbClahe)
                mClahe->apply(im, im);

            // -----------------------------
            // 6. Pass data to ORB-SLAM3
            // -----------------------------
            mpSLAM->TrackMonocular(im, tIm, vImuMeas);
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
}


private:
    ORB_SLAM3::System* mpSLAM;
    ImuGrabber* mpImuGb;
    const bool mbClahe;
    cv::Ptr<cv::CLAHE> mClahe = cv::createCLAHE(3.0, cv::Size(8, 8));

    queue<sensor_msgs::msg::Image::SharedPtr> img0Buf;
    std::mutex mBufMutex;

    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_img0_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr sub_imu_;
    std::thread sync_thread_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);

    if (argc < 3 || argc > 4)
    {
        std::cerr << "Usage: ros2 run ORB_SLAM3 mono_inertial path_to_vocabulary path_to_settings [do_equalize]" << std::endl;
        rclcpp::shutdown();
        return 1;
    }

    bool bEqual = false;
    if (argc == 4)
    {
        std::string sbEqual(argv[3]);
        if (sbEqual == "true")
            bEqual = true;
    }

    ORB_SLAM3::System SLAM(argv[1], argv[2], ORB_SLAM3::System::IMU_MONOCULAR, true);

    auto imuGrabber = std::make_shared<ImuGrabber>();
    auto imgGrabber = std::make_shared<ImageGrabber>(&SLAM, imuGrabber.get(), bEqual);

    rclcpp::spin(imgGrabber);

    SLAM.Shutdown();
    SLAM.SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory.txt");

    rclcpp::shutdown();
    return 0;
}
