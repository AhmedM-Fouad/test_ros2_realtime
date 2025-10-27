//#include <System.h>
//#include <opencv2/opencv.hpp>
//#include <chrono>
//#include <thread>
//#include <iostream>
//#include <sstream>
//#include <vector>
//#include <cmath>
//#include <fcntl.h>
//#include <termios.h>
//#include <unistd.h>

//using namespace std;

//// IMU scales for MPU6050
//const double ACCEL_SCALE = 16384.0;   // LSB/g
//const double GYRO_SCALE  = 131.0;     // LSB/°/s

//// Configure POSIX serial port
//int configure_serial(const char* portname, speed_t baud=B921600)
//{
    //int fd = open(portname, O_RDONLY | O_NOCTTY | O_SYNC);
    //if (fd < 0) {
        //cerr << "ERROR: cannot open serial port " << portname << endl;
        //return -1;
    //}

    //struct termios tty;
    //if (tcgetattr(fd, &tty) != 0) {
        //cerr << "Error from tcgetattr" << endl;
        //return -1;
    //}

    //cfsetospeed(&tty, baud);
    //cfsetispeed(&tty, baud);

    //tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8;   // 8-bit chars
    //tty.c_iflag &= ~IGNBRK;
    //tty.c_lflag = 0;  // no signaling chars, no echo, no canonical processing
    //tty.c_oflag = 0;
    //tty.c_cc[VMIN]  = 1;
    //tty.c_cc[VTIME] = 1;

    //tty.c_iflag &= ~(IXON | IXOFF | IXANY);
    //tty.c_cflag |= (CLOCAL | CREAD);
    //tty.c_cflag &= ~(PARENB | PARODD);
    //tty.c_cflag &= ~CSTOPB;
    //tty.c_cflag &= ~CRTSCTS;

    //if (tcsetattr(fd, TCSANOW, &tty) != 0) {
        //cerr << "Error from tcsetattr" << endl;
        //return -1;
    //}

    //return fd;
//}

//int main(int argc, char **argv)
//{
    //if(argc != 4) {
        //cerr << "Usage: ./mono_inertial path_to_vocabulary path_to_settings /dev/ttyUSB0" << endl;
        //return 1;
    //}

    //// --- ORB-SLAM3 system ---
    //ORB_SLAM3::System SLAM(argv[1], argv[2],
                           //ORB_SLAM3::System::IMU_MONOCULAR, true);

    //// --- Camera capture ---
    //cv::VideoCapture cap(0); // adjust index
    //if(!cap.isOpened()) {
        //cerr << "ERROR: Cannot open camera" << endl;
        //return -1;
    //}

    //// --- Serial IMU ---
    //int serial_fd = configure_serial(argv[3], B921600);
    //if(serial_fd < 0) return -1;

    //string buffer;
    //char c;

    //while(true) {
        //cv::Mat frame;
        //cap >> frame;
        //if(frame.empty()) break;

        //// Frame timestamp
        //double tframe = chrono::duration<double>(
            //chrono::system_clock::now().time_since_epoch()).count();

        //// Collect IMU measurements
        //vector<ORB_SLAM3::IMU::Point> vImuMeas;

        //// Non-blocking read loop
        //while (true) {
            //int n = read(serial_fd, &c, 1);
            //if (n > 0) {
                //if (c == '\n') {
                    //// Process one line
                    //stringstream ss(buffer);
                    //string item;
                    //vector<int> values;
                    //while (getline(ss, item, ',')) {
                        //try {
                            //values.push_back(stoi(item));
                        //} catch (...) {
                            //values.clear();
                            //break;
                        //}
                    //}
                    //if(values.size() == 6) {
                        //int ax_raw = values[0];
                        //int ay_raw = values[1];
                        //int az_raw = values[2];
                        //int gx_raw = values[3];
                        //int gy_raw = values[4];
                        //int gz_raw = values[5];

                        //// Convert to SI units
                        //double ax = (ax_raw / ACCEL_SCALE) * 9.81;
                        //double ay = (ay_raw / ACCEL_SCALE) * 9.81;
                        //double az = (az_raw / ACCEL_SCALE) * 9.81;

                        //double gx = (gx_raw / GYRO_SCALE) * M_PI / 180.0;
                        //double gy = (gy_raw / GYRO_SCALE) * M_PI / 180.0;
                        //double gz = (gz_raw / GYRO_SCALE) * M_PI / 180.0;

                        //double timu = chrono::duration<double>(
                            //chrono::system_clock::now().time_since_epoch()).count();

                        //vImuMeas.emplace_back(ax, ay, az, gx, gy, gz, timu);
                    //}
                    //buffer.clear();
                //} else {
                    //buffer += c;
                //}
            //} else {
                //break; // no more data
            //}
        //}

        //// Feed into ORB-SLAM3
        //SLAM.TrackMonocular(frame, tframe, vImuMeas);

        //cv::imshow("Camera", frame);
        //if(cv::waitKey(1) == 27) break; // ESC to quit
    //}

    //SLAM.Shutdown();
    //close(serial_fd);
    //return 0;
//}

#include <iostream>
#include <thread>
#include <mutex>
#include <vector>
#include <chrono>
#include <opencv2/opencv.hpp>
#include <termios.h>
#include <unistd.h>
#include <fcntl.h>

// ORB-SLAM3 headers
#include "System.h"
#include "ImuTypes.h"

struct ImuSample {
    double timestamp;     // seconds
    float ax, ay, az;     // m/s^2
    float gx, gy, gz;     // rad/s
};

std::vector<ImuSample> imuBuffer;
std::mutex imuMutex;
bool running = true;

// MPU6050 scale factors
const float ACC_SCALE = 9.81f / 16384.0f;            // m/s^2 per LSB
const float GYRO_SCALE = (M_PI / 180.0f) / 131.0f;   // rad/s per LSB

// Serial port setup
int openSerial(const char* portname, int baud=921600) {
    int fd = open(portname, O_RDWR | O_NOCTTY | O_SYNC);
    if(fd < 0) {
        std::cerr << "Error opening " << portname << std::endl;
        exit(1);
    }
    termios tty{};
    tcgetattr(fd, &tty);
    cfsetospeed(&tty, B921600);
    cfsetispeed(&tty, B921600);
    tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8; // 8-bit chars
    tty.c_iflag &= ~IGNBRK;
    tty.c_lflag = 0;
    tty.c_oflag = 0;
    tty.c_cc[VMIN]  = 0;
    tty.c_cc[VTIME] = 1;
    tty.c_cflag |= (CLOCAL | CREAD);
    tty.c_cflag &= ~(PARENB | PARODD);
    tty.c_cflag &= ~CSTOPB;
    tty.c_cflag &= ~CRTSCTS;
    tcsetattr(fd, TCSANOW, &tty);
    return fd;
}

// Thread function to read IMU packets
void imuReaderThread(const char* port) {
    int fd = openSerial(port);
    uint8_t buffer[14]; // header(2) + 6*2 bytes
    uint8_t c;
    while(running) {
        // look for header 0xAA 0x55
        if(read(fd, &c, 1) == 1 && c == 0xAA) {
            if(read(fd, &c, 1) == 1 && c == 0x55) {
                int got = 0;
                while(got < 12) {
                    int r = read(fd, buffer+got, 12-got);
                    if(r > 0) got += r;
                }
                int16_t ax_raw = (buffer[0]<<8) | buffer[1];
                int16_t ay_raw = (buffer[2]<<8) | buffer[3];
                int16_t az_raw = (buffer[4]<<8) | buffer[5];
                int16_t gx_raw = (buffer[6]<<8) | buffer[7];
                int16_t gy_raw = (buffer[8]<<8) | buffer[9];
                int16_t gz_raw = (buffer[10]<<8)| buffer[11];

                auto now = std::chrono::steady_clock::now();
                double ts = std::chrono::duration_cast<std::chrono::duration<double>>(now.time_since_epoch()).count();

                ImuSample s;
                s.timestamp = ts;
                s.ax = ax_raw * ACC_SCALE;
                s.ay = ay_raw * ACC_SCALE;
                s.az = az_raw * ACC_SCALE;
                s.gx = gx_raw * GYRO_SCALE;
                s.gy = gy_raw * GYRO_SCALE;
                s.gz = gz_raw * GYRO_SCALE;

                std::lock_guard<std::mutex> lock(imuMutex);
                imuBuffer.push_back(s);
                
                //Debug IMU rate
                static auto last = std::chrono::steady_clock::now();
                static int count = 0;
                count++;
                auto now2 = std::chrono::steady_clock::now();
                double dt = std::chrono::duration<double>(now2 - last).count();
                if (dt >= 1.0) {
                    std::cout << "IMU rate = " << count/dt << " Hz" << std::endl;
                    last = now2;
                    count = 0;
                }
            }
        }
    }
    close(fd);
}

int main(int argc, char** argv) {
    if(argc < 4) {
        std::cerr << "Usage: ./vi_slam ORBvoc.txt settings.yaml /dev/ttyUSB0" << std::endl;
        return 1;
    }

    // Start IMU thread
    std::thread imuThread(imuReaderThread, argv[3]);

    // Open camera
    cv::VideoCapture cap(0);
    cap.set(cv::CAP_PROP_FPS, 30);
    cap.set(cv::CAP_PROP_FRAME_WIDTH, 640);
    cap.set(cv::CAP_PROP_FRAME_HEIGHT, 480);
    std::cout << "Camera reports FPS: " << cap.get(cv::CAP_PROP_FPS) << std::endl;

    if(!cap.isOpened()) {
        std::cerr << "Cannot open camera" << std::endl;
        return -1;
    }

    // Init ORB-SLAM3 system (monocular + IMU)
    ORB_SLAM3::System SLAM(argv[1], argv[2], ORB_SLAM3::System::IMU_MONOCULAR, true);

    double lastFrameTime = 0;
    while(true) {
        cv::Mat frame;
        cap >> frame;
        if(frame.empty()) break;

        auto now = std::chrono::steady_clock::now();
        double tframe = std::chrono::duration_cast<std::chrono::duration<double>>(now.time_since_epoch()).count();

        std::vector<ORB_SLAM3::IMU::Point> vImuMeas;
        {
            std::lock_guard<std::mutex> lock(imuMutex);
            for(auto it = imuBuffer.begin(); it != imuBuffer.end();) {
                if(it->timestamp <= tframe) {
                    vImuMeas.emplace_back(
                        it->ax, it->ay, it->az,
                        it->gx, it->gy, it->gz,
                        it->timestamp
                    );
                    it = imuBuffer.erase(it);
                } else {
                    ++it;
                }
            }

        }
        lastFrameTime = tframe;
        
        //std::cout << "Frame " << tframe 
          //<< " | IMU samples = " << vImuMeas.size()
          //<< " | Last accel z = " << (vImuMeas.empty()?0:vImuMeas.back().a.z())
          //<< std::endl;
        std::cout << "Frame timestamp: " << tframe 
          << " | IMU samples sent: " << vImuMeas.size() << std::endl;
        
        // Feed into ORB-SLAM3
        SLAM.TrackMonocular(frame, tframe, vImuMeas);

        //cv::imshow("Camera", frame);
        if(cv::waitKey(1) == 27) break; // ESC
    }

    running = false;
    imuThread.join();
    SLAM.Shutdown();

    return 0;
}
