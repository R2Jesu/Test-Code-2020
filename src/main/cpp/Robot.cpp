/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/
#define _USE_MATH_DEFINES

#include <thread>

#include <cameraserver/CameraServer.h>
#include <frc/smartdashboard/smartdashboard.h>
#include <frc/TimedRobot.h>
#include <opencv2/core/core.hpp>
#include <opencv2/core/types.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <wpi/raw_ostream.h>
#include <GripPipeline.h>
#include <cmath>
#include <frc/PWMVictorSPX.h>
#include <frc/drive/DifferentialDrive.h>


//frc::PWMVictorSPX m_left{1};
//  frc::PWMVictorSPX m_right{0};

//frc::DifferentialDrive m_robotDrive{m_left, m_right};


double turning;
/**
 * This is a demo program showing the use of OpenCV to do vision processing. The
 * image is acquired from the USB camera, then a rectangle is put on the image
 * and sent to the dashboard. OpenCV has many methods for different types of
 * processing.
 */
class Robot : public frc::TimedRobot {
#if defined(__linux__)

 private:
  static void VisionThread() {
    // Get the USB camera from CameraServer
    printf("Start\n");
    cs::UsbCamera camera = frc::CameraServer::GetInstance()->StartAutomaticCapture();
    // Set the resolution
    camera.SetResolution(640, 480);
    camera.SetExposureManual(5);
    //camera.SetFPS(15);

    // Get a CvSink. This will capture Mats from the Camera
    cs::CvSink cvSink = frc::CameraServer::GetInstance()->GetVideo();
    // Setup a CvSource. This will send images back to the Dashboard
    cs::CvSource outputStream = frc::CameraServer::GetInstance()->PutVideo("Rectangle", 640, 480);

    // Mats are very memory expensive. Lets reuse this Mat.
    cv::Mat mat;
    grip::GripPipeline gp;
    printf("Before while\n");
    while (true) {
      // Tell the CvSink to grab a frame from the camera and
      // put it
      // in the source mat.  If there is an error notify the
      // output.
      if (cvSink.GrabFrame(mat) == 0) {
        // Send the output the error.
        outputStream.NotifyError(cvSink.GetError());
        // skip the rest of the current iteration
        printf("have to continue\n");
        continue;
      }
      // Put a rectangle on the image
//      rectangle(mat, cv::Point(100, 100), cv::Point(400, 400),
//                cv::Scalar(255, 255, 255), 5);
      gp.GripPipeline::Process(mat);
      // Give the output stream a new image to display
		for (size_t i = 0; i < (*gp.GripPipeline::GetFindContoursOutput()).size(); i++)
        {
		   float contourArea = cv::contourArea((*gp.GripPipeline::GetFindContoursOutput())[i]);
          if (contourArea > 100000 || contourArea < 100)
           {
             printf("continue for this reason\n");
              continue;
           }
          printf("here\n");
           cv::Rect boundRect = cv::boundingRect((*gp.GripPipeline::GetFindContoursOutput())[i]);
           double centerX = boundRect.x + (boundRect.width / 2);
           // We actually want the top middle as the target center as this is only half the goal
           //double centerY = boundRect.y + (boundRect.height / 2);
           double centerY = boundRect.y;
           std::vector<cv::Point2d> ourPointVec;
           std::vector<cv::Point2d> undistortedPointVec;
           printf("there\n");

          ourPointVec.push_back(cv::Point2d(centerX, centerY));
          printf("this\n");
          cv::Point2d ourPoint = ourPointVec[0];
            frc::SmartDashboard::PutNumber("our x", ourPoint.x);  
            frc::SmartDashboard::PutNumber("our y", ourPoint.y);
          printf("that\n");
          //ourPointVec[0] = ourPoint;
          printf("have our point\n");

           cv::drawContours(mat, *gp.GripPipeline::GetFindContoursOutput(), i, cv::Scalar(255, 0, 0), 3);
           rectangle(mat, cv::Point(centerX - 10, centerY - 10), cv::Point(centerX + 10, centerY + 10), cv::Scalar(0, 0, 255), 5);
           printf("rectabgle and contours\n");
           cv::Mat camMat = (cv::Mat1d(3,3) << 667.0055536838427, 0.0, 342.42511872039944, 0.0, 664.985144080759, 237.32436945681167, 0.0, 0.0, 1.0);
           cv::Mat distortion = (cv::Mat1d(1,5) << 0.15703749174667256, -1.134926997716282, -0.0033293254944312435, 0.0016418473011026258, 2.1006981908434668);
           printf("before undistort\n");
           cv::undistortPoints(ourPointVec, undistortedPointVec, camMat, distortion);
           printf("after undistort\n");
           cv::Point2d undistortedPoint = undistortedPointVec[0];
           
            //double lengthX = (centerX - 320.00) / 333.82;
            //double lengthY = -(centerY - 240.00) / 333.82;
            frc::SmartDashboard::PutNumber("undist x", undistortedPoint.x);  
            frc::SmartDashboard::PutNumber("undist y", undistortedPoint.y);
            double lengthX = (undistortedPoint.x - camMat.at<double>(0, 2)) / camMat.at<double>(0, 0);
            double lengthY = -(undistortedPoint.y - camMat.at<double>(1, 2)) / camMat.at<double>(1, 1);
            frc::SmartDashboard::PutNumber("length y", lengthY);  
            frc::SmartDashboard::PutNumber("length x", lengthX);
            frc::SmartDashboard::PutNumber("center y", centerY);  
            frc::SmartDashboard::PutNumber("center x", centerX);
  
            double ax = atan2(lengthX, 1.0);
            double ay = atan2(lengthY * cos(ax), 1.0);
            //You need to remasure the camera angle and set the radians below replacing 0.139626 with whatever
            double ourDist = (98.25 - 28.00) / tan(0.139626 + ay);
            frc::SmartDashboard::PutNumber("DISTANCE", ourDist);
            turning = centerX;
            printf("end of it\n");

        }
      outputStream.PutFrame(mat); 
      //printf("crap %s\n", gp.GetFindContoursOutput());
    }
  }
  
void TeleopInit()  {
 /*while (turning < 320){
  m_left.Set(-.07);
  m_right.Set(-.07);
 }
 m_left.Set(0);
 m_right.Set(0); */
 
}

#endif

  void RobotInit() override {
    // We need to run our vision program in a separate thread. If not, our robot
    // program will not run.
#if defined(__linux__)
    std::thread visionThread(VisionThread);
    visionThread.detach();
#else
    wpi::errs() << "Vision only available on Linux.\n";
    wpi::errs().flush();
#endif
  }
};

#ifndef RUNNING_FRC_TESTS
int main() { return frc::StartRobot<Robot>(); }
#endif
