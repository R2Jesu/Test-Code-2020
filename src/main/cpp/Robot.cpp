/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

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


frc::PWMVictorSPX m_left{1};
  frc::PWMVictorSPX m_right{0};

frc::DifferentialDrive m_robotDrive{m_left, m_right};


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
    cs::UsbCamera camera =
        frc::CameraServer::GetInstance()->StartAutomaticCapture();
    // Set the resolution
    camera.SetResolution(640, 480);
    camera.SetExposureManual(5);

    // Get a CvSink. This will capture Mats from the Camera
    cs::CvSink cvSink = frc::CameraServer::GetInstance()->GetVideo();
    // Setup a CvSource. This will send images back to the Dashboard
    cs::CvSource outputStream =
        frc::CameraServer::GetInstance()->PutVideo("Rectangle", 640, 480);

    // Mats are very memory expensive. Lets reuse this Mat.
    cv::Mat mat;
    grip::GripPipeline gp;

    while (true) {
      // Tell the CvSink to grab a frame from the camera and
      // put it
      // in the source mat.  If there is an error notify the
      // output.
      if (cvSink.GrabFrame(mat) == 0) {
        // Send the output the error.
        outputStream.NotifyError(cvSink.GetError());
        // skip the rest of the current iteration
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
              continue;
           }
//           printf("Something else: %f\n", contourArea);
           cv::Rect boundRect = cv::boundingRect((*gp.GripPipeline::GetFindContoursOutput())[i]);
// We actually want the top middle as the target center as this is half the goal
           double centerX = boundRect.x + (boundRect.width / 2);
           double centerY = boundRect.y + (boundRect.height / 2);
//           double centerY = boundRect.y;
           cv::drawContours(mat, *gp.GripPipeline::GetFindContoursOutput(), i, cv::Scalar(255, 0, 0), 3);
           rectangle(mat, cv::Point(centerX - 10, centerY - 10), cv::Point(centerX + 10, centerY + 10), cv::Scalar(0, 0, 255), 5);
         
           
            double lengthX = abs(centerX - 320);
            double lengthY = abs(centerY - 240);
            frc::SmartDashboard::PutNumber("length y", lengthY);  
            frc::SmartDashboard::PutNumber("length x", lengthX);
             frc::SmartDashboard::PutNumber("center y", centerY);  
            frc::SmartDashboard::PutNumber("center x", centerX);
           // double distance = 70.75 * ( 1 / tan(atan(sqrt(pow(lengthX, 2) + pow(lengthY, 2)) / 333.82 )));  98.25
    //       double triLength = sqrt((lengthX * lengthX) + (lengthY * lengthY));//(lengthX * lengthX) + (lengthY * lengthY)
           double triLength = sqrt((centerX * centerX) + (centerY * centerY));//(lengthX * lengthX) + (lengthY * lengthY)
     //      double triLength = sqrt((centerX * centerX) + (lengthY * lengthY));//(lengthX * lengthX) + (lengthY * lengthY)
           double angle = atan(triLength / 333.82); //700
           double distance = 70.25 * (1 / tan(angle));
           
           frc::SmartDashboard::PutNumber("trilength", triLength);
           frc::SmartDashboard::PutNumber("angle", angle);
            frc::SmartDashboard::PutNumber("distance", distance);
            double distance2 = centerY * 51 / 133 + 70;
            frc::SmartDashboard::PutNumber("distance2", distance2);
            double distance3 = ((70.25 * 333.82) / triLength);
            frc::SmartDashboard::PutNumber("distance3", distance3);
            turning = centerX;
        }
      outputStream.PutFrame(mat); 
      //printf("crap %s\n", gp.GetFindContoursOutput());
    }
  }
  
void TeleopInit()  {
 while (turning < 320){
  m_left.Set(-.15);
  m_right.Set(-.15);
 }
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
