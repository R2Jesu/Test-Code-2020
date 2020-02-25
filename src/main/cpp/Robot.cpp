/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include <thread>

#include <cameraserver/CameraServer.h>
#include <frc/TimedRobot.h>
#include <opencv2/core/core.hpp>
#include <opencv2/core/types.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <wpi/raw_ostream.h>
#include <GripPipeline.h>

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

    // Get a CvSink. This will capture Mats from the Camera
    cs::CvSink cvSink = frc::CameraServer::GetInstance()->GetVideo();
    // Setup a CvSource. This will send images back to the Dashboard
    cs::CvSource outputStream =
        frc::CameraServer::GetInstance()->PutVideo("Rectangle", 640, 480);

    // Mats are very memory expensive. Lets reuse this Mat.
    cv::Mat mat;
    grip::GripPipeline gp;

       while (true) {
      // Get a frame from the camera stream and
      // put it in the source mat.  
      // If there is an error notify the output and skip the rest of the while

      if (cvSink.GrabFrame(mat) == 0) {
        outputStream.NotifyError(cvSink.GetError());
        continue;
      }
      //This below sends the mat or frame to the GRIP process that should tune it down to just the part of the image we want to see.  
      //Tuning can really only be done in GRIP easily.  Alternatively you can change the numbers in the other cpp file, but you
      //have to understand what they mean and adjust them slowly.  This really just gives you back contour values which was the last step in GRIP processing

      gp.GripPipeline::Process(mat);

      //This calculates the area of the contours as it loops through them.  This is to discard really big and really small contours
      //This number might need to be played with.  I guessed on a max an min.  We might lose something here.  If it isn't within the bounds the rest of the while
      // is skipped

      for (size_t i = 0; i < (*gp.GripPipeline::GetFindContoursOutput()).size(); i++)
        {
   float contourArea = cv::contourArea((*gp.GripPipeline::GetFindContoursOutput())[i]);
           if (contourArea > 100000 || contourArea < 100)
           {
              continue;
           }

      // This draws an imaginary rectangle around the contours.  The rectangle is used to find the midpoint 

           cv::Rect boundRect = cv::boundingRect((*gp.GripPipeline::GetFindContoursOutput())[i]);

      // The below finds the middle of the rectangle.  Since we have only half the target, I commented out one calculation and
      // changed it to get the top middle instead of the middle middle.  That is the centerY calculation

           double centerX = boundRect.x + (boundRect.width / 2);
      //      double centerY = boundRect.y + (boundRect.height / 2);
           double centerY = boundRect.y;

      // This just takes the contours again and draws a colored line on them.

           cv::drawContours(mat, *gp.GripPipeline::GetFindContoursOutput(), i, cv::Scalar(255, 0, 0), 3);

      // This draws a 10 pixel the box at the center point.  The centerX and centerY are the values you should look at if you are targeting in autonomous
      // You want to figure what they are when the target shows in the middle of the frame and then look for those values in your turn
      // In autonomous the drawing is probably waste, you just need the values

           rectangle(mat, cv::Point(centerX - 10, centerY - 10), cv::Point(centerX + 10, centerY + 10), cv::Scalar(0, 0, 255), 5);
        }

      // Take the mat with all the changes made above and send it to the output stream in the default driver station console
      outputStream.PutFrame(mat); 
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
