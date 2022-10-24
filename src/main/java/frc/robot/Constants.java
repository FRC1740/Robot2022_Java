// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
   
    public static class ConMath {
        public static double PI = Math.PI; // 3.141592;
        public static double METERS_2_INCH = .0254; // m/in
        public static double MINUTES_2_SECONDS = 1/60.; // sec/min
        public static double RAD_2_DEG = 180.0/PI;
        public static double DEG_2_RAD = 1/RAD_2_DEG;
    }

    public static class ConLimelight {
        public static int VISION_MODE = 0;
        public static int CAMERA_MODE = 1;
    
        public static int LED_PIPLINE_DEFAULT = 0;
        public static int LED_OFF = 1;
        public static int LED_BLINK = 2;
        public static int LED_ON = 3;
    
        public static int SNAPSHOT_STOP = 0;
        public static int SNAPSHOT_START = 1;
    
        public static double HORIZONTAL_TOLERANCE = 1.0;  //degrees
        public static double TARGET_HEIGHT = 38.5; //in to center of target
        public static double CAMERA_HEIGHT = 19.5; //in to center of camera
        public static double MAX_HORIZONTAL_OFFSET = 29.8; //degrees
    
        // constexpr cv::Matx33d cameraMatrix = cv::Matx33d(
        //                     772.53876202, 0., 479.132337442,
        //                     0., 769.052151477, 359.143001808,
        //                     0., 0., 1.0);
        // constexpr std::vector istortionCoefficient =  std::vector<double> {
        //                     2.9684613693070039e-01, -1.4380252254747885e+00,-2.2098421479494509e-03,
        //                     -3.3894563533907176e-03, 2.5344430354806740e+00};
    
        public static double focalLength = 2.9272781257541; //mm
    }
    
    public static class ConSparkMax {
        public static double POSITION_CONVERSION_FACTOR = 42.0;
    }
}

/** Original CPP

#pragma once

#include <cmath>  // for std::fabs
#include <math.h>

// Units required for Trajectory Following
#include <units/acceleration.h>
#include <units/angle.h>
#include <units/angular_acceleration.h>
#include <units/angular_velocity.h>
#include <units/length.h>
#include <units/time.h>
#include <units/velocity.h>
#include <units/voltage.h>
//On error, create env.h from env-default.h and modify ROBOT_VERSION_STRING
#include "env.h"


namespace ConMath {
    constexpr double PI = M_PI; // 3.141592;
    constexpr double METERS_2_INCH = .0254; // m/in
    constexpr double MINUTES_2_SECONDS = 1/60.; // sec/min
    constexpr double RAD_2_DEG = 180.0/PI;
    constexpr double DEG_2_RAD = 1/RAD_2_DEG;
}

namespace ConLimelight {
    constexpr int VISION_MODE = 0;
    constexpr int CAMERA_MODE = 1;

    constexpr int LED_PIPLINE_DEFAULT = 0;
    constexpr int LED_OFF = 1;
    constexpr int LED_BLINK = 2;
    constexpr int LED_ON = 3;

    constexpr int SNAPSHOT_STOP = 0;
    constexpr int SNAPSHOT_START = 1;

    constexpr double HORIZONTAL_TOLERANCE = 1.0;  //degrees
    constexpr units::length::inch_t TARGET_HEIGHT = 38.5_in; //in to center of target
    constexpr units::length::inch_t CAMERA_HEIGHT = 19.5_in; //in to center of camera
    constexpr units::angle::degree_t MAX_HORIZONTAL_OFFSET = 29.8_deg; //degrees

    // constexpr cv::Matx33d cameraMatrix = cv::Matx33d(
    //                     772.53876202, 0., 479.132337442,
    //                     0., 769.052151477, 359.143001808,
    //                     0., 0., 1.0);
    // constexpr std::vector istortionCoefficient =  std::vector<double> {
    //                     2.9684613693070039e-01, -1.4380252254747885e+00,-2.2098421479494509e-03,
    //                     -3.3894563533907176e-03, 2.5344430354806740e+00};

    constexpr double focalLength = 2.9272781257541; //mm
}

namespace ConSparkMax {
    constexpr double POSITION_CONVERSION_FACTOR = 42.0;
}

*/