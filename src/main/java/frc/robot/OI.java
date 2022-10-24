package frc.robot;
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

/** Add your docs here. */
public class OI {

    public static class ConShuffleboard {
        public static String RobotTab = "Robot";
        public static String ClimberTab = "Climber";
        public static String DriveTrainTab = "DriveTrain";
        public static String LauncherTab = "Launcher";
        public static String IntakeTab = "Intake";
        public static String VisionTab = "Vision";
    }

    public static class ConFlightControl {
        // Axes
        public static int AILERON = 0;
        public static int ELEVATOR = 1;
        public static int RUDDER = 2;
        public static int TRIM = 3;
        // Buttons
        public static int TRIGGER = 0;
    }
    
    public static class ConXBOXControl {
        // Axis inputs
        public static int LEFT_JOYSTICK_X = 0;
        public static int LEFT_JOYSTICK_Y = 1;
        public static int LEFT_TRIGGER = 2;
        public static int RIGHT_TRIGGER = 3;
        public static int RIGHT_JOYSTICK_X = 4;
        public static int RIGHT_JOYSTICK_Y = 5;
        // Buttons
        public static int A = 1;
        public static int B = 2;
        public static int X = 3;
        public static int Y = 4;
        public static int LEFT_BUMPER = 5;
        public static int RIGHT_BUMPER = 6;
        public static int SELECT = 7;
        public static int START = 8;
        public static int LEFT_JOYSTICK = 9;
        public static int RIGHT_JOYSTICK = 10;
    
        // Dead zone
        public static double DEAD_ZONE = 0.1;
    
        // Drive Station Controller "Ports"
        public static int DRIVER_CONTROLLER_PORT = 0;
        public static int CODRIVER_CONTROLLER_PORT = 1;
    }
    
    // DeadZone lambda function
   // auto DeadZone = [] (double value) { return (std::fabs(value) > ConXBOXControl.DEAD_ZONE) ? value : 0.0; };
    public static double deadZone(double value) {
        if(Math.abs(value) > ConXBOXControl.DEAD_ZONE) {
            return value;
        }
        return 0.0;
    }
    
    // Texas Instruments Controller (MSP430F5529 LaunchPad)
    public static class ConLaunchPad {
        public static class Button {
            public static int RED = 1;  // Jumble Fwd
            public static int BLUE = 2; // Jumble Rev
            public static int YELLOW = 3; // Not Used (Climber?)
            public static int GREEN = 4; // Not Used (Climber?)
            public static int WHITE = 5; // Climber Unlock (Hold to unlock)
        }
    
        public static class Switch {
            public static int RED = 6;
            public static int BLUE = 7;
            public static int YELLOW = 8;
            public static int GREEN = 9; // Reset/Disable Climber Encoder
        }
    
        public static class Dial {
            public static int LEFT = 2;
            public static int RIGHT = 6;
        }
    
        public static int SLIDER = 42; // Unknown axis
        public static int LEFT_STICK_X = 1; 
        public static int LEFT_STICK_Y = 0;
    
        public static int RIGHT_STICK_X = 5; 
        public static int RIGHT_STICK_Y = 4;
    
        public static int LAUNCHPAD_CONTROLLER_PORT = 1;
    }   
}

/** Original H

#pragma once

namespace ConShuffleboard {
    constexpr char RobotTab[] = "Robot";
    constexpr char ClimberTab[] = "Climber";
    constexpr char DriveTrainTab[] = "DriveTrain";
    constexpr char LauncherTab[] = "Launcher";
    constexpr char IntakeTab[] = "Intake";
    constexpr char VisionTab[] = "Vision";
}

namespace ConFlightControl {
    // Axes
    constexpr int AILERON = 0;
    constexpr int ELEVATOR = 1;
    constexpr int RUDDER = 2;
    constexpr int TRIM = 3;
    // Buttons
    constexpr int TRIGGER = 0;
}

namespace ConXBOXControl {
    // Axis inputs
    constexpr int LEFT_JOYSTICK_X = 0;
    constexpr int LEFT_JOYSTICK_Y = 1;
    constexpr int LEFT_TRIGGER = 2;
    constexpr int RIGHT_TRIGGER = 3;
    constexpr int RIGHT_JOYSTICK_X = 4;
    constexpr int RIGHT_JOYSTICK_Y = 5;
    // Buttons
    constexpr int A = 1;
    constexpr int B = 2;
    constexpr int X = 3;
    constexpr int Y = 4;
    constexpr int LEFT_BUMPER = 5;
    constexpr int RIGHT_BUMPER = 6;
    constexpr int SELECT = 7;
    constexpr int START = 8;
    constexpr int LEFT_JOYSTICK = 9;
    constexpr int RIGHT_JOYSTICK = 10;

    // Dead zone
    constexpr double DEAD_ZONE = 0.1;

    // Drive Station Controller "Ports"
    constexpr int DRIVER_CONTROLLER_PORT = 0;
    constexpr int CODRIVER_CONTROLLER_PORT = 1;
}

// DeadZone lambda function
auto DeadZone = [] (double value) { return (std::fabs(value) > ConXBOXControl::DEAD_ZONE) ? value : 0.0; };

// Texas Instruments Controller (MSP430F5529 LaunchPad)
namespace ConLaunchPad {
    namespace Button {
        constexpr int RED = 1;  // Jumble Fwd
        constexpr int BLUE = 2; // Jumble Rev
        constexpr int YELLOW = 3; // Not Used (Climber?)
        constexpr int GREEN = 4; // Not Used (Climber?)
        constexpr int WHITE = 5; // Climber Unlock (Hold to unlock)
    }

    namespace Switch {
        constexpr int RED = 6;
        constexpr int BLUE = 7;
        constexpr int YELLOW = 8;
        constexpr int GREEN = 9; // Reset/Disable Climber Encoder
    }

    namespace Dial {
        constexpr int LEFT = 2;
        constexpr int RIGHT = 6;
    }

    constexpr int SLIDER = 42; // Unknown axis
    constexpr int LEFT_STICK_X = 1; 
    constexpr int LEFT_STICK_Y = 0;

    constexpr int RIGHT_STICK_X = 5; 
    constexpr int RIGHT_STICK_Y = 4;

    constexpr int LAUNCHPAD_CONTROLLER_PORT = 1;
}   
 */