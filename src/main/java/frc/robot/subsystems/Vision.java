// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.OI.ConShuffleboard;

public class Vision extends SubsystemBase {

    public static class ConVision {
        public static class AlignToPlayerStation {
            public static double P = 0.15;
            public static double I = 0.0;
            public static double D = 0.0;
        }
        // LED Modes: Write to NT "ledMode"
        public static int ON = 3; 
        public static int BLINK = 2;
        public static int OFF = 1;
        
        // Pipelines: 0 = Vision Processing; 1 = Driver
        public static int VISION_PIPELINE = 0;
        public static int DRIVER_PIPELINE = 1;
        
        // Stream: 1 = Primary only; 2 = Primary/2nd PIP; 3 = 2nd/Primary PIP
        public static int PRIMARY_ONLY = 1;
        public static int PRIMARY_SECONDARY_PIP = 2;
        public static int SECONDARY_PRIMARY_PIP = 3;
        
        // Camera Mode: Write to NT "camMode"
        public static int DRIVER_ONLY = 1;
        public static int VISION_TRACKING = 0;
    }
    
    // Components (e.g. motor controllers and sensors) should generally be
    // declared private and exposed only through public methods.
    NetworkTable m_nt_Limelight;
    int m_LEDStatus;
    int m_Stream;
    
    ShuffleboardTab m_sbt_Vision;
    NetworkTableEntry m_nte_Align_P;
    NetworkTableEntry m_nte_Align_I;
    NetworkTableEntry m_nte_Align_D;
    
    double m_nte_tx;

    public Vision() {
        m_nt_Limelight = NetworkTableInstance.getDefault().getTable("limelight");
        m_sbt_Vision = Shuffleboard.getTab(ConShuffleboard.VisionTab);
        m_nte_Align_P = m_sbt_Vision.addPersistent("Vision P", 1.0)
            .withSize(1, 1)
            .withPosition(0, 0)
            .getEntry();
        m_nte_Align_I = m_sbt_Vision.addPersistent("Vision I", 0.0)
            .withSize(1, 1)
            .withPosition(0, 1)
            .getEntry();
        m_nte_Align_D = m_sbt_Vision.addPersistent("Vision D", 100.0)
            .withSize(1, 1)
            .withPosition(0, 2)
            .getEntry();
    }
    
    public void initVision() {
        // #ifdef ENABLE_VISION
        // If using Vision Tracking use the following:
        // LightOn();
        // m_nt_Limelight->PutNumber("camMode", ConVision::VISION_TRACKING);
        // If using JUST for driver camera, use the following:
        // LightOff();
        m_nt_Limelight.getEntry("pipeline").setNumber(ConVision.DRIVER_PIPELINE);
        // Set PIP w/ secondary camera main view
        // PiPStream();
        // #endif // ENABLE_VISION
    }
    
    // #ifdef ENABLE_VISION
    public void primaryStream() {
        // Set PIP w/ secondary camera main view
        // m_nt_Limelight->PutNumber("camMode", ConVision::VISION_TRACKING);
        m_nt_Limelight.getEntry("stream").setNumber(ConVision.PRIMARY_ONLY);
    }
    
    public void piPStream() {
        m_nt_Limelight.getEntry("camMode").setNumber(ConVision.DRIVER_ONLY);
        // Set PIP w/ secondary camera main view
        m_nt_Limelight.getEntry("stream").setNumber(ConVision.SECONDARY_PRIMARY_PIP);
    }

    // This method will be called once per scheduler run
    public void periodic() {}
    
    public double align() {
        m_nte_tx = m_nt_Limelight.getEntry("tx").getDouble(0.0);
        return m_nte_tx;
    }
    
    public void toggleLight() {
        if (m_nt_Limelight.getEntry("ledMode").getDouble(ConVision.ON) == ConVision.OFF) {
            m_nt_Limelight.getEntry("ledMode").setNumber(ConVision.ON);
        } 
        else {
            m_nt_Limelight.getEntry("ledMode").setNumber(ConVision.OFF);
    }
    }
    
    public void lightOn() {
        m_nt_Limelight.getEntry("ledMode").setNumber(ConVision.ON);
    }
    
    public void lightOff() {
        m_nt_Limelight.getEntry("ledMode").setNumber(ConVision.OFF);
    }
    
    public void lightBlink() {
        m_nt_Limelight.getEntry("ledMode").setNumber(ConVision.BLINK);
    }

    public void selectPlayerStationPipeline() {
        m_nt_Limelight.getEntry("pipeline").setNumber(0);
    }
    
    public void selectNearGoalPipeline() {
        m_nt_Limelight.getEntry("pipeline").setNumber(1);
    }
    
    public void selectFarGoalPipeline() {
        m_nt_Limelight.getEntry("pipeline").setNumber(2);
    }
    // #endif // ENABLE_VISION

}

/** Oringinal H
 

#pragma once

#include "Constants.h"
#include <frc2/command/SubsystemBase.h>
#include <networktables/NetworkTable.h>
#include <networktables/NetworkTableInstance.h>
#include <frc/shuffleboard/Shuffleboard.h>
#include <frc/shuffleboard/ShuffleboardTab.h>
#include <networktables/NetworkTableEntry.h>

namespace ConVision {
    namespace AlignToPlayerStation {
        constexpr double P = 0.15;
        constexpr double I = 0.0;
        constexpr double D = 0.0;
    }
    // LED Modes: Write to NT "ledMode"
    constexpr int ON = 3; 
    constexpr int BLINK = 2;
    constexpr int OFF = 1;

    // Pipelines: 0 = Vision Processing; 1 = Driver
    constexpr int VISION_PIPELINE = 0;
    constexpr int DRIVER_PIPELINE = 1;
    
    // Stream: 1 = Primary only; 2 = Primary/2nd PIP; 3 = 2nd/Primary PIP
    constexpr int PRIMARY_ONLY = 1;
    constexpr int PRIMARY_SECONDARY_PIP = 2;
    constexpr int SECONDARY_PRIMARY_PIP = 3;

    // Camera Mode: Write to NT "camMode"
    constexpr int DRIVER_ONLY = 1;
    constexpr int VISION_TRACKING = 0;
}

class Vision : public frc2::SubsystemBase {
 public:
  Vision();

  void InitVision();

#ifdef ENABLE_VISION
  void Periodic();
  double Align();
  void ToggleLight();
  void LightOn();
  void LightOff();
  void LightBlink();
  void PrimaryStream();
  void PiPStream();

  void SelectPlayerStationPipeline();
  void SelectNearGoalPipeline();
  void SelectFarGoalPipeline();

#endif // ENABLE_VISION

 private:
  // Components (e.g. motor controllers and sensors) should generally be
  // declared private and exposed only through public methods.
  std::shared_ptr<nt::NetworkTable> m_nt_Limelight;
  int m_LEDStatus;
  int m_Stream;

 public:
  frc::ShuffleboardTab *m_sbt_Vision;
  nt::NetworkTableEntry m_nte_Align_P;
  nt::NetworkTableEntry m_nte_Align_I;
  nt::NetworkTableEntry m_nte_Align_D;

  double m_nte_tx;
};

*/

/** Original CPP

#include "subsystems/Vision.h"
#include "OI.h"

Vision::Vision() {
    m_nt_Limelight = nt::NetworkTableInstance::GetDefault().GetTable("limelight");
    m_sbt_Vision = &frc::Shuffleboard::GetTab(ConShuffleboard::VisionTab);
    m_nte_Align_P = m_sbt_Vision->AddPersistent("Vision P", 1.0)  .WithSize(1, 1).WithPosition(0, 0).GetEntry();;
    m_nte_Align_I = m_sbt_Vision->AddPersistent("Vision I", 0.0)  .WithSize(1, 1).WithPosition(0, 1).GetEntry();;
    m_nte_Align_D = m_sbt_Vision->AddPersistent("Vision D", 100.0).WithSize(1, 1).WithPosition(0, 2).GetEntry();;
}

void Vision::InitVision() {
#ifdef ENABLE_VISION
    // If using Vision Tracking use the following:
    // LightOn();
    // m_nt_Limelight->PutNumber("camMode", ConVision::VISION_TRACKING);
    // If using JUST for driver camera, use the following:
    // LightOff();
    m_nt_Limelight->PutNumber("pipeline", ConVision::DRIVER_PIPELINE);
    // Set PIP w/ secondary camera main view
    // PiPStream();
#endif // ENABLE_VISION
}

#ifdef ENABLE_VISION
void Vision::PrimaryStream() {
    // Set PIP w/ secondary camera main view
    // m_nt_Limelight->PutNumber("camMode", ConVision::VISION_TRACKING);
    m_nt_Limelight->PutNumber("stream", ConVision::PRIMARY_ONLY);
}

void Vision::PiPStream() {
    m_nt_Limelight->PutNumber("camMode", ConVision::DRIVER_ONLY);
    // Set PIP w/ secondary camera main view
    m_nt_Limelight->PutNumber("stream", ConVision::SECONDARY_PRIMARY_PIP);
}
// This method will be called once per scheduler run
void Vision::Periodic() {}

double Vision::Align() {
    m_nte_tx = m_nt_Limelight->GetNumber("tx", 0.0);
    return m_nte_tx;
}

void Vision::ToggleLight() {
    if (m_nt_Limelight->GetNumber("ledMode", ConVision::ON) == ConVision::OFF) {
        m_nt_Limelight->PutNumber("ledMode", ConVision::ON);
    } 
    else {
        m_nt_Limelight->PutNumber("ledMode", ConVision::OFF);
    }
}

void Vision::LightOn() {
    m_nt_Limelight->PutNumber("ledMode", ConVision::ON);
}

void Vision::LightOff() {
    m_nt_Limelight->PutNumber("ledMode", ConVision::OFF);
}

void Vision::LightBlink() {
    m_nt_Limelight->PutNumber("ledMode", ConVision::BLINK);
}
void Vision::SelectPlayerStationPipeline() {
    m_nt_Limelight->PutNumber("pipeline", 0);
}

void Vision::SelectNearGoalPipeline() {
    m_nt_Limelight->PutNumber("pipeline", 1);
}

void Vision::SelectFarGoalPipeline() {
    m_nt_Limelight->PutNumber("pipeline", 2);
}
#endif // ENABLE_VISION

*/