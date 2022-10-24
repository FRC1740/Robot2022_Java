// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ConSparkMax;
import frc.robot.OI.ConShuffleboard;

public class Launcher extends SubsystemBase {

  public static class ConLauncher {
    public static int MOTOR_BERT_ID = 6;
    public static int MOTOR_ERNIE_ID = 1;
    // Starting point for Launcher soft limits
    public static int BERT_FWD_LIMIT = 116;
    public static int ERNIE_FWD_LIMIT = 115;
    public static int BERT_REV_LIMIT = 0;
    public static int ERNIE_REV_LIMIT = 0;  
    public static int BERT_FAR_LIMIT = 176;
    public static int ERNIE_FAR_LIMIT = 191;
    public static double BERT_RAMP_RATE = .05;
    public static double ERNIE_RAMP_RATE = .05;
    public static double BERT_POWER = .96;
    public static double ERNIE_POWER = .92;
    public static double BERT_FAR_POWER = .77;
    public static double ERNIE_FAR_POWER = .78;
    public static int CURRENT_STALL_LIMIT = 80;
    public static double DOUBLE_LAUNCH_PWR_SCALE_FACTOR = 1.05;
  }
  
  ShuffleboardTab m_sbt_Launcher;
  // Inputs from Dashboard
  NetworkTableEntry m_nte_Bert_FwdLimit; // LAUNCHER forward may be MOTOR REVERSE!!!
  NetworkTableEntry m_nte_Bert_RevLimit; // Ditto for the reverse
  NetworkTableEntry m_nte_Ernie_FwdLimit; // LAUNCHER forward may be MOTOR REVERSE!!!
  NetworkTableEntry m_nte_Ernie_RevLimit; // Ditto for the reverse

  NetworkTableEntry m_nte_Bert_FarLimit; // Alternate
  NetworkTableEntry m_nte_Ernie_FarLimit; // Alternate

  NetworkTableEntry m_nte_Ernie_Power; // 0.0 to 1.0 (max)
  NetworkTableEntry m_nte_Bert_Power; // Ditto

  NetworkTableEntry m_nte_Ernie_FarPower; // Alternate
  NetworkTableEntry m_nte_Bert_FarPower; // Alternate

  NetworkTableEntry m_nte_Bert_Ramp_Rate;
  NetworkTableEntry m_nte_Ernie_Ramp_Rate;

  // Outputs to Dashboard
  NetworkTableEntry m_nte_Ernie_Position; // Encoder feedback
  NetworkTableEntry m_nte_Bert_Position; // Ditto
  NetworkTableEntry m_nte_Bert_Voltage;
  NetworkTableEntry m_nte_Ernie_Voltage;

  // #ifdef LAUNCHER_VELOCITY_CONTROL
    // Velocity Control- see
    // https://github.com/REVrobotics/SPARK-MAX-Examples/tree/master/C%2B%2B/Velocity%20PID%20Control
    NetworkTableEntry m_nte_Launcher_P__Gain;
    NetworkTableEntry m_nte_Launcher_I_Gain;
    NetworkTableEntry m_nte_Launcher_D_Gain;
    NetworkTableEntry m_nte_Launcher_I_Zone;
    NetworkTableEntry m_nte_Launcher_Feed_Forward;
    NetworkTableEntry m_nte_Launcher_Max_Output;
    NetworkTableEntry m_nte_Launcher_Min_Output;
  // #endif
  
  // #ifdef ENABLE_LAUNCHER
    // NEO motor
    CANSparkMax m_launcherMotorBert = new CANSparkMax(ConLauncher.MOTOR_BERT_ID, CANSparkMax.MotorType.kBrushless);
    CANSparkMax m_launcherMotorErnie = new CANSparkMax(ConLauncher.MOTOR_ERNIE_ID, CANSparkMax.MotorType.kBrushless);
  
    // built-in encoders
    RelativeEncoder m_launcherEncoderBert = m_launcherMotorBert.getEncoder();
    RelativeEncoder m_launcherEncoderErnie = m_launcherMotorErnie.getEncoder();
  
  // #ifdef LAUNCHER_VELOCITY_CONTROL
    SparkMaxPIDController m_pidControllerBert = m_launcherMotorBert.getPIDController();
    SparkMaxPIDController m_pidControllerErnie = m_launcherMotorErnie.getPIDController();
  // #endif
  
  // #endif
    boolean m_useClose = true;
    
    // Power Settings for each launcher
    double m_ErnieFwdPower;
    double m_BertFwdPower;
  
    double m_ErnieFwdLimit;
    double m_BertFwdLimit;
  
    // Alternate settings
    double m_ErnieFarPower;
    double m_BertFarPower;
  
    double m_ErnieFarLimit;
    double m_BertFarLimit;  
  
  public Launcher() {
    
    // Initialize stuff here
    // #ifdef ENABLE_LAUNCHER
    // #ifdef LAUNCHER_VELOCITY_CONTROL
    double kP = 6e-5, kI = 1e-6, kD = 0, kIz = 0, kFF = 0.000015, kMaxOutput = 1.0, kMinOutput = -1.0;
    m_launcherMotorBert.restoreFactoryDefaults();
    m_launcherMotorErnie.restoreFactoryDefaults();

    // set PID coefficients
    m_pidControllerBert.setP(kP);
    m_pidControllerBert.setI(kI);
    m_pidControllerBert.setD(kD);
    m_pidControllerBert.setIZone(kIz);
    m_pidControllerBert.setFF(kFF);
    m_pidControllerBert.setOutputRange(kMinOutput, kMaxOutput);

    m_pidControllerErnie.setP(kP);
    m_pidControllerErnie.setI(kI);
    m_pidControllerErnie.setD(kD);
    m_pidControllerErnie.setIZone(kIz);
    m_pidControllerErnie.setFF(kFF);
    m_pidControllerErnie.setOutputRange(kMinOutput, kMaxOutput);
    // #endif

    // All of our Encoders are based on a tick count of 42
    m_launcherEncoderBert.setPositionConversionFactor(ConSparkMax.POSITION_CONVERSION_FACTOR); // Generally 42
    m_launcherEncoderErnie.setPositionConversionFactor(ConSparkMax.POSITION_CONVERSION_FACTOR); // Generally 42

    // I believe brake mode is the default, but...
    m_launcherMotorBert.setIdleMode(CANSparkMax.IdleMode.kBrake);
    m_launcherMotorErnie.setIdleMode(CANSparkMax.IdleMode.kBrake);

    // No hardware limits on these controllers

    m_launcherEncoderBert.setPosition(0.0);
    m_launcherEncoderErnie.setPosition(0.0);
    // Enable & Set Encoder Soft Limits...
    m_launcherMotorBert.enableSoftLimit(CANSparkMax.SoftLimitDirection.kForward, true);
    m_launcherMotorBert.enableSoftLimit(CANSparkMax.SoftLimitDirection.kReverse, true);
    m_launcherMotorErnie.enableSoftLimit(CANSparkMax.SoftLimitDirection.kForward, true);
    m_launcherMotorErnie.enableSoftLimit(CANSparkMax.SoftLimitDirection.kReverse, true);
    // Forward Limits
    m_launcherMotorBert.setSoftLimit(CANSparkMax.SoftLimitDirection.kForward, (float) m_nte_Bert_FwdLimit.getDouble(ConLauncher.BERT_FWD_LIMIT));
    m_launcherMotorBert.setSoftLimit(CANSparkMax.SoftLimitDirection.kReverse, (float) m_nte_Bert_RevLimit.getDouble(ConLauncher.BERT_REV_LIMIT));
    m_launcherMotorErnie.setSoftLimit(CANSparkMax.SoftLimitDirection.kForward, (float) m_nte_Ernie_FwdLimit.getDouble(ConLauncher.ERNIE_FWD_LIMIT));
    m_launcherMotorErnie.setSoftLimit(CANSparkMax.SoftLimitDirection.kReverse, (float) m_nte_Ernie_RevLimit.getDouble(ConLauncher.ERNIE_REV_LIMIT));

    // Naturally, Bert is Backwards...
    m_launcherMotorBert.setInverted(true);
    m_launcherMotorErnie.setInverted(false);

    // Set Current limits (Stall, Free)
    m_launcherMotorBert.setSmartCurrentLimit(ConLauncher.CURRENT_STALL_LIMIT, ConLauncher.CURRENT_STALL_LIMIT);
    m_launcherMotorErnie.setSmartCurrentLimit(ConLauncher.CURRENT_STALL_LIMIT, ConLauncher.CURRENT_STALL_LIMIT);

    // #endif            

    m_sbt_Launcher = Shuffleboard.getTab(ConShuffleboard.LauncherTab);

    // Robot outputs -> Shuffleboard
    m_nte_Bert_Position = m_sbt_Launcher.addPersistent("Bert Pos", ConLauncher.BERT_REV_LIMIT)
          .withSize(1,1)
          .withPosition(0,0)
          .getEntry();
    m_nte_Ernie_Position = m_sbt_Launcher.addPersistent("Ernie Pos", ConLauncher.ERNIE_REV_LIMIT)
          .withSize(1,1)
          .withPosition(0,1)
          .getEntry();

    // Robot inputs <- Shuffleboard
    m_nte_Bert_FwdLimit = m_sbt_Launcher.addPersistent("Bert Fwd Limit", ConLauncher.BERT_FWD_LIMIT)
          .withSize(1,1)
          .withPosition(2,0)
          .getEntry();
    m_nte_Ernie_FwdLimit = m_sbt_Launcher.addPersistent("Ernie Fwd Limit", ConLauncher.ERNIE_FWD_LIMIT)
          .withSize(1,1)
          .withPosition(2,1)
          .getEntry();
    m_nte_Bert_FarLimit = m_sbt_Launcher.addPersistent("Bert Far Limit", ConLauncher.BERT_FAR_LIMIT)
          .withSize(1,1)
          .withPosition(3,0)
          .getEntry();
    m_nte_Ernie_FarLimit = m_sbt_Launcher.addPersistent("Ernie Far Limit", ConLauncher.ERNIE_FAR_LIMIT)
          .withSize(1,1)
          .withPosition(3,1)
          .getEntry();
    m_nte_Bert_Power = m_sbt_Launcher.addPersistent("Bert Power", ConLauncher.BERT_POWER)
          .withSize(1,1)
          .withPosition(4,0)
          .getEntry();
    m_nte_Ernie_Power = m_sbt_Launcher.addPersistent("Ernie Power", ConLauncher.ERNIE_POWER)
          .withSize(1,1)
          .withPosition(4,1)
          .getEntry();
    m_nte_Bert_FarPower = m_sbt_Launcher.addPersistent("Bert Far Power", ConLauncher.BERT_FAR_POWER)
          .withSize(1,1)
          .withPosition(5,0)
          .getEntry();
    m_nte_Ernie_FarPower = m_sbt_Launcher.addPersistent("Ernie Far Power", ConLauncher.ERNIE_FAR_POWER)
          .withSize(1,1)
          .withPosition(5,1)
          .getEntry();
    m_nte_Bert_Voltage = m_sbt_Launcher.addPersistent("Bert Voltage", 0.0)
          .withSize(2,1)
          .withPosition(3,2)
          .withWidget(BuiltInWidgets.kDial)
          .getEntry();
    m_nte_Ernie_Voltage = m_sbt_Launcher.addPersistent("Ernie Voltage", 0.0)
          .withSize(2,1)
          .withPosition(3,3)
          .withWidget(BuiltInWidgets.kDial)
          .getEntry();

    // #ifdef LAUNCHER_VELOCITY_CONTROL
    // display PID coefficients on Shuffleboard
    m_nte_Launcher_P__Gain = m_sbt_Launcher.addPersistent("Launcher P Gain", kP)
          .withSize(2,1)
          .withPosition(6,0)
          .getEntry();
    m_nte_Launcher_I_Gain = m_sbt_Launcher.addPersistent("Launcher I Gain", kI)
          .withSize(2,1)
          .withPosition(6,1)
          .getEntry();
    m_nte_Launcher_D_Gain = m_sbt_Launcher.addPersistent("Launcher D Gain", kD)
          .withSize(2,1)
          .withPosition(6,2)
          .getEntry();
    m_nte_Launcher_I_Zone = m_sbt_Launcher.addPersistent("Launcher Iz Gain", kIz)
          .withSize(2,1)
          .withPosition(6,4)
          .getEntry();
    m_nte_Launcher_Min_Output = m_sbt_Launcher.addPersistent("Launcher Min Output", kMinOutput)
          .withSize(2,1)
          .withPosition(8,0)
          .getEntry();
    m_nte_Launcher_Max_Output = m_sbt_Launcher.addPersistent("Launcher Max Output", kMaxOutput)
          .withSize(2,1)
          .withPosition(8,1)
          .getEntry();
    m_nte_Launcher_Feed_Forward = m_sbt_Launcher.addPersistent("Launcher Feed Forward", kFF)
          .withSize(2,1)
          .withPosition(8,3)
          .getEntry();
    // #endif

    m_ErnieFwdPower = ConLauncher.ERNIE_POWER;
    m_BertFwdPower = ConLauncher.BERT_POWER;
    m_ErnieFarPower = ConLauncher.ERNIE_FAR_POWER;
    m_BertFarPower = ConLauncher.BERT_FAR_POWER;

    // Ensure the launcher is in the retracted position
    retract();
  }
 
  public void launch() {
    launchBert(ConLauncher.DOUBLE_LAUNCH_PWR_SCALE_FACTOR);
    launchErnie(ConLauncher.DOUBLE_LAUNCH_PWR_SCALE_FACTOR);
  }

  public void launchBert(double pwrScale) {
    if(m_useClose) {
      System.out.println("LaunchBert() Power " + m_BertFwdPower + "Limit " + m_BertFwdLimit);
    } else {
      System.out.println("LaunchBert() Power " + m_BertFarPower + "Limit " + m_BertFarLimit);
    }
    // Launch a ball
    //#ifdef ENABLE_LAUNCHER
    if (m_useClose) {
      m_launcherMotorBert.set(m_BertFwdPower * pwrScale);
    } else {
      m_launcherMotorBert.set(m_BertFarPower * pwrScale);
    }
    System.out.println("Battery Voltage: " + RobotController.getBatteryVoltage());
    //#endif
  }

  public void launchErnie(double pwrScale) {
    if(m_useClose) {
      System.out.println("LaunchErnie() Power " + m_ErnieFwdPower + "Limit " + m_ErnieFwdLimit);
    } else {
      System.out.println("LaunchErnie() Power " + m_ErnieFarPower + "Limit " + m_ErnieFarLimit);
    } 
    // Launch a ball
    // #ifdef ENABLE_LAUNCHER
    if (m_useClose) {
      m_launcherMotorErnie.set(m_ErnieFwdPower * pwrScale);
    } else {
      m_launcherMotorErnie.set(m_ErnieFarPower * pwrScale);
    }
    System.out.println("Battery Voltage: " + RobotController.getBatteryVoltage());
    // #endif
  }

  public void retract() {
    // Bring both launchers back
    retractBert();
    retractErnie();
  }

  public void retractBert() {
    System.out.println("Launcher::RetractBert() Executing...");  
    // #ifdef ENABLE_LAUNCHER
    m_launcherMotorBert.set(-.1);
    // #endif
  }

  public void retractErnie() {
    System.out.println("Launcher::RetractErnie() Executing...");  
    // #ifdef ENABLE_LAUNCHER
    m_launcherMotorErnie.set(-.1);
    // #endif
  }

  public void setupFar(){
    System.out.println("Launcher::SetupFar()");
    m_useClose = false;
    // #ifdef ENABLE_LAUNCHER
    m_launcherMotorErnie.setSoftLimit(CANSparkMax.SoftLimitDirection.kForward, (float) m_ErnieFarLimit);
    m_launcherMotorBert.setSoftLimit(CANSparkMax.SoftLimitDirection.kForward, (float) m_BertFarLimit);
    // #endif // ENABLE_LAUNCHER
  }
  
  public void setupClose() {
    System.out.println("Launcher::SetupClose()");
    m_useClose = true;
    // #ifdef ENABLE_LAUNCHER
    m_launcherMotorErnie.setSoftLimit(CANSparkMax.SoftLimitDirection.kForward, (float) m_ErnieFwdLimit);
    m_launcherMotorBert.setSoftLimit(CANSparkMax.SoftLimitDirection.kForward, (float) m_BertFwdLimit);
    // #endif // ENABLE_LAUNCHER
  }

  public void setLaunchSoftLimits() {
    double d;

    // Close Power
    d = m_nte_Ernie_Power.getDouble(ConLauncher.ERNIE_POWER);
    if (d != m_ErnieFwdPower) {
      m_ErnieFwdPower = d;
      System.out.println("m_ErnieFwdPower set to " + m_ErnieFwdPower);
    }
    d = m_nte_Bert_Power.getDouble(ConLauncher.BERT_POWER);
    if (d != m_BertFwdPower) {
      m_BertFwdPower = d;
      System.out.println("m_BertFwdPower set to " + m_BertFwdPower);
    }

    // Close Limits
    d = m_nte_Ernie_FwdLimit.getDouble(ConLauncher.ERNIE_FWD_LIMIT);
    if (d != m_ErnieFwdLimit) {
      m_ErnieFwdLimit = d;
      System.out.println("m_ErnieFwdLimit set to " + m_ErnieFwdLimit);
    }
    d = m_nte_Bert_FwdLimit.getDouble(ConLauncher.BERT_FWD_LIMIT);
    if (d != m_BertFwdLimit) {
      m_BertFwdLimit = d;
      System.out.println("m_BertFwdLimit set to " + m_BertFwdLimit);
    }

    // Far Power
    d = m_nte_Ernie_FarPower.getDouble(ConLauncher.ERNIE_FAR_POWER);
    if (d != m_ErnieFarPower) {
      m_ErnieFarPower = d;
      System.out.println("m_ErnieFarPower set to " + m_ErnieFarPower);
    }
    d = m_nte_Bert_FarPower.getDouble(ConLauncher.BERT_FAR_POWER);
    if (d != m_BertFarPower) {
      m_BertFarPower = d;
      System.out.println("m_BertFarPower set to " + m_BertFarPower);
    }

    // Far Limits
    d = m_nte_Ernie_FarLimit.getDouble(ConLauncher.ERNIE_FAR_LIMIT);
    if (d != m_ErnieFarLimit) {
      m_ErnieFarLimit = d;
      System.out.println("m_ErnieFarLimit set to " + m_ErnieFarLimit);
    }
    d = m_nte_Bert_FarLimit.getDouble(ConLauncher.BERT_FAR_LIMIT);
    if (d != m_BertFarLimit) {
      m_BertFarLimit = d;
      System.out.println("m_BertFarLimit set to " + m_BertFarLimit);
    }
  }

  public void setResetSoftLimits() {
  }
  
  public void stop() {
    // #ifdef ENABLE_LAUNCHER
    m_launcherMotorErnie.set(0.0);
    m_launcherMotorBert.set(0.0);
    // #endif
  }
  
  public void periodic() {
    // #ifdef ENABLE_LAUNCHER
    // Display
    m_nte_Bert_Position.setDouble(m_launcherEncoderBert.getPosition());
    m_nte_Ernie_Position.setDouble(m_launcherEncoderErnie.getPosition());
    m_nte_Bert_Voltage.setDouble(m_launcherMotorErnie.getBusVoltage());
    m_nte_Ernie_Voltage.setDouble(m_launcherMotorErnie.getBusVoltage());
    // #endif
  }
  
  public void burnFlash() {
    System.out.println("BurnFlash for Launchers");
    // #ifdef ENABLE_LAUNCHER
    // Save the configuration to flash memory
    m_launcherMotorErnie.burnFlash();
    m_launcherMotorBert.burnFlash();
    // #endif // ENABLE_LAUNCHER
  }
  
}

/** Original H
 // Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/SubsystemBase.h>
#include <rev/CANSparkMax.h>
#include <frc/shuffleboard/Shuffleboard.h>
#include <frc/shuffleboard/ShuffleboardTab.h>
#include <networktables/NetworkTableEntry.h>
#include "Constants.h"

namespace ConLauncher {
  constexpr int MOTOR_BERT_ID = 6;
  constexpr int MOTOR_ERNIE_ID = 1;
  // Starting point for Launcher soft limits
  constexpr int BERT_FWD_LIMIT = 116;
  constexpr int ERNIE_FWD_LIMIT = 115;
  constexpr int BERT_REV_LIMIT = 0;
  constexpr int ERNIE_REV_LIMIT = 0;  
  constexpr int BERT_FAR_LIMIT = 176;
  constexpr int ERNIE_FAR_LIMIT = 191;
  constexpr double BERT_RAMP_RATE = .05;
  constexpr double ERNIE_RAMP_RATE = .05;
  constexpr double BERT_POWER = .96;
  constexpr double ERNIE_POWER = .92;
  constexpr double BERT_FAR_POWER = .77;
  constexpr double ERNIE_FAR_POWER = .78;
  constexpr int CURRENT_STALL_LIMIT = 80;
  constexpr double DOUBLE_LAUNCH_PWR_SCALE_FACTOR = 1.05;
}

class Launcher : public frc2::SubsystemBase {
 public:
  Launcher();
  void LaunchBert(double);
  void LaunchErnie(double);
  void Launch(); // Launch both Bert & Ernie
  void RetractBert();
  void RetractErnie();
  void Retract(); // Move both catapaults back to starting position
  void Stop(); // Stop launcher motors
  void SetupFar();
  void SetupClose();
  void SetLaunchSoftLimits();
  void SetResetSoftLimits();
  void BurnFlash();

  frc::ShuffleboardTab *m_sbt_Launcher;
  // Inputs from Dashboard
  nt::NetworkTableEntry m_nte_Bert_FwdLimit; // LAUNCHER forward may be MOTOR REVERSE!!!
  nt::NetworkTableEntry m_nte_Bert_RevLimit; // Ditto for the reverse
  nt::NetworkTableEntry m_nte_Ernie_FwdLimit; // LAUNCHER forward may be MOTOR REVERSE!!!
  nt::NetworkTableEntry m_nte_Ernie_RevLimit; // Ditto for the reverse

  nt::NetworkTableEntry m_nte_Bert_FarLimit; // Alternate
  nt::NetworkTableEntry m_nte_Ernie_FarLimit; // Alternate

  nt::NetworkTableEntry m_nte_Ernie_Power; // 0.0 to 1.0 (max)
  nt::NetworkTableEntry m_nte_Bert_Power; // Ditto

  nt::NetworkTableEntry m_nte_Ernie_FarPower; // Alternate
  nt::NetworkTableEntry m_nte_Bert_FarPower; // Alternate

  nt::NetworkTableEntry m_nte_Bert_Ramp_Rate;
  nt::NetworkTableEntry m_nte_Ernie_Ramp_Rate;

  // Outputs to Dashboard
  nt::NetworkTableEntry m_nte_Ernie_Position; // Encoder feedback
  nt::NetworkTableEntry m_nte_Bert_Position; // Ditto
  nt::NetworkTableEntry m_nte_Bert_Voltage;
  nt::NetworkTableEntry m_nte_Ernie_Voltage;

#ifdef LAUNCHER_VELOCITY_CONTROL
  // Velocity Control- see
  // https://github.com/REVrobotics/SPARK-MAX-Examples/tree/master/C%2B%2B/Velocity%20PID%20Control
  nt::NetworkTableEntry m_nte_Launcher_P__Gain;
  nt::NetworkTableEntry m_nte_Launcher_I_Gain;
  nt::NetworkTableEntry m_nte_Launcher_D_Gain;
  nt::NetworkTableEntry m_nte_Launcher_I_Zone;
  nt::NetworkTableEntry m_nte_Launcher_Feed_Forward;
  nt::NetworkTableEntry m_nte_Launcher_Max_Output;
  nt::NetworkTableEntry m_nte_Launcher_Min_Output;
#endif

 protected:
#ifdef ENABLE_LAUNCHER
  // NEO motor
  rev::CANSparkMax m_launcherMotorBert{ConLauncher::MOTOR_BERT_ID, rev::CANSparkMax::MotorType::kBrushless};
  rev::CANSparkMax m_launcherMotorErnie{ConLauncher::MOTOR_ERNIE_ID, rev::CANSparkMax::MotorType::kBrushless};

  // built-in encoders
  rev::SparkMaxRelativeEncoder m_launcherEncoderBert = m_launcherMotorBert.GetEncoder();
  rev::SparkMaxRelativeEncoder m_launcherEncoderErnie = m_launcherMotorErnie.GetEncoder();

#ifdef LAUNCHER_VELOCITY_CONTROL
  rev::SparkMaxPIDController m_pidControllerBert = m_launcherMotorBert.GetPIDController();
  rev::SparkMaxPIDController m_pidControllerErnie = m_launcherMotorErnie.GetPIDController();
#endif

#endif
  bool m_useClose = true;
  
  // Power Settings for each launcher
  double m_ErnieFwdPower;
  double m_BertFwdPower;

  double m_ErnieFwdLimit;
  double m_BertFwdLimit;

  // Alternate settings
  double m_ErnieFarPower;
  double m_BertFarPower;

  double m_ErnieFarLimit;
  double m_BertFarLimit;  

  void Periodic();
};

 */

/** Original CPP
 // Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include <frc2/command/SubsystemBase.h>
#include <frc/DriverStation.h>
#include "subsystems/Launcher.h"
#include "OI.h"

Launcher::Launcher() {
            // Initialize stuff here
#ifdef ENABLE_LAUNCHER
#ifdef LAUNCHER_VELOCITY_CONTROL
    double kP = 6e-5, kI = 1e-6, kD = 0, kIz = 0, kFF = 0.000015, kMaxOutput = 1.0, kMinOutput = -1.0;
    m_launcherMotorBert.RestoreFactoryDefaults();
    m_launcherMotorErnie.RestoreFactoryDefaults();
    
    // set PID coefficients
    m_pidControllerBert.SetP(kP);
    m_pidControllerBert.SetI(kI);
    m_pidControllerBert.SetD(kD);
    m_pidControllerBert.SetIZone(kIz);
    m_pidControllerBert.SetFF(kFF);
    m_pidControllerBert.SetOutputRange(kMinOutput, kMaxOutput);

    m_pidControllerErnie.SetP(kP);
    m_pidControllerErnie.SetI(kI);
    m_pidControllerErnie.SetD(kD);
    m_pidControllerErnie.SetIZone(kIz);
    m_pidControllerErnie.SetFF(kFF);
    m_pidControllerErnie.SetOutputRange(kMinOutput, kMaxOutput);
#endif
            // All of our Encoders are based on a tick count of 42
            m_launcherEncoderBert.SetPositionConversionFactor(ConSparkMax::POSITION_CONVERSION_FACTOR); // Generally 42
            m_launcherEncoderErnie.SetPositionConversionFactor(ConSparkMax::POSITION_CONVERSION_FACTOR); // Generally 42

            // I believe brake mode is the default, but...
            m_launcherMotorBert.SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);
            m_launcherMotorErnie.SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);

            // No hardware limits on these controllers

            m_launcherEncoderBert.SetPosition(0.0);
            m_launcherEncoderErnie.SetPosition(0.0);
            // Enable & Set Encoder Soft Limits...
            m_launcherMotorBert.EnableSoftLimit(rev::CANSparkMax::SoftLimitDirection::kForward, true);
            m_launcherMotorBert.EnableSoftLimit(rev::CANSparkMax::SoftLimitDirection::kReverse, true);
            m_launcherMotorErnie.EnableSoftLimit(rev::CANSparkMax::SoftLimitDirection::kForward, true);
            m_launcherMotorErnie.EnableSoftLimit(rev::CANSparkMax::SoftLimitDirection::kReverse, true);
            // Forward Limits
            m_launcherMotorBert.SetSoftLimit(rev::CANSparkMax::SoftLimitDirection::kForward, m_nte_Bert_FwdLimit.GetDouble(ConLauncher::BERT_FWD_LIMIT));
            m_launcherMotorBert.SetSoftLimit(rev::CANSparkMax::SoftLimitDirection::kReverse, m_nte_Bert_RevLimit.GetDouble(ConLauncher::BERT_REV_LIMIT));
            m_launcherMotorErnie.SetSoftLimit(rev::CANSparkMax::SoftLimitDirection::kForward, m_nte_Ernie_FwdLimit.GetDouble(ConLauncher::ERNIE_FWD_LIMIT));
            m_launcherMotorErnie.SetSoftLimit(rev::CANSparkMax::SoftLimitDirection::kReverse, m_nte_Ernie_RevLimit.GetDouble(ConLauncher::ERNIE_REV_LIMIT));

            // Naturally, Bert is Backwards...
            m_launcherMotorBert.SetInverted(true);
            m_launcherMotorErnie.SetInverted(false);

            // Set Current limits (Stall, Free)
            m_launcherMotorBert.SetSmartCurrentLimit(ConLauncher::CURRENT_STALL_LIMIT, ConLauncher::CURRENT_STALL_LIMIT);
            m_launcherMotorErnie.SetSmartCurrentLimit(ConLauncher::CURRENT_STALL_LIMIT, ConLauncher::CURRENT_STALL_LIMIT);

#endif            

            m_sbt_Launcher = &frc::Shuffleboard::GetTab(ConShuffleboard::LauncherTab);

            // Robot outputs -> Shuffleboard
            m_nte_Bert_Position = m_sbt_Launcher->AddPersistent("Bert Pos", ConLauncher::BERT_REV_LIMIT)
                  .WithSize(1,1)
                  .WithPosition(0,0)
                  .GetEntry();
            m_nte_Ernie_Position = m_sbt_Launcher->AddPersistent("Ernie Pos", ConLauncher::ERNIE_REV_LIMIT)
                  .WithSize(1,1)
                  .WithPosition(0,1)
                  .GetEntry();

            // Robot inputs <- Shuffleboard
            m_nte_Bert_FwdLimit = m_sbt_Launcher->AddPersistent("Bert Fwd Limit", ConLauncher::BERT_FWD_LIMIT)
                  .WithSize(1,1)
                  .WithPosition(2,0)
                  .GetEntry();
            m_nte_Ernie_FwdLimit = m_sbt_Launcher->AddPersistent("Ernie Fwd Limit", ConLauncher::ERNIE_FWD_LIMIT)
                  .WithSize(1,1)
                  .WithPosition(2,1)
                  .GetEntry();
            m_nte_Bert_FarLimit = m_sbt_Launcher->AddPersistent("Bert Far Limit", ConLauncher::BERT_FAR_LIMIT)
                  .WithSize(1,1)
                  .WithPosition(3,0)
                  .GetEntry();
            m_nte_Ernie_FarLimit = m_sbt_Launcher->AddPersistent("Ernie Far Limit", ConLauncher::ERNIE_FAR_LIMIT)
                  .WithSize(1,1)
                  .WithPosition(3,1)
                  .GetEntry();
            m_nte_Bert_Power = m_sbt_Launcher->AddPersistent("Bert Power", ConLauncher::BERT_POWER)
                  .WithSize(1,1)
                  .WithPosition(4,0)
                  .GetEntry();
            m_nte_Ernie_Power = m_sbt_Launcher->AddPersistent("Ernie Power", ConLauncher::ERNIE_POWER)
                  .WithSize(1,1)
                  .WithPosition(4,1)
                  .GetEntry();
            m_nte_Bert_FarPower = m_sbt_Launcher->AddPersistent("Bert Far Power", ConLauncher::BERT_FAR_POWER)
                  .WithSize(1,1)
                  .WithPosition(5,0)
                  .GetEntry();
            m_nte_Ernie_FarPower = m_sbt_Launcher->AddPersistent("Ernie Far Power", ConLauncher::ERNIE_FAR_POWER)
                  .WithSize(1,1)
                  .WithPosition(5,1)
                  .GetEntry();
            m_nte_Bert_Voltage = m_sbt_Launcher->AddPersistent("Bert Voltage", 0.0)
                  .WithSize(2,1)
                  .WithPosition(3,2)
                  .WithWidget(frc::BuiltInWidgets::kDial)
                  .GetEntry();
            m_nte_Ernie_Voltage = m_sbt_Launcher->AddPersistent("Ernie Voltage", 0.0)
                  .WithSize(2,1)
                  .WithPosition(3,3)
                  .WithWidget(frc::BuiltInWidgets::kDial)
                  .GetEntry();

#ifdef LAUNCHER_VELOCITY_CONTROL
  // display PID coefficients on Shuffleboard
  m_nte_Launcher_P__Gain = m_sbt_Launcher->AddPersistent("Launcher P Gain", kP)
                  .WithSize(2,1)
                  .WithPosition(6,0)
                  .GetEntry();
  m_nte_Launcher_I_Gain = m_sbt_Launcher->AddPersistent("Launcher I Gain", kI)
                  .WithSize(2,1)
                  .WithPosition(6,1)
                  .GetEntry();
  m_nte_Launcher_D_Gain = m_sbt_Launcher->AddPersistent("Launcher D Gain", kD)
                  .WithSize(2,1)
                  .WithPosition(6,2)
                  .GetEntry();
  m_nte_Launcher_I_Zone = m_sbt_Launcher->AddPersistent("Launcher Iz Gain", kIz)
                  .WithSize(2,1)
                  .WithPosition(6,4)
                  .GetEntry();
  m_nte_Launcher_Min_Output = m_sbt_Launcher->AddPersistent("Launcher Min Output", kMinOutput)
                  .WithSize(2,1)
                  .WithPosition(8,0)
                  .GetEntry();
  m_nte_Launcher_Max_Output = m_sbt_Launcher->AddPersistent("Launcher Max Output", kMaxOutput)
                  .WithSize(2,1)
                  .WithPosition(8,1)
                  .GetEntry();
  m_nte_Launcher_Feed_Forward = m_sbt_Launcher->AddPersistent("Launcher Feed Forward", kFF)
                  .WithSize(2,1)
                  .WithPosition(8,3)
                  .GetEntry();
#endif

            m_ErnieFwdPower = ConLauncher::ERNIE_POWER;
            m_BertFwdPower = ConLauncher::BERT_POWER;
            m_ErnieFarPower = ConLauncher::ERNIE_FAR_POWER;
            m_BertFarPower = ConLauncher::BERT_FAR_POWER;

            // Ensure the launcher is in the retracted position
            Retract();
          }

void Launcher::Launch() {
  LaunchBert(ConLauncher::DOUBLE_LAUNCH_PWR_SCALE_FACTOR);
  LaunchErnie(ConLauncher::DOUBLE_LAUNCH_PWR_SCALE_FACTOR);
}

void Launcher::LaunchBert(double pwrScale) {
  printf("LaunchBert() Power %f Limit %f\n", 
    m_useClose ? m_BertFwdPower : m_BertFarPower,
    m_useClose ? m_BertFwdLimit : m_BertFarLimit);
  // Launch a ball
#ifdef ENABLE_LAUNCHER
  if (m_useClose) {
    m_launcherMotorBert.Set(m_BertFwdPower * pwrScale);
  } else {
    m_launcherMotorBert.Set(m_BertFarPower * pwrScale);
  }
  printf("Battery Voltage: %f\n", frc::DriverStation::GetBatteryVoltage());
#endif
}

void Launcher::LaunchErnie(double pwrScale) {
  printf("LaunchErnie() Power %f Limit %f\n",
    m_useClose ? m_ErnieFwdPower : m_ErnieFarPower,
    m_useClose ? m_ErnieFwdLimit : m_ErnieFarLimit); 
  // Launch a ball
#ifdef ENABLE_LAUNCHER
  if (m_useClose) {
    m_launcherMotorErnie.Set(m_ErnieFwdPower * pwrScale);
  } else {
    m_launcherMotorErnie.Set(m_ErnieFarPower * pwrScale);
  }
  printf("Battery Voltage: %f\n", frc::DriverStation::GetBatteryVoltage());
#endif
}

void Launcher::Retract() {
  // Bring both launchers back
  RetractBert();
  RetractErnie();
}

void Launcher::RetractBert() {
  printf("Launcher::RetractBert() Executing...\n");  
  #ifdef ENABLE_LAUNCHER
  m_launcherMotorBert.Set(-.1);
  #endif
}

void Launcher::RetractErnie() {
  printf("Launcher::RetractErnie() Executing...\n");  
  #ifdef ENABLE_LAUNCHER
  m_launcherMotorErnie.Set(-.1);
  #endif
}

void Launcher::SetupFar(){
  printf("Launcher::SetupFar()\n");
  m_useClose = false;
#ifdef ENABLE_LAUNCHER
  m_launcherMotorErnie.SetSoftLimit(rev::CANSparkMax::SoftLimitDirection::kForward, m_ErnieFarLimit);
  m_launcherMotorBert.SetSoftLimit(rev::CANSparkMax::SoftLimitDirection::kForward, m_BertFarLimit);
#endif // ENABLE_LAUNCHER
}

void Launcher::SetupClose() {
  printf("Launcher::SetupClose()\n");
  m_useClose = true;
#ifdef ENABLE_LAUNCHER
  m_launcherMotorErnie.SetSoftLimit(rev::CANSparkMax::SoftLimitDirection::kForward, m_ErnieFwdLimit);
  m_launcherMotorBert.SetSoftLimit(rev::CANSparkMax::SoftLimitDirection::kForward, m_BertFwdLimit);
#endif // ENABLE_LAUNCHER
}

void Launcher::SetLaunchSoftLimits() {
  double d;

  // Close Power
  d = m_nte_Ernie_Power.GetDouble(ConLauncher::ERNIE_POWER);
  if (d != m_ErnieFwdPower) {
    m_ErnieFwdPower = d;
    printf("m_ErnieFwdPower set to %f\n", m_ErnieFwdPower);
  }
  d = m_nte_Bert_Power.GetDouble(ConLauncher::BERT_POWER);
  if (d != m_BertFwdPower) {
    m_BertFwdPower = d;
    printf("m_BertFwdPower set to %f\n", m_BertFwdPower);
  }

  // Close Limits
  d = m_nte_Ernie_FwdLimit.GetDouble(ConLauncher::ERNIE_FWD_LIMIT);
  if (d != m_ErnieFwdLimit) {
    m_ErnieFwdLimit = d;
    printf("m_ErnieFwdLimit set to %f\n", m_ErnieFwdLimit);
  }
  d = m_nte_Bert_FwdLimit.GetDouble(ConLauncher::BERT_FWD_LIMIT);
  if (d != m_BertFwdLimit) {
    m_BertFwdLimit = d;
    printf("m_BertFwdLimit set to %f\n", m_BertFwdLimit);
  }

  // Far Power
  d = m_nte_Ernie_FarPower.GetDouble(ConLauncher::ERNIE_FAR_POWER);
  if (d != m_ErnieFarPower) {
    m_ErnieFarPower = d;
    printf("m_ErnieFarPower set to %f\n", m_ErnieFarPower);
  }
  d = m_nte_Bert_FarPower.GetDouble(ConLauncher::BERT_FAR_POWER);
  if (d != m_BertFarPower) {
    m_BertFarPower = d;
    printf("m_BertFarPower set to %f\n", m_BertFarPower);
  }

  // Far Limits
  d = m_nte_Ernie_FarLimit.GetDouble(ConLauncher::ERNIE_FAR_LIMIT);
  if (d != m_ErnieFarLimit) {
    m_ErnieFarLimit = d;
    printf("m_ErnieFarLimit set to %f\n", m_ErnieFarLimit);
  }
  d = m_nte_Bert_FarLimit.GetDouble(ConLauncher::BERT_FAR_LIMIT);
  if (d != m_BertFarLimit) {
    m_BertFarLimit = d;
    printf("m_BertFarLimit set to %f\n", m_BertFarLimit);
  }
}

void Launcher::SetResetSoftLimits() {
}

void Launcher::Stop() {
  #ifdef ENABLE_LAUNCHER
  m_launcherMotorErnie.Set(0.0);
  m_launcherMotorBert.Set(0.0);
  #endif
}

void Launcher::Periodic() {
#ifdef ENABLE_LAUNCHER
  // Display
  m_nte_Bert_Position.SetDouble(m_launcherEncoderBert.GetPosition());
  m_nte_Ernie_Position.SetDouble(m_launcherEncoderErnie.GetPosition());
  m_nte_Bert_Voltage.SetDouble(m_launcherMotorErnie.GetBusVoltage());
  m_nte_Ernie_Voltage.SetDouble(m_launcherMotorErnie.GetBusVoltage());
#endif
}

void Launcher::BurnFlash() {
  printf("BurnFlash for Launchers\n");
#ifdef ENABLE_LAUNCHER
  // Save the configuration to flash memory
  m_launcherMotorErnie.BurnFlash();
  m_launcherMotorBert.BurnFlash();
#endif // ENABLE_LAUNCHER
}

 */