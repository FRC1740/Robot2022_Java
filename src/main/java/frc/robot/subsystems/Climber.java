// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.PIDSubsystem;
import frc.robot.Constants.ConSparkMax;
import frc.robot.OI.ConShuffleboard;

public class Climber extends PIDSubsystem {
  
  public static class ConClimber {
    // Motor
    public static int CLIMBER_MOTOR_ID = 8;
  
    //Spark Max Settings
    public static double RAMP_RATE = 0.100; //seconds
    public static boolean INVERTED = true; //
    public static boolean NONINVERTED = false; //
    public static double CLIMB_SPEED = -1.0;  // Climb Motor Speed
    public static double DESCEND_SPEED = 1.0; // Descend Motor Speed
    public static int SOFT_LIMIT_FWD = 4784; // Soft Limit Extension 5' 6" MAX height; Bar @ 60-1/4"
    public static int SOFT_LIMIT_REV = 200;
    public static int CURRENT_STALL_LIMIT = 80;
  
    //Servo
    public static int kServoPWMPort = 9;
  }
  
  ShuffleboardTab m_sbt_Climber;
  NetworkTableEntry m_nte_ClimberDistance;
  NetworkTableEntry m_nte_ClimberOutput;
  NetworkTableEntry m_nte_ClimbSpeedLimit;
  NetworkTableEntry m_nte_DescendSpeedLimit;
  NetworkTableEntry m_nte_ExtendLimit;
  NetworkTableEntry m_nte_RetractLimit;
  NetworkTableEntry m_nte_MotorCurrent;
  double m_softLimitFwd = ConClimber.SOFT_LIMIT_FWD;
  double m_softLimitRev = ConClimber.SOFT_LIMIT_REV;
  
  // #ifdef ENABLE_CLIMBER
    // Neo motor controllers
    CANSparkMax m_climberMotor = new CANSparkMax(ConClimber.CLIMBER_MOTOR_ID, CANSparkMax.MotorType.kBrushless);
    // Drive encoders
    RelativeEncoder m_climberEncoder = m_climberMotor.getEncoder();
  // #endif // ENABLE_CLIMBER
  
  public Climber()  {
    // The PIDController used by the subsystem
    super(new PIDController(0, 0, 0));

    // Initialize Shuffleboard Tab and Network Table Entries
    m_sbt_Climber = Shuffleboard.getTab(ConShuffleboard.ClimberTab);

    m_nte_ClimberDistance = m_sbt_Climber.addPersistent("Climber Position", 0.0)
          .withSize(2,1)
          .withPosition(0,0)
          .getEntry();
    m_nte_ClimberOutput = m_sbt_Climber.addPersistent("Climber Output", 0.0)
          .withSize(2,2)
          .withPosition(0,1)
          .withWidget(BuiltInWidgets.kDial)
          .getEntry();
    m_nte_MotorCurrent = m_sbt_Climber.addPersistent("Motor Current", 0.0)
          .withSize(2,2)
          .withPosition(0,2)
          .withWidget(BuiltInWidgets.kDial)
          .getEntry();
    m_nte_ClimbSpeedLimit = m_sbt_Climber.addPersistent("Climb Speed Limit", ConClimber.CLIMB_SPEED)
          .withSize(2,1)
          .withPosition(2,0)
          .getEntry();
    m_nte_DescendSpeedLimit = m_sbt_Climber.addPersistent("Descend Speed Limit", ConClimber.DESCEND_SPEED)
          .withSize(2,1)
          .withPosition(2,1)
          .getEntry();
    m_nte_ExtendLimit = m_sbt_Climber.addPersistent("Extension Limit", ConClimber.SOFT_LIMIT_FWD)
          .withSize(2,1)
          .withPosition(2,2)
          .getEntry();
    m_nte_RetractLimit = m_sbt_Climber.addPersistent("Retract Limit", ConClimber.SOFT_LIMIT_REV)
          .withSize(2,1)
          .withPosition(2,3)
          .getEntry();

  // #ifdef ENABLE_CLIMBER
    m_climberMotor.setSmartCurrentLimit(ConClimber.CURRENT_STALL_LIMIT, ConClimber.CURRENT_STALL_LIMIT);
    m_climberEncoder.setPositionConversionFactor(ConSparkMax.POSITION_CONVERSION_FACTOR); // Generally 42
    m_climberMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);
    // Reset Encoder to zero for starting configuration
    m_climberEncoder.setPosition(0.0);
    // Configure SparkMax SoftLimits
    m_climberMotor.enableSoftLimit(CANSparkMax.SoftLimitDirection.kForward, true);
    m_climberMotor.setSoftLimit(CANSparkMax.SoftLimitDirection.kForward, ConClimber.SOFT_LIMIT_FWD);
    m_climberMotor.enableSoftLimit(CANSparkMax.SoftLimitDirection.kReverse, true);
    m_climberMotor.setSoftLimit(CANSparkMax.SoftLimitDirection.kReverse, ConClimber.SOFT_LIMIT_REV);
  // #endif 
  }

public void useOutput(double output, double setpoint) {
  // Use the output here
}

public double getMeasurement() {
  // Return the process variable measurement here
  return 0;
}

// Climb lifts the robot up to target position
public void climb() {
  // #ifdef ENABLE_CLIMBER
    m_climberMotor.set(m_nte_ClimbSpeedLimit.getDouble(ConClimber.CLIMB_SPEED));
  // #endif
}

// Descend is a manual override to lower the robot
public void extend() {
  // #ifdef ENABLE_CLIMBER
    m_climberMotor.set(m_nte_DescendSpeedLimit.getDouble(ConClimber.DESCEND_SPEED));
  // #endif
}

public void stop() {
  // #ifdef ENABLE_CLIMBER
    m_climberMotor.set(0.0);
  // #endif
}

public void periodic() {
  // #ifdef ENABLE_CLIMBER
    m_nte_ClimberDistance.setDouble(m_climberEncoder.getPosition());
    m_nte_ClimberOutput.setDouble(m_climberEncoder.getVelocity());
    m_nte_MotorCurrent.setDouble(m_climberMotor.getOutputCurrent());
  // #endif
}

public void setClimberSoftLimits() {
  double d;
  d = m_nte_ExtendLimit.getDouble(ConClimber.SOFT_LIMIT_FWD);
  if (d != m_softLimitFwd) {
    System.out.println("Changing Forward limit from " + m_softLimitFwd + " to " + d);
    m_softLimitFwd = d;
  // #ifdef ENABLE_CLIMBER
    m_climberMotor.enableSoftLimit(CANSparkMax.SoftLimitDirection.kForward, true);
    m_climberMotor.setSoftLimit(CANSparkMax.SoftLimitDirection.kForward, (float) m_softLimitFwd);
  // #endif // ENABLE_CLIMBER
  }
  d = m_nte_RetractLimit.getDouble(ConClimber.SOFT_LIMIT_REV);
  if (d != m_softLimitRev) {
    System.out.println("Changing Reverse limit from " + m_softLimitRev + " to " + d);
    m_softLimitRev = d;
  // #ifdef ENABLE_CLIMBER
    m_climberMotor.enableSoftLimit(CANSparkMax.SoftLimitDirection.kReverse, true);
    m_climberMotor.setSoftLimit(CANSparkMax.SoftLimitDirection.kReverse, (float) m_softLimitRev);
  // #endif // ENABLE_CLIMBER
  }
}

public void burnFlash() {
  System.out.println("BurnFlash for Climber");
  // #ifdef ENABLE_CLIMBER
    m_climberMotor.burnFlash();
  // #endif // ENABLE_CLIMBER
}

}

/** Original H
 // Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/PIDSubsystem.h>
#include <frc/shuffleboard/Shuffleboard.h>
#include <frc/shuffleboard/ShuffleboardTab.h>
#include <networktables/NetworkTableEntry.h>
#include <rev/CANSparkMax.h>
#include "Constants.h"

namespace ConClimber {
  // Motor
  constexpr int CLIMBER_MOTOR_ID = 8;

  //Spark Max Settings
  constexpr int RAMP_RATE = 0.100; //seconds
  constexpr bool INVERTED = true; //
  constexpr bool NONINVERTED = false; //
  constexpr double CLIMB_SPEED = -1.0;  // Climb Motor Speed
  constexpr double DESCEND_SPEED = 1.0; // Descend Motor Speed
  constexpr int SOFT_LIMIT_FWD = 4784; // Soft Limit Extension 5' 6" MAX height; Bar @ 60-1/4"
  constexpr int SOFT_LIMIT_REV = 200;
  constexpr int CURRENT_STALL_LIMIT = 80;

  //Servo
  constexpr int kServoPWMPort = 9;
}


class Climber : public frc2::PIDSubsystem {
 public:
  Climber();

  void Climb();
  void Extend();
  void Stop();
  void SetClimberSoftLimits();
  void BurnFlash();

  frc::ShuffleboardTab *m_sbt_Climber;
  nt::NetworkTableEntry m_nte_ClimberDistance;
  nt::NetworkTableEntry m_nte_ClimberOutput;
  nt::NetworkTableEntry m_nte_ClimbSpeedLimit;
  nt::NetworkTableEntry m_nte_DescendSpeedLimit;
  nt::NetworkTableEntry m_nte_ExtendLimit;
  nt::NetworkTableEntry m_nte_RetractLimit;
  nt::NetworkTableEntry m_nte_MotorCurrent;
  double m_softLimitFwd = ConClimber::SOFT_LIMIT_FWD;
  double m_softLimitRev = ConClimber::SOFT_LIMIT_REV;

 protected:

  void UseOutput(double output, double setpoint) override;
  double GetMeasurement() override;
  void Periodic();

#ifdef ENABLE_CLIMBER
  // Neo motor controllers
  rev::CANSparkMax m_climberMotor {ConClimber::CLIMBER_MOTOR_ID, rev::CANSparkMax::MotorType::kBrushless};
  // Drive encoders
  rev::SparkMaxRelativeEncoder m_climberEncoder = m_climberMotor.GetEncoder();
#endif // ENABLE_CLIMBER
};

 */

/** Original CPP
 // Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/Climber.h"
#include "OI.h"

Climber::Climber()
    : PIDSubsystem(
          // The PIDController used by the subsystem
          frc2::PIDController(0, 0, 0)) {

    // Initialize Shuffleboard Tab and Network Table Entries
    m_sbt_Climber = &frc::Shuffleboard::GetTab(ConShuffleboard::ClimberTab);

    m_nte_ClimberDistance = m_sbt_Climber->AddPersistent("Climber Position", 0.0)
          .WithSize(2,1)
          .WithPosition(0,0)
          .GetEntry();
    m_nte_ClimberOutput = m_sbt_Climber->AddPersistent("Climber Output", 0.0)
          .WithSize(2,2)
          .WithPosition(0,1)
          .WithWidget(frc::BuiltInWidgets::kDial)
          .GetEntry();
    m_nte_MotorCurrent = m_sbt_Climber->AddPersistent("Motor Current", 0.0)
          .WithSize(2,2)
          .WithPosition(0,2)
          .WithWidget(frc::BuiltInWidgets::kDial)
          .GetEntry();
    m_nte_ClimbSpeedLimit = m_sbt_Climber->AddPersistent("Climb Speed Limit", ConClimber::CLIMB_SPEED)
          .WithSize(2,1)
          .WithPosition(2,0)
          .GetEntry();
    m_nte_DescendSpeedLimit = m_sbt_Climber->AddPersistent("Descend Speed Limit", ConClimber::DESCEND_SPEED)
          .WithSize(2,1)
          .WithPosition(2,1)
          .GetEntry();
    m_nte_ExtendLimit = m_sbt_Climber->AddPersistent("Extension Limit", ConClimber::SOFT_LIMIT_FWD)
          .WithSize(2,1)
          .WithPosition(2,2)
          .GetEntry();
    m_nte_RetractLimit = m_sbt_Climber->AddPersistent("Retract Limit", ConClimber::SOFT_LIMIT_REV)
          .WithSize(2,1)
          .WithPosition(2,3)
          .GetEntry();

#ifdef ENABLE_CLIMBER
    m_climberMotor.SetSmartCurrentLimit(ConClimber::CURRENT_STALL_LIMIT, ConClimber::CURRENT_STALL_LIMIT);
    m_climberEncoder.SetPositionConversionFactor(ConSparkMax::POSITION_CONVERSION_FACTOR); // Generally 42
    m_climberMotor.SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);
    // Reset Encoder to zero for starting configuration
    m_climberEncoder.SetPosition(0.0);
    // Configure SparkMax SoftLimits
    m_climberMotor.EnableSoftLimit(rev::CANSparkMax::SoftLimitDirection::kForward, true);
    m_climberMotor.SetSoftLimit(rev::CANSparkMax::SoftLimitDirection::kForward, ConClimber::SOFT_LIMIT_FWD);
    m_climberMotor.EnableSoftLimit(rev::CANSparkMax::SoftLimitDirection::kReverse, true);
    m_climberMotor.SetSoftLimit(rev::CANSparkMax::SoftLimitDirection::kReverse, ConClimber::SOFT_LIMIT_REV);
#endif 
  }

void Climber::UseOutput(double output, double setpoint) {
  // Use the output here
}

double Climber::GetMeasurement() {
  // Return the process variable measurement here
  return 0;
}

// Climb lifts the robot up to target position
void Climber::Climb() {
#ifdef ENABLE_CLIMBER
    m_climberMotor.Set(m_nte_ClimbSpeedLimit.GetDouble(ConClimber::CLIMB_SPEED));
#endif
}

// Descend is a manual override to lower the robot
void Climber::Extend() {
#ifdef ENABLE_CLIMBER
  m_climberMotor.Set(m_nte_DescendSpeedLimit.GetDouble(ConClimber::DESCEND_SPEED));
#endif
}

void Climber::Stop() {
#ifdef ENABLE_CLIMBER
  m_climberMotor.Set(0.0);
#endif
}

void Climber::Periodic() {
#ifdef ENABLE_CLIMBER
  m_nte_ClimberDistance.SetDouble(m_climberEncoder.GetPosition());
  m_nte_ClimberOutput.SetDouble(m_climberEncoder.GetVelocity());
  m_nte_MotorCurrent.SetDouble(m_climberMotor.GetOutputCurrent());
#endif
}

void Climber::SetClimberSoftLimits() {
  double d;
  d = m_nte_ExtendLimit.GetDouble(ConClimber::SOFT_LIMIT_FWD);
  if (d != m_softLimitFwd) {
    printf("Changing Forward limit from %f to %f\n", m_softLimitFwd, d);
    m_softLimitFwd = d;
#ifdef ENABLE_CLIMBER
    m_climberMotor.EnableSoftLimit(rev::CANSparkMax::SoftLimitDirection::kForward, true);
    m_climberMotor.SetSoftLimit(rev::CANSparkMax::SoftLimitDirection::kForward, m_softLimitFwd);
#endif // ENABLE_CLIMBER
  }
  d = m_nte_RetractLimit.GetDouble(ConClimber::SOFT_LIMIT_REV);
  if (d != m_softLimitRev) {
    printf("Changing Reverse limit from %f to %f\n", m_softLimitRev, d);
    m_softLimitRev = d;
#ifdef ENABLE_CLIMBER
    m_climberMotor.EnableSoftLimit(rev::CANSparkMax::SoftLimitDirection::kReverse, true);
    m_climberMotor.SetSoftLimit(rev::CANSparkMax::SoftLimitDirection::kReverse, m_softLimitRev);
#endif // ENABLE_CLIMBER
  }
}

void Climber::BurnFlash() {
  printf("BurnFlash for Climber\n");
#ifdef ENABLE_CLIMBER
  m_climberMotor.BurnFlash();
#endif // ENABLE_CLIMBER
}

 */