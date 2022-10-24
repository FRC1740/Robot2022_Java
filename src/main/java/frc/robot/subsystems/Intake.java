// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.OI;

public class Intake extends SubsystemBase {

  public static class ConIntake {
    public static int MOTOR_ID = 9;
    public static int PNEUM_PORT_A = 0;
    public static int PNEUM_PORT_B = 1;
    public static double LOAD_BALL = -.9; // Should be motor forward
    public static double REJECT_BALL = .9; // Should be motor reverse
    public static int CURRENT_STALL_LIMIT = 40;
    public static double INTAKE_POWER = .55; // Up from 60% power
    public static DoubleSolenoid.Value DEPLOY_INTAKE = DoubleSolenoid.Value.kReverse;
    public static DoubleSolenoid.Value STOW_INTAKE = DoubleSolenoid.Value.kForward;
    public static double SHUTDOWN_DELAY = 1.0; // Seconds to delay between Stow() and motor shutdown  /** Creates a new Intake. */
    }
  
  ShuffleboardTab m_sbt_Intake;
  NetworkTableEntry m_nte_MotorCurrent;
  NetworkTableEntry m_nte_StowedState;
  NetworkTableEntry m_nte_MotorPower;
  NetworkTableEntry m_nte_ShutdownDelay;

  //#ifdef ENABLE_INTAKE
  DoubleSolenoid deployDoublePCM = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, ConIntake.PNEUM_PORT_A, ConIntake.PNEUM_PORT_B);
  CANSparkMax m_intakeMotor = new CANSparkMax(ConIntake.MOTOR_ID, CANSparkMax.MotorType.kBrushless); // New Neo motor
  RelativeEncoder m_intakeEncoder = m_intakeMotor.getEncoder();
  //#endif // ENABLE_INTAKE

  boolean m_deployedState; // True if intake system is deployed outside of robot perimeter
  double m_intakePower; // Power to run intake motor
  Timer m_timer;
  
  public Intake() {
    // #ifdef ENABLE_INTAKE
      // Initialize the DoubleSolenoid so it knows where to start.  Not required for single solenoids.
      deployDoublePCM.set(DoubleSolenoid.Value.kReverse);
      m_deployedState = false;

      m_intakeMotor.setSmartCurrentLimit(ConIntake.CURRENT_STALL_LIMIT, ConIntake.CURRENT_STALL_LIMIT);
      m_intakeMotor.setIdleMode(CANSparkMax.IdleMode.kCoast);
      m_intakeMotor.setOpenLoopRampRate(0.1);

      // Initialize Shuffleboard Tab and Network Table Entries
      m_sbt_Intake = Shuffleboard.getTab(OI.ConShuffleboard.IntakeTab);

      m_nte_MotorPower = m_sbt_Intake.addPersistent("Motor Power", ConIntake.INTAKE_POWER)
            .withSize(2,1)
            .withPosition(0,0)
            .getEntry();
      m_nte_ShutdownDelay = m_sbt_Intake.addPersistent("Shutdown Delay", ConIntake.SHUTDOWN_DELAY)
            .withSize(2,1)
            .withPosition(0,1)
            .getEntry();
      m_nte_MotorCurrent = m_sbt_Intake.addPersistent("Intake Current", 0.0)
            .withSize(2,1)
            .withPosition(0,2)
            .withWidget(BuiltInWidgets.kDial)
            // .WithProperties({"min" : 0, "max" : ConIntake::CURRENT_STALL_LIMIT});
            // Would like to use .WithProperties() to set Max to CURRENT_LIMIT
            .getEntry();

      m_nte_StowedState = m_sbt_Intake.addPersistent("Deployed State", true)
            .withSize(2,2)
            .withPosition(2,0)
            .getEntry();

      m_timer = new Timer(); // For delayed shutdown of intake motor
    //#endif // ENABLE_INTAKE
  }

  public void ToggleDeployedState() {
    if(m_deployedState) {
      stow();
    } else {
      deploy();
    }
  }

  public void deploy() {
    System.out.println("Intake::Deploy() Executing...");
    //#ifdef ENABLE_INTAKE
      deployDoublePCM.set(ConIntake.DEPLOY_INTAKE);
    //#endif // ENABLE_INTAKE
      m_deployedState = true;
      load();
      System.out.println("Battery Voltage: " + RobotController.getBatteryVoltage());
    
  }

  public void stow() {
    System.out.println("Intake::Stow() Executing...");
  
    //#ifdef ENABLE_INTAKE
      deployDoublePCM.set(ConIntake.STOW_INTAKE);
      m_timer.reset();
      m_timer.start();
    //#endif // ENABLE_INTAKE
      m_deployedState = false;
      System.out.println("Battery Voltage: " + RobotController.getBatteryVoltage());
    
  }

  public void load() {
    System.out.println("Intake::Load() Executing...");
    //#ifdef ENABLE_INTAKE
      if (m_deployedState) {
        m_intakeMotor.set(ConIntake.LOAD_BALL * m_intakePower);
      } else {
        m_intakeMotor.set(0.0);
      }
    //#endif // ENABLE_INTAKE
  }

  public void reject() {
    System.out.println("Intake::Reject() Executing...");
    // #ifdef ENABLE_INTAKE
      if (m_deployedState) {
        m_intakeMotor.set(ConIntake.REJECT_BALL * m_intakePower);
      } else {
        m_intakeMotor.set(0.0);
      }  
    // #endif // ENABLE_INTAKE
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    // #ifdef ENABLE_INTAKE
    m_nte_MotorCurrent.setDouble(m_intakeMotor.getOutputCurrent());
    m_nte_StowedState.setBoolean(!m_deployedState);
    m_intakePower = m_nte_MotorPower.getDouble(ConIntake.INTAKE_POWER);
    if (m_deployedState == false && m_timer.get() > ConIntake.SHUTDOWN_DELAY) {
      m_intakeMotor.set(0.0);
    }
    // #endif // ENABLE_INTAKE 
  }

  public void burnFlash() {
    System.out.println("BurnFlash for Intake");
    //#ifdef ENABLE_INTAKE
      // Save SparkMax motor/encoder config to flash memory
      m_intakeMotor.burnFlash();
   //#endif // ENABLE_INTAKE
  }

}

/** Original H
 // Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/SubsystemBase.h>
#include <frc/DoubleSolenoid.h>
#include <frc/shuffleboard/Shuffleboard.h>
#include <frc/shuffleboard/ShuffleboardTab.h>
#include <networktables/NetworkTableEntry.h>
#include <frc/Timer.h>
#include <rev/CANSparkMax.h>
#include "Constants.h"

namespace ConIntake {
  constexpr int MOTOR_ID = 9;
  constexpr int PNEUM_PORT_A = 0;
  constexpr int PNEUM_PORT_B = 1;
  constexpr double LOAD_BALL = -.9; // Should be motor forward
  constexpr double REJECT_BALL = .9; // Should be motor reverse
  constexpr int CURRENT_STALL_LIMIT = 40;
  constexpr double INTAKE_POWER = .55; // Up from 60% power
  constexpr frc::DoubleSolenoid::Value DEPLOY_INTAKE = frc::DoubleSolenoid::Value::kReverse;
  constexpr frc::DoubleSolenoid::Value STOW_INTAKE = frc::DoubleSolenoid::Value::kForward;
  constexpr units::time::second_t SHUTDOWN_DELAY = 1.0_s; // Seconds to delay between Stow() and motor shutdown
}

class Intake : public frc2::SubsystemBase {
 public:
  Intake();
  void ToggleDeployedState();
  void Deploy();
  void Stow();
  void Load();
  void Reject();
  void Periodic() override;
  void BurnFlash();

  frc::ShuffleboardTab *m_sbt_Intake;
  nt::NetworkTableEntry m_nte_MotorCurrent;
  nt::NetworkTableEntry m_nte_StowedState;
  nt::NetworkTableEntry m_nte_MotorPower;
  nt::NetworkTableEntry m_nte_ShutdownDelay;

 protected:
#ifdef ENABLE_INTAKE
  frc::DoubleSolenoid deployDoublePCM{frc::PneumaticsModuleType::CTREPCM, ConIntake::PNEUM_PORT_A, ConIntake::PNEUM_PORT_B};
  rev::CANSparkMax m_intakeMotor {ConIntake::MOTOR_ID, rev::CANSparkMax::MotorType::kBrushless}; // New Neo motor
  rev::SparkMaxRelativeEncoder m_intakeEncoder = m_intakeMotor.GetEncoder();
#endif // ENABLE_INTAKE

  bool m_deployedState; // True if intake system is deployed outside of robot perimeter
  double m_intakePower; // Power to run intake motor
  frc::Timer m_timer;
};

 */

/** Original CPP
 // Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include <frc/DriverStation.h>
#include "subsystems/Intake.h"
#include "OI.h"

Intake::Intake() {
#ifdef ENABLE_INTAKE
        // Initialize the DoubleSolenoid so it knows where to start.  Not required for single solenoids.
        deployDoublePCM.Set(frc::DoubleSolenoid::Value::kReverse);
        m_deployedState = false;

        m_intakeMotor.SetSmartCurrentLimit(ConIntake::CURRENT_STALL_LIMIT, ConIntake::CURRENT_STALL_LIMIT);
        m_intakeMotor.SetIdleMode(rev::CANSparkMax::IdleMode::kCoast);
        m_intakeMotor.SetOpenLoopRampRate(0.1);

        // Initialize Shuffleboard Tab and Network Table Entries
        m_sbt_Intake = &frc::Shuffleboard::GetTab(ConShuffleboard::IntakeTab);

        m_nte_MotorPower = m_sbt_Intake->AddPersistent("Motor Power", ConIntake::INTAKE_POWER)
              .WithSize(2,1)
              .WithPosition(0,0)
              .GetEntry();
        m_nte_ShutdownDelay = m_sbt_Intake->AddPersistent("Shutdown Delay", (double)ConIntake::SHUTDOWN_DELAY)
              .WithSize(2,1)
              .WithPosition(0,1)
              .GetEntry();
        m_nte_MotorCurrent = m_sbt_Intake->AddPersistent("Intake Current", 0.0)
              .WithSize(2,1)
              .WithPosition(0,2)
              .WithWidget(frc::BuiltInWidgets::kDial)
              // .WithProperties({"min" : 0, "max" : ConIntake::CURRENT_STALL_LIMIT});
              // Would like to use .WithProperties() to set Max to CURRENT_LIMIT
              .GetEntry();

        m_nte_StowedState = m_sbt_Intake->AddPersistent("Deployed State", true)
              .WithSize(2,2)
              .WithPosition(2,0)
              .GetEntry();

        m_timer = frc::Timer(); // For delayed shutdown of intake motor
#endif // ENABLE_INTAKE
        }

void Intake::ToggleDeployedState() {
  m_deployedState == true ? Stow() : Deploy();
}

void Intake::Deploy() {
  printf("Intake::Deploy() Executing...\n");
#ifdef ENABLE_INTAKE
  deployDoublePCM.Set(ConIntake::DEPLOY_INTAKE);
#endif // ENABLE_INTAKE
  m_deployedState = true;
  Load();
  printf("Battery Voltage: %f\n", frc::DriverStation::GetBatteryVoltage());
}

void Intake::Stow() {
  printf("Intake::Stow() Executing...\n");
  
#ifdef ENABLE_INTAKE
  deployDoublePCM.Set(ConIntake::STOW_INTAKE);
  m_timer.Reset();
  m_timer.Start();
#endif // ENABLE_INTAKE
  m_deployedState = false;
  printf("Battery Voltage: %f\n", frc::DriverStation::GetBatteryVoltage());
}

void Intake::Load() {
  printf("Intake::Load() Executing...\n");
#ifdef ENABLE_INTAKE
  if (m_deployedState) {
    m_intakeMotor.Set(ConIntake::LOAD_BALL * m_intakePower);
  } else {
    m_intakeMotor.Set(0.0);
  }
#endif // ENABLE_INTAKE
}

void Intake::Reject() {
  printf("Intake::Reject() Executing...\n");
#ifdef ENABLE_INTAKE
  if (m_deployedState) {
    m_intakeMotor.Set(ConIntake::REJECT_BALL * m_intakePower);
  } else {
    m_intakeMotor.Set(0.0);
  }  
#endif // ENABLE_INTAKE
}

void Intake::Periodic() {
#ifdef ENABLE_INTAKE
  m_nte_MotorCurrent.SetDouble(m_intakeMotor.GetOutputCurrent());
  m_nte_StowedState.SetBoolean(!m_deployedState);
  m_intakePower = m_nte_MotorPower.GetDouble(ConIntake::INTAKE_POWER);
  if (m_deployedState == false && m_timer.Get() > ConIntake::SHUTDOWN_DELAY) {
    m_intakeMotor.Set(0.0);
  }
#endif // ENABLE_INTAKE
}

void Intake::BurnFlash() {
  printf("BurnFlash for Intake\n");
#ifdef ENABLE_INTAKE
  // Save SparkMax motor/encoder config to flash memory
  m_intakeMotor.BurnFlash();
#endif // ENABLE_INTAKE
}

 */