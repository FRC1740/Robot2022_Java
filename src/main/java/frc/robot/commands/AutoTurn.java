// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.OI.ConShuffleboard;
import frc.robot.subsystems.DriveTrain;

public class AutoTurn extends CommandBase {

  private DriveTrain m_driveTrain;
  private ShuffleboardTab m_sbt_Robot;
  private NetworkTableEntry m_nte_AutoTurn_kP;
  private NetworkTableEntry m_nte_AutoTurn_kI;
  private NetworkTableEntry m_nte_AutoTurn_kD;
  private NetworkTableEntry m_nte_AutoTurn_error;
  private NetworkTableEntry m_nte_AutoTurn_output;
  private double m_angle_degrees;
  private double m_rotationOut = 0.0;
  private double m_kP, m_kI, m_kD, m_setpoint, m_integral, m_output, m_previous_error;

  /** Creates a new AutoTurn. */
  public AutoTurn(DriveTrain driveTrain) {
    m_driveTrain = driveTrain;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(driveTrain);

    m_integral = 0;
    // Create and get reference to SB tab
    m_sbt_Robot = Shuffleboard.getTab(ConShuffleboard.RobotTab);
    // PID constants: Starting with .5/.001/.3 worked OK on terazzo floor
    m_nte_AutoTurn_kP = m_sbt_Robot.addPersistent("AutoTurn kP", 0.5)
      .withSize(1,1)
      .withPosition(0,1)
      .getEntry();
    m_nte_AutoTurn_kI = m_sbt_Robot.addPersistent("AutoTurn kI", 0.01)
      .withSize(1,1)
      .withPosition(1,1)
      .getEntry();
    m_nte_AutoTurn_kD = m_sbt_Robot.addPersistent("AutoTurn kD", 0.3)
      .withSize(1,1)
      .withPosition(2,1)
      .getEntry();
    m_nte_AutoTurn_error = m_sbt_Robot.addPersistent("PID Error", 0.0)
      .withSize(1,1)
      .withPosition(0,2)
      .getEntry();
    m_nte_AutoTurn_output = m_sbt_Robot.addPersistent("PID Output", 0.0)
      .withSize(1,1)
      .withPosition(1,2)
      .getEntry();
}

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_driveTrain.resetGyro();

    // Read the target angle from the dashboard
    //m_setpoint = m_driveTrain->m_nte_c_DriveTurnAngle.GetDouble(0.0);
    m_setpoint = m_angle_degrees;
    // Read PID control parameters from the dashboard
    m_kP = m_nte_AutoTurn_kP.getDouble(1.0);
    m_kI = m_nte_AutoTurn_kI.getDouble(0.0);
    m_kD = m_nte_AutoTurn_kD.getDouble(0.0);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double speed = 0.0;
    
    pid();
    // Negating the output because wheels are rotating in the wrong direction
    m_driveTrain.arcadeDrive(speed, -m_output);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_driveTrain.arcadeDrive(0.0, 0.0);  
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }

  public void pid() {
    double error, derivative;
    error = (m_setpoint - m_driveTrain.getGyroAngle())/Math.abs(m_setpoint); // Error = Target - Actual, eg 45 degrees, normalized to 0.0 - 1.0
    m_integral += (error*.02); // Integral is increased by the error*time (which is .02 seconds using normal IterativeRobot)
    derivative = (error - m_previous_error) / .02;
    m_output = m_kP * error + m_kI*m_integral + m_kD*derivative;
    m_nte_AutoTurn_error.setDouble(error);
    m_nte_AutoTurn_output.setDouble(-m_output);
    m_previous_error = error;
  }

}

/** Original H
 // Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/CommandBase.h>
#include <frc2/command/CommandHelper.h>
#include "subsystems/DriveTrain.h"

 * An example command.
 *
 * <p>Note that this extends CommandHelper, rather extending CommandBase
 * directly; this is crucially important, or else the decorator functions in
 * Command will *not* work!
 
class AutoTurn
: public frc2::CommandHelper<frc2::CommandBase, AutoTurn> {
public:
AutoTurn(DriveTrain *drivetrain, double angle);

#ifdef ENABLE_DRIVETRAIN
void Initialize() override;

void Execute() override;

void End(bool interrupted) override;

bool IsFinished() override;

void PID();
#endif // ENABLE_DRIVETRAIN

private:

DriveTrain *m_driveTrain;
frc::ShuffleboardTab* m_sbt_Robot;
nt::NetworkTableEntry m_nte_AutoTurn_kP;
nt::NetworkTableEntry m_nte_AutoTurn_kI;
nt::NetworkTableEntry m_nte_AutoTurn_kD;
nt::NetworkTableEntry m_nte_AutoTurn_error;
nt::NetworkTableEntry m_nte_AutoTurn_output;
double m_angle_degrees;
double m_rotationOut = 0.0;
double m_kP, m_kI, m_kD, m_setpoint, m_integral, m_output, m_previous_error;
};

 */

/** Original CPP
 // Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include <cmath>
#include <frc/shuffleboard/Shuffleboard.h>
#include <frc/shuffleboard/ShuffleboardTab.h>
#include <networktables/NetworkTableEntry.h>
#include "commands/AutoTurn.h"

AutoTurn::AutoTurn(DriveTrain *drivetrain, double angle) : m_driveTrain{drivetrain}, m_angle_degrees{angle} {
  // Use addRequirements() here to declare subsystem dependencies.
  AddRequirements(drivetrain);
  m_integral = 0;
  // Create and get reference to SB tab
  m_sbt_Robot = &frc::Shuffleboard::GetTab(ConShuffleboard::RobotTab);
  // PID constants: Starting with .5/.001/.3 worked OK on terazzo floor
  m_nte_AutoTurn_kP = m_sbt_Robot->AddPersistent("AutoTurn kP", 0.5).WithSize(1,1).WithPosition(0,1).GetEntry();
  m_nte_AutoTurn_kI = m_sbt_Robot->AddPersistent("AutoTurn kI", 0.01).WithSize(1,1).WithPosition(1,1).GetEntry();
  m_nte_AutoTurn_kD = m_sbt_Robot->AddPersistent("AutoTurn kD", 0.3).WithSize(1,1).WithPosition(2,1).GetEntry();
  m_nte_AutoTurn_error = m_sbt_Robot->AddPersistent("PID Error", 0.0).WithSize(1,1).WithPosition(0,2).GetEntry();
  m_nte_AutoTurn_output = m_sbt_Robot->AddPersistent("PID Output", 0.0).WithSize(1,1).WithPosition(1,2).GetEntry();

}

#ifdef ENABLE_DRIVETRAIN
// Called when the command is initially scheduled.
void AutoTurn::Initialize() {
  m_driveTrain->ResetGyro();

  // Read the target angle from the dashboard
  //m_setpoint = m_driveTrain->m_nte_c_DriveTurnAngle.GetDouble(0.0);
  m_setpoint = m_angle_degrees;
  // Read PID control parameters from the dashboard
  m_kP = m_nte_AutoTurn_kP.GetDouble(1.0);
  m_kI = m_nte_AutoTurn_kI.GetDouble(0.0);
  m_kD = m_nte_AutoTurn_kD.GetDouble(0.0);
}

void AutoTurn::PID() {
  double error, derivative;
  error = (m_setpoint - m_driveTrain->GetGyroAngle())/abs(m_setpoint); // Error = Target - Actual, eg 45 degrees, normalized to 0.0 - 1.0
  m_integral += (error*.02); // Integral is increased by the error*time (which is .02 seconds using normal IterativeRobot)
  derivative = (error - m_previous_error) / .02;
  m_output = m_kP * error + m_kI*m_integral + m_kD*derivative;
  m_nte_AutoTurn_error.SetDouble(error);
  m_nte_AutoTurn_output.SetDouble(-m_output);
  m_previous_error = error;
}

// Called repeatedly when this Command is scheduled to run
void AutoTurn::Execute() {

  constexpr double speed = 0.0;

  PID();
  // Negating the output because wheels are rotating in the wrong direction
  m_driveTrain->ArcadeDrive(speed, -m_output);
}

// Called once the command ends or is interrupted.
void AutoTurn::End(bool interrupted) {
  m_driveTrain->ArcadeDrive(0.0, 0.0);  
}

// Returns true when the command should end.
bool AutoTurn::IsFinished() {
  return false;
}
#endif // ENABLE_DRIVETRAIN

 */