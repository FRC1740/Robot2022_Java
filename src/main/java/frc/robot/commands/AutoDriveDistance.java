// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveTrain;

public class AutoDriveDistance extends CommandBase {

  private DriveTrain m_driveTrain;
  private double m_distance_inches = 0.0; // CRE 2022-01-28 Read from shuffleboard
  private double m_speedOut = 0.0;

  /** Creates a new AutoDriveDistance. */
  public AutoDriveDistance(DriveTrain drivetrain, double distance) {
    m_driveTrain = drivetrain;
    m_distance_inches = distance;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_driveTrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_driveTrain.resetEncoders();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double speedN = 11.0; // length of digital filter
    double maxSpeed = 0.5;
    double rotation = 0.0;
  
    double desiredSpeed = (m_distance_inches > m_driveTrain.getAverageDistanceInches()) ? maxSpeed : -maxSpeed;
    double speed = (((speedN - 1.0) * m_speedOut) + desiredSpeed) / speedN;
    m_driveTrain.arcadeDrive(speed, rotation);
    m_speedOut = speed;
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_driveTrain.arcadeDrive(0.0, 0.0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    double epsilon = 5.0;

    return ((Math.abs(m_distance_inches + Math.copySign(epsilon / 2.0, m_distance_inches) - m_driveTrain.getAverageDistanceInches())) < epsilon);
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

// Moved to DriveTrain.h
// namespace ConAutoDriveDistance {
//   constexpr double DISTANCE = 86; // Inches (negative is reverse) Needed to exit the 
//   constexpr double LAUNCH_DELAY = 0.5; // Seconds to delay between launch & drive
// }


 * An example command.
 *
 * <p>Note that this extends CommandHelper, rather extending CommandBase
 * directly; this is crucially important, or else the decorator functions in
 * Command will *not* work!
 
class AutoDriveDistance
: public frc2::CommandHelper<frc2::CommandBase, AutoDriveDistance> {
public:
AutoDriveDistance(DriveTrain *drivetrain, double distance);

#ifdef ENABLE_DRIVETRAIN
void Initialize() override;

void Execute() override;

void End(bool interrupted) override;

bool IsFinished() override;
#else // ENABLE_DRIVETRAIN
bool IsFinished() { return true; };
#endif // ENABLE_DRIVETRAIN



private:
DriveTrain *m_driveTrain;
double m_distance_inches = 0.0; // CRE 2022-01-28 Read from shuffleboard
double m_speedOut = 0.0;
};

 */

/** Original CPP
 // Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/AutoDriveDistance.h"

AutoDriveDistance::AutoDriveDistance(DriveTrain *drivetrain, double distance) 
  : m_driveTrain(drivetrain), m_distance_inches(distance) {
  // Use addRequirements() here to declare subsystem dependencies.
  AddRequirements(drivetrain);
}

#ifdef ENABLE_DRIVETRAIN
// Called when the command is initially scheduled.
void AutoDriveDistance::Initialize() {
  m_driveTrain->ResetEncoders();
  // Read the desired travel distance from network tables (shuffleboard driver input)
}

// Called repeatedly when this Command is scheduled to run
void AutoDriveDistance::Execute() {
  // See TeleOpDrive for more filtering information
  constexpr double speedN = 11.0; // length of digital filter
  constexpr double maxSpeed = 0.5;
  constexpr double rotation = 0.0;

  double desiredSpeed = (m_distance_inches > m_driveTrain->GetAverageDistanceInches()) ? maxSpeed : -maxSpeed;
  double speed = (((speedN - 1.0) * m_speedOut) + desiredSpeed) / speedN;
  m_driveTrain->ArcadeDrive(speed, rotation);
  m_speedOut = speed;
}

// Called once the command ends or is interrupted.
void AutoDriveDistance::End(bool interrupted) {
  m_driveTrain->ArcadeDrive(0.0, 0.0);
}

// Returns true when the command should end.
bool AutoDriveDistance::IsFinished() {
  constexpr double epsilon = 5.0;

  return ((fabs(m_distance_inches + copysign(epsilon / 2.0, m_distance_inches)- m_driveTrain->GetAverageDistanceInches())) < epsilon);
}
#endif // ENABLE_DRIVETRAIN

 */