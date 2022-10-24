// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveTrain;

public class AutoDelay extends CommandBase {

  DriveTrain m_driveTrain;
  double m_seconds;
  // double m_seconds; // Don't know how to type convert the NTE GetDouble() -> second_t
  Timer m_timer;  

  
  public AutoDelay(double seconds) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_timer = new Timer(); 
    m_seconds = seconds;
  }
  
  // Called when the command is initially scheduled.
  public void initialize() {
    m_timer.reset();
    m_timer.start();
    // m_seconds = (units::time::second_t)m_driveTrain->m_nte_a_DriveDelay.GetDouble(0.0); // Seconds delay before driving
  }   
  
  // Called repeatedly when this Command is scheduled to run
  // void AutoDelay::Execute() {}
  
  // Called once the command ends or is interrupted.
  public void end(boolean interrupted) {
    // #ifdef DEBUG
    System.out.println("AutoDelay::End()");
    // #endif
  }
  
  // Returns true when the command should end.
  public boolean isFinished() {
    // m_timer.Get() returns a units::time::second_t
    return (double)m_timer.get() > m_seconds;
  }
  
}

/** Original H
 // Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/CommandBase.h>
#include <frc2/command/CommandHelper.h>
#include <frc/Timer.h>
#include "subsystems/DriveTrain.h"


 * An example command.
 *
 * <p>Note that this extends CommandHelper, rather extending CommandBase
 * directly; this is crucially important, or else the decorator functions in
 * Command will *not* work!
 
class AutoDelay
: public frc2::CommandHelper<frc2::CommandBase, AutoDelay> {
public:
explicit AutoDelay(double);

void Initialize() override;

// void Execute() override;

void End(bool interrupted) override;

bool IsFinished() override;

private:
DriveTrain *m_driveTrain;
double m_seconds;
// double m_seconds; // Don't know how to type convert the NTE GetDouble() -> second_t
frc::Timer m_timer;  
};

 */

/** Original CPP
 // Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/AutoDelay.h"

AutoDelay::AutoDelay(double seconds) : m_seconds(seconds) {
  // Use addRequirements() here to declare subsystem dependencies.
  m_timer = frc::Timer(); 
}

// Called when the command is initially scheduled.
void AutoDelay::Initialize() {
  m_timer.Reset();
  m_timer.Start();
  // m_seconds = (units::time::second_t)m_driveTrain->m_nte_a_DriveDelay.GetDouble(0.0); // Seconds delay before driving
}   

// Called repeatedly when this Command is scheduled to run
// void AutoDelay::Execute() {}

// Called once the command ends or is interrupted.
void AutoDelay::End(bool interrupted) {
  #ifdef DEBUG
    printf("AutoDelay::End()\n");
  #endif
}

// Returns true when the command should end.
bool AutoDelay::IsFinished() {
  // m_timer.Get() returns a units::time::second_t
  return (double)m_timer.Get() > m_seconds;
}

 */