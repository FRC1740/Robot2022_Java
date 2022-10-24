// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Launcher;

public class Launch extends CommandBase {

  private Launcher m_launcher;
  
  /** Creates a new Launch. */
  public Launch(Launcher launcher) {
    m_launcher = launcher;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_launcher);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    System.out.println("Command Launch::Initialize()");
    m_launcher.launch();  
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    System.out.println("Command Launch::End()");
    m_launcher.retract();  
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }

}

/** Original H
 // Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/CommandBase.h>
#include <frc2/command/CommandHelper.h>
#include "subsystems/Launcher.h"

class Launch
    : public frc2::CommandHelper<frc2::CommandBase, Launch> {      
 public:
  explicit Launch(Launcher *);
  void Initialize() override;
  void Execute();
  void End();
  bool IsFinished() override;

 private:
  Launcher *m_launcher;
};

 */

/** Original CPP
 // Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/Launch.h"

Launch::Launch(Launcher *launcher) : m_launcher{launcher} {
  //AddRequirements(launcher);
}

// PID Command does not use these three methods...
void Launch::Initialize() {
  printf("Command Launch::Initialize()\n");
  m_launcher->Launch();
}

void Launch::Execute() {}

void Launch::End() {
  printf("Command Launch::End()\n");
  m_launcher->Retract();
}

// Returns true when the command should end.
bool Launch::IsFinished() {
  return true;
}

 */