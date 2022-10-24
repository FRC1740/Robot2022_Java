// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Intake;

public class Stow extends CommandBase {
  private Intake m_intake;

  /** Creates a new Stow. */
  public Stow(Intake intake) {
    m_intake = intake;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_intake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    System.out.println("Command Stow::Initialize()");
    m_intake.stow();  
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    System.out.println("Command Stow::End()");
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
#include "subsystems/Intake.h"


 * An example command.
 *
 * <p>Note that this extends CommandHelper, rather extending CommandBase
 * directly; this is crucially important, or else the decorator functions in
 * Command will *not* work!
 
class Stow
: public frc2::CommandHelper<frc2::CommandBase, Stow> {
public:
Stow(Intake*);

void Initialize() override;

void Execute() override;

void End(bool interrupted) override;

bool IsFinished() override;

private:
Intake *m_intake;
};

 */

/** Original CPP
 // Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/Stow.h"

Stow::Stow(Intake *intake) : m_intake{intake} {
  // Use addRequirements() here to declare subsystem dependencies.
  //AddRequirements(intake);
}

// Called when the command is initially scheduled.
void Stow::Initialize() {
  printf("Command Stow::Initialize()\n");
  m_intake->Stow();
}

// Called repeatedly when this Command is scheduled to run
void Stow::Execute() {}

// Called once the command ends or is interrupted.
void Stow::End(bool interrupted) {
  printf("Command Stow::End()\n");
}

// Returns true when the command should end.
bool Stow::IsFinished() {
  return true;
}

 */