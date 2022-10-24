// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Climber;

public class ExtendClimber extends CommandBase {

  private Climber m_climber;

  /** Creates a new ExtendClimber. */
  public ExtendClimber(Climber climber  ) {
    m_climber = climber;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_climber);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_climber.extend();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_climber.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }

}

/** Original H
 // Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/CommandBase.h>
#include <frc2/command/CommandHelper.h>
#include "subsystems/Climber.h"


 * An example command.
 *
 * <p>Note that this extends CommandHelper, rather extending CommandBase
 * directly; this is crucially important, or else the decorator functions in
 * Command will *not* work!
 
class ExtendClimber
: public frc2::CommandHelper<frc2::CommandBase, ExtendClimber> {
public:
ExtendClimber(Climber *);

void Initialize() override;

void Execute() override;

void End(bool interrupted) override;

bool IsFinished() override;

private:
Climber *m_climber;
};

 */

/** Original CPP
 // Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/ExtendClimber.h"

ExtendClimber::ExtendClimber(Climber *climber) : m_climber{climber} {
  // Use addRequirements() here to declare subsystem dependencies.
  AddRequirements(climber);
}

// Called when the command is initially scheduled.
void ExtendClimber::Initialize() {}

// Called repeatedly when this Command is scheduled to run
void ExtendClimber::Execute() {
  m_climber->Extend();
}

// Called once the command ends or is interrupted.
void ExtendClimber::End(bool interrupted) {
  m_climber->Stop();
}

// Returns true when the command should end.
bool ExtendClimber::IsFinished() {
  return false;
}

 */