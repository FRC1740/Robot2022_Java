// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Climber;

public class Climb extends CommandBase {

  private Climber m_climber;

  public static class ConClimber {
    public static double MOTOR_ID = 8.0;
    public static double MOTOR_SPEED = 1.0;
  }
  
  /** Creates a new Climb. */
  public Climb(Climber climber) {
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
    m_climber.climb();
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
 
class Climb
: public frc2::CommandHelper<frc2::CommandBase, Climb> {
public:
Climb(Climber *);

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

#include "commands/Climb.h"

namespace ConClimber {
  constexpr double MOTOR_ID = 8.0;
  constexpr double MOTOR_SPEED = 1.0;
}

Climb::Climb(Climber *climber) : m_climber{climber} {
  // Use addRequirements() here to declare subsystem dependencies.
  AddRequirements(climber);
}

// Called when the command is initially scheduled.
void Climb::Initialize() {
}

// Called repeatedly when this Command is scheduled to run
void Climb::Execute() {
  m_climber->Climb();
}

// Called once the command ends or is interrupted.
void Climb::End(bool interrupted) {
  m_climber->Stop();
}

// Returns true when the command should end.
bool Climb::IsFinished() {
  return false;
}

 */