// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class Test extends CommandBase {
  /** Creates a new Test. */
  public Test() {
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

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
#include "subsystems/ExampleSubsystem.h"


 * An example command.
 *
 * <p>Note that this extends CommandHelper, rather extending CommandBase
 * directly; this is crucially important, or else the decorator functions in
 * Command will *not* work!
 
class Test
: public frc2::CommandHelper<frc2::CommandBase, Test> {
public:
Test(ExampleSubsystem *, int);

void Initialize() override;

void Execute() override;

void End(bool interrupted) override;

bool IsFinished() override;

private:
ExampleSubsystem *m_example;
int m_direction;
};

 */

/** Original CPP
 // Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/Test.h"
#include "subsystems/ExampleSubsystem.h"

Test::Test(ExampleSubsystem *example, int direction) : m_example{example}, m_direction{direction} {
  // Use addRequirements() here to declare subsystem dependencies.
  AddRequirements(example);
}

// Called when the command is initially scheduled.
void Test::Initialize() {
  m_example->TestRelay(m_direction);
  m_example->TestServo(m_direction);
}

// Called repeatedly when this Command is scheduled to run
void Test::Execute() {}

// Called once the command ends or is interrupted.
void Test::End(bool interrupted) {}

// Returns true when the command should end.
bool Test::IsFinished() {
  return true;
}

 */