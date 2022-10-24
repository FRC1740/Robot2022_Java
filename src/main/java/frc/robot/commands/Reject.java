// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Intake;

public class Reject extends CommandBase {
  private Intake m_intake;

  /** Creates a new Reject. */
  public Reject(Intake intake) {
    m_intake = intake;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_intake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    System.out.println("Reject::Initialize()");
    m_intake.reject();  
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_intake.load();
    System.out.println("Reject::End()");  
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
#include "subsystems/Intake.h"


 * An example command.
 *
 * <p>Note that this extends CommandHelper, rather extending CommandBase
 * directly; this is crucially important, or else the decorator functions in
 * Command will *not* work!
 
class Reject
: public frc2::CommandHelper<frc2::CommandBase, Reject> {
public:
Reject(Intake *);

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

#include "commands/Reject.h"

Reject::Reject(Intake *intake) : m_intake{intake} {
  // Use addRequirements() here to declare subsystem dependencies.
  //AddRequirements(intake);
}

// Called when the command is initially scheduled.
void Reject::Initialize() {
  printf("Reject::Initialize()\n");
  m_intake->Reject();
}

// Called repeatedly when this Command is scheduled to run
void Reject::Execute() {}

// Called once the command ends or is interrupted.
void Reject::End(bool interrupted) {
  m_intake->Load();
  printf("Reject::End()\n");
}

// Returns true when the command should end.
bool Reject::IsFinished() {
  return false;
}

 */