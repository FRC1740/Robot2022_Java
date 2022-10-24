// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.DriveTrain.ConDriveTrain;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Launcher;

public class AutoDrive extends ParallelCommandGroup {
  
  private DriveTrain m_driveTrain;
  private Launcher m_launcher;
  private Intake m_intake;
  
  // The enum used as keys for selecting the command to run.
  // enum CommandSelector { ONE, TWO, THREE };
  
  public AutoDrive(DriveTrain drivetrain, Launcher launcher, Intake intake) {
    m_driveTrain = drivetrain;
    m_launcher = launcher;
    m_intake = intake;

    // SHuffleboard parameters NOT refreshing this way. Moving them into the specific commands instead of
    // passing them as arguments to the command seems to function as desired.
    // units::time::second_t a = .5_s; // FIXME: Temporary pending proper type conversion double -> second_t
    // double a = drivetrain->m_nte_a_DriveDelay.GetDouble(0.0); // Drive delay (seconds)
    // double c = drivetrain->m_nte_c_DriveTurnAngle.GetDouble(0.0); // Turning Angle (degrees)
    
    // #if 1
    // Should be set in AutonomousInit() as well as DisabledPeriodic()
    //m_driveTrain->SetAutonomousParameters();
    
    double m_distance = m_driveTrain.m_autoDistance;
    int m_mode = (int) (m_driveTrain.m_autoDriveMode+.5);
    System.out.println("m_mode: " + m_mode + ", m_autoDriveMode: " + m_driveTrain.m_autoDriveMode);
    System.out.println("Driving distance: " + m_distance);
    System.out.println("Drive delay: " + m_driveTrain.m_autoDriveDelay);
  
    // Uncomment one of the following to force a specific Autonomous mode 
    // m_mode = ConDriveTrain::AUTONOMOUS_MODE_2_BALL;
    // m_mode = ConDriveTrain::AUTONOMOUS_MODE_LAUNCH_DELAY_MOVE;
    
    // Add your commands here, e.g.
    // AddCommands(FooCommand(), BarCommand());
    switch (m_mode) {
      case ConDriveTrain.AUTONOMOUS_MODE_2_BALL:
        System.out.println("Autonomous " + m_mode + " Ball Mode");
        addCommands(
          new SequentialCommandGroup(
            new AutoDelay(0.5),
            new InstantCommand(m_launcher::launch, m_launcher),
            // new InstantCommand(m_launcher.launchErnie(), m_launcher),
            new AutoDelay(0.5),
            new InstantCommand(m_launcher::retract, m_launcher),
            // new InstantCommand(m_launcher.retractErnie(), m_launcher),
            new InstantCommand(intake::deploy, intake),
            new AutoDriveDistance(m_driveTrain, m_distance),
            new InstantCommand(intake::stow, intake),
            new AutoDelay(0.5), 
            new AutoDriveDistance(m_driveTrain, -m_distance),
            // new InstantCommand(intake.deploy(), intake), // Only needed to avoid motor interference
            new AutoDelay(0.5),
            new InstantCommand(m_launcher::launch, m_launcher),
            // new InstantCommand(m_launcher.launchErnie(), m_launcher),
            new AutoDelay(0.5),
            // new InstantCommand(intake.stow(), intake),
            new InstantCommand(m_launcher::retract, m_launcher)
            // new InstantCommand(m_launcher.retractErnie(), m_launcher),

            // possible additional ball pickup?
            // new AutoTurn(m_driveTrain, 45.0),
            // new AutoDelay(0.5),
            // new AutoDriveDistance(m_driveTrain, 84.0),
          )
        );
      break;
        
      case ConDriveTrain.AUTONOMOUS_MODE_LAUNCH_DELAY_MOVE:
        System.out.println("Autonomous Mode " + m_mode + ": Shoot-Delay-Move");
        addCommands (
          new SequentialCommandGroup (
            new InstantCommand(m_launcher::launch, m_launcher),
            // new InstantCommand(m_launcher::launchErnie, m_launcher),
            new AutoDelay(m_driveTrain.m_autoDriveDelay),
            new InstantCommand(m_launcher::retract, m_launcher),
            // new InstantCommand(m_launcher::retractErnie, m_launcher),
            new AutoDriveDistance(m_driveTrain, m_distance)
          )
        );
        break;
          
      case ConDriveTrain.AUTONOMOUS_MODE_5_BALL:
      break;
          
        // Default DO NOTHING
      default:
      break;
          
          // #if 0
          // // Add autonomous drive & launcher commands
          // AddCommands (
          //   frc2::SequentialCommandGroup { AutoDelay(1.5_s), AutoDriveDistance(m_driveTrain), AutoTurn(m_driveTrain) },
          //   frc2::SequentialCommandGroup{ AutoDelay(1.0_s), 
          //     frc2::ParallelRaceGroup{ Deploy(intake), AutoDelay(1.0_s) }
          //   }
          //   //
          //   //frc2::ParallelRaceGroup{ Launch(m_launcher), AutoDelay() },
          //   //frc2::SequentialCommandGroup{ AutoDelay(1.0_s),
          //     //                              frc2::ParallelRaceGroup{ Deploy(intake), AutoDelay(1.0_s) }
          //     //                            }
          //     //
          //     );
          //     #endif  // defined(ENABLE_LAUNCHER)
              
          //   } // switch(mode)
          //   #endif // if 0
            
          // }
          
      }
    }
  }
        
 /** Original H
         // Copyright (c) FIRST and other WPILib contributors.
         // Open Source Software; you can modify and/or share it under the terms of
         // the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/CommandBase.h>
#include <frc2/command/CommandHelper.h>
#include <frc2/command/SequentialCommandGroup.h>
#include <frc2/command/ParallelCommandGroup.h>
#include <frc2/command/ParallelRaceGroup.h>
#include "subsystems/DriveTrain.h"
#include "subsystems/Launcher.h"
#include "subsystems/Intake.h"


 * An example command.
 *
 * <p>Note that this extends CommandHelper, rather extending CommandBase
 * directly; this is crucially important, or else the decorator functions in
 * Command will *not* work!
 
class AutoDrive
: public frc2::CommandHelper<frc2::ParallelCommandGroup, AutoDrive> {

public:
explicit AutoDrive(DriveTrain *drivetrain, Launcher *launcher, Intake *intake);

private:
DriveTrain *m_driveTrain;
Launcher *m_launcher;
Intake *m_intake;

// The enum used as keys for selecting the command to run.
enum CommandSelector { ONE, TWO, THREE };

// An example selector method for the selectcommand.  Returns the selector
// that will select which command to run.  Can base this choice on logical
// conditions evaluated at runtime.
CommandSelector Select() { return ONE; }

};

 */

/** Original CPP
 // Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/AutoDrive.h"
#include "commands/AutoDriveDistance.h"
#include "commands/AutoDelay.h"
#include "commands/AutoTurn.h"
#include "commands/Deploy.h"
#include "commands/Stow.h"
#include "commands/Launch.h"
#include <frc2/command/SequentialCommandGroup.h>
#include <frc2/command/ParallelCommandGroup.h>
#include <frc2/command/ParallelRaceGroup.h>
#include <frc2/command/InstantCommand.h>
#include <frc2/command/StartEndCommand.h>
#include <frc2/command/SelectCommand.h>

AutoDrive::AutoDrive(DriveTrain *drivetrain, Launcher *launcher, Intake *intake) {
  m_driveTrain = drivetrain;
  m_launcher = launcher;
  m_intake = intake;

    // SHuffleboard parameters NOT refreshing this way. Moving them into the specific commands instead of
    // passing them as arguments to the command seems to function as desired.
    // units::time::second_t a = .5_s; // FIXME: Temporary pending proper type conversion double -> second_t
    // double a = drivetrain->m_nte_a_DriveDelay.GetDouble(0.0); // Drive delay (seconds)
    // double c = drivetrain->m_nte_c_DriveTurnAngle.GetDouble(0.0); // Turning Angle (degrees)

  // An example selectcommand.  Will select from the three commands based on the
    // value returned by the selector method at runtime.  Note that selectcommand
    // takes a generic type, so the selector does not have to be an enum; it could
    // be any desired type (string, integer, boolean, double...)

    #if 0
    frc2::SelectCommand<CommandSelector> m_exampleSelectCommand{
        [this] { return Select(); },
        // Maps selector values to commands
        std::pair{ONE, frc2::SequentialCommandGroup {
                        frc2::PrintCommand{"Command one was selected!"},
                        AutoDriveDistance(drivetrain, m_distance),
                        }  },
        std::pair{TWO, frc2::PrintCommand{"Command two was selected!"}
                },
        std::pair{THREE, frc2::PrintCommand{"Command three was selected!"}
                }
        };
    #endif
    #if 1
    // Should be set in AutonomousInit() as well as DisabledPeriodic()
    //m_driveTrain->SetAutonomousParameters();

    double m_distance = m_driveTrain->m_autoDistance;
    int m_mode = static_cast <int>(m_driveTrain->m_autoDriveMode+.5);
    printf("m_mode: %d, m_autoDriveMode: %f\n", m_mode, m_driveTrain->m_autoDriveMode);
    printf("Driving distance: %f\n", m_distance);
    printf("Drive delay: %f\n", m_driveTrain->m_autoDriveDelay);

    // Uncomment one of the following to force a specific Autonomous mode 
    // m_mode = ConDriveTrain::AUTONOMOUS_MODE_2_BALL;
    // m_mode = ConDriveTrain::AUTONOMOUS_MODE_LAUNCH_DELAY_MOVE;

    // Add your commands here, e.g.
    // AddCommands(FooCommand(), BarCommand());
    switch (m_mode) {
      case ConDriveTrain::AUTONOMOUS_MODE_2_BALL:
        printf("Autonomous %d Ball Mode\n", m_mode);
        AddCommands (
          frc2::SequentialCommandGroup { 
              AutoDelay(0.5),
              frc2::InstantCommand( [&] { m_launcher->Launch(); }, { m_launcher }),
              //frc2::InstantCommand( [&] { m_launcher->LaunchErnie(); }, { m_launcher }),
              AutoDelay(0.5),
              frc2::InstantCommand( [&] { m_launcher->Retract(); }, { m_launcher }),
              // frc2::InstantCommand( [&] { m_launcher->RetractErnie(); }, { m_launcher }),
              frc2::InstantCommand( [intake] { intake->Deploy(); }, { intake }), 
              AutoDriveDistance(m_driveTrain, m_distance),
              frc2::InstantCommand( [intake] { intake->Stow(); }, {intake}),
              AutoDelay(0.5), 
              AutoDriveDistance(m_driveTrain, -m_distance),
//              frc2::InstantCommand( [intake] { intake->Deploy(); }, { intake }), // Only needed to avoid motor interference
              AutoDelay(0.5),
              frc2::InstantCommand( [&] { m_launcher->Launch(); }, { m_launcher }),
              //frc2::InstantCommand( [&] { m_launcher->LaunchErnie(); }, { m_launcher }),                                  
              AutoDelay(0.5),
//              frc2::InstantCommand( [intake] { intake->Stow(); }, {intake}),
              frc2::InstantCommand( [&] { m_launcher->Retract(); }, { m_launcher }),
              // frc2::InstantCommand( [&] { m_launcher->RetractErnie(); }, { m_launcher }),
              #if 0
              // possible additional ball pickup?
              AutoTurn(m_driveTrain, 45.0),
              AutoDelay(0.5),
              AutoDriveDistance(m_driveTrain, 84.0),
              #endif
            } );
      break;

      case ConDriveTrain::AUTONOMOUS_MODE_LAUNCH_DELAY_MOVE:
        printf("Autonomous Mode %d: Shoot-Delay-Move\n", m_mode);
        AddCommands (
            frc2::SequentialCommandGroup {
              frc2::InstantCommand( [&] { m_launcher->Launch(); }, { m_launcher }),
              // frc2::InstantCommand( [&] { m_launcher->LaunchErnie(); }, { m_launcher }),
              AutoDelay(m_driveTrain->m_autoDriveDelay),
              frc2::InstantCommand( [&] { m_launcher->Retract(); }, { m_launcher }),
              //frc2::InstantCommand( [&] { m_launcher->RetractErnie(); }, { m_launcher }),
              AutoDriveDistance(m_driveTrain, m_distance),
          } );
      break;

      case ConDriveTrain::AUTONOMOUS_MODE_5_BALL:
      break;

      // Default DO NOTHING
      default:
      break;
    
    #if 0
    // Add autonomous drive & launcher commands
    AddCommands (
      frc2::SequentialCommandGroup { AutoDelay(1.5_s), AutoDriveDistance(m_driveTrain), AutoTurn(m_driveTrain) },
      frc2::SequentialCommandGroup{ AutoDelay(1.0_s), 
                                    frc2::ParallelRaceGroup{ Deploy(intake), AutoDelay(1.0_s) }
                                  }
      //
      //frc2::ParallelRaceGroup{ Launch(m_launcher), AutoDelay() },
      //frc2::SequentialCommandGroup{ AutoDelay(1.0_s),
      //                              frc2::ParallelRaceGroup{ Deploy(intake), AutoDelay(1.0_s) }
      //                            }
      //
      );
      #endif  // defined(ENABLE_LAUNCHER)
  
      } // switch(mode)
      #endif // if 0
  
  }
  
 */