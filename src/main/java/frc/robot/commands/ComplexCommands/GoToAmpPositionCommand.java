// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.ComplexCommands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.IntakeCommand;
import frc.robot.commands.RollerCommand;
import frc.robot.commands.WristCommand;
import frc.robot.commands.simpleWaitCommand;
import frc.robot.constants.CommandConstants;
import frc.robot.state.IntakeState;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.Shooter;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class GoToAmpPositionCommand extends SequentialCommandGroup {
  /** Creates a new scoreAmpCommand. */
  public GoToAmpPositionCommand(IntakeState intakeState, IntakeSubsystem intake, Shooter shooter) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addRequirements(intake, shooter);
        ParallelDeadlineGroup rollInPiece = new ParallelDeadlineGroup(
new simpleWaitCommand(.1),
new ParallelCommandGroup(
new IntakeCommand(intake, 0.1),
new RollerCommand(shooter, -0.1)
)
        );
    ParallelDeadlineGroup intakeUp = new ParallelDeadlineGroup(
      new WristCommand(intakeState, 100, true, false), 
      
      new IntakeCommand(intake,-0.1)
    );


    addCommands(
      rollInPiece,
    new  ParallelCommandGroup(
       new WristCommand(intakeState, 100, false, false),
       new IntakeCommand(intake,0.05)
    )


    );
  }
}
