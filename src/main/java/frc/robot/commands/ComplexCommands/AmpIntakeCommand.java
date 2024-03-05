// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.ComplexCommands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.simpleWaitCommand;
import frc.robot.commands.SimpleCommands.IntakeCommand;
import frc.robot.commands.SimpleCommands.WristCommand;
import frc.robot.constants.CommandConstants;
import frc.robot.state.IntakeState;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.Shooter;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AmpIntakeCommand extends SequentialCommandGroup {
  /** Creates a new SpitOutNoteCommand. */
  public AmpIntakeCommand(Shooter shooter, IntakeSubsystem intakeSubsystem, IntakeState intakeState) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addRequirements(shooter,intakeSubsystem);
    addCommands(
 new ParallelDeadlineGroup(
  new simpleWaitCommand(.4)
  ,
      new ParallelCommandGroup(
        new IntakeCommand(intakeSubsystem,-.65,false), // at 12.5 volts
        new WristCommand(intakeSubsystem,intakeState, CommandConstants.INTAKE_AMP_ENCODERVALUE,false,false)
    )
 ),
 new WristCommand(intakeSubsystem,intakeState,CommandConstants.INTAKE_UP_ENCODERVALUE,true,false)
    );
  }
}
