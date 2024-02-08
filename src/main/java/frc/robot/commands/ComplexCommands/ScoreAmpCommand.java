// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.ComplexCommands;

import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.simpleWaitCommand;
import frc.robot.commands.SimpleCommands.IntakeCommand;
import frc.robot.commands.SimpleCommands.WristCommand;
import frc.robot.constants.CommandConstants;
import frc.robot.constants.Constants;
import frc.robot.state.IntakeState;
import frc.robot.subsystems.IntakeSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ScoreAmpCommand extends SequentialCommandGroup {
  /** Creates a new ScoreAmpCommand. */
  public ScoreAmpCommand(IntakeSubsystem intake, IntakeState state) {
    addRequirements(intake);
    ParallelDeadlineGroup score = new ParallelDeadlineGroup(
      new simpleWaitCommand(0.5),
      new IntakeCommand(intake,-CommandConstants.INTAKE_FAST_SPEED, false)
    );

    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      score,
      new WristCommand(intake,state,CommandConstants.INTAKE_AMP_ENCODERVALUE,false,false)
    );
  }
}
