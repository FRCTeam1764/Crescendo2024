// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.ComplexCommands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.simpleWaitCommand;
import frc.robot.commands.SimpleCommands.AmpCommand;
import frc.robot.commands.SimpleCommands.IntakeCommand;
import frc.robot.commands.SimpleCommands.RollerCommand;
import frc.robot.commands.SimpleCommands.ShooterSpecial;
import frc.robot.commands.SimpleCommands.WristCommand;
import frc.robot.constants.CommandConstants;
import frc.robot.constants.Constants;
import frc.robot.state.IntakeState;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.Shooter;
import pabeles.concurrency.ConcurrencyOps.NewInstance;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ScoreAmpCommand extends SequentialCommandGroup {
  /** Creates a new ScoreAmpCommand. */
  public ScoreAmpCommand(IntakeSubsystem intake, Shooter shooter) {
    addRequirements(intake);
    ParallelDeadlineGroup score = new ParallelDeadlineGroup(
      new simpleWaitCommand(1),
      new ShooterSpecial(shooter,5,false)

    );

    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      score,
      
      new ParallelDeadlineGroup(

      new ShooterSpecial(shooter,5,false),
      new RollerCommand(shooter, 0.1, false),
      new IntakeCommand(intake, -0.1, false)
      )
// new ParallelDeadlineGroup( new simpleWaitCommand(1),
// new ShooterSpecial(shooter, 30, false)
// )
    );
  }
}
