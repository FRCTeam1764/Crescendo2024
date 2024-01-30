
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.ComplexCommands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.IntakeCommand;
import frc.robot.commands.RollerCommand;
import frc.robot.commands.ShooterCommand;
import frc.robot.commands.simpleWaitCommand;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.Shooter;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class Shoot extends SequentialCommandGroup {
  /** Creates a new Shoot. */
  public Shoot(Shooter shooter, IntakeSubsystem intake) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addRequirements(shooter, intake);

    ParallelDeadlineGroup shootprep = new ParallelDeadlineGroup(
      new simpleWaitCommand(1),
      new ShooterCommand(shooter, true)
      );
          ParallelDeadlineGroup fire = new ParallelDeadlineGroup(
      new simpleWaitCommand(0.4),
  new ParallelCommandGroup(
      new ShooterCommand(shooter, true),
      new RollerCommand(shooter,0.2),
      new IntakeCommand(intake, 0.3)
)

          );
    addCommands(
      shootprep,
      fire
    );
  }
}
