// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.ComplexCommands;

import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.simpleWaitCommand;
import frc.robot.commands.SimpleCommands.BlinkinCommand;
import frc.robot.commands.SimpleCommands.ClimberCommand;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.Blinkin;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class EmoteCommand extends SequentialCommandGroup {
  /** Creates a new EmoteCommand. */
  public EmoteCommand(ClimberSubsystem climberSubsystem, Blinkin blinkin) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    ParallelDeadlineGroup left = new ParallelDeadlineGroup(
      new simpleWaitCommand(1.5),
      new ClimberCommand(climberSubsystem, -150, -110)
    );
    ParallelDeadlineGroup right = new ParallelDeadlineGroup(
      new simpleWaitCommand(1.5),
      new ClimberCommand(climberSubsystem, -110, -150)
    );
    addCommands(
      new ParallelDeadlineGroup(
        new SequentialCommandGroup(left, right, left, right, left, right),
        new BlinkinCommand(blinkin, false) // default color is set to party palette
        ));
  }
}
