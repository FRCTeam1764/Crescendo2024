// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.ComplexCommands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import frc.robot.commands.simpleWaitCommand;
import frc.robot.commands.SimpleCommands.ClimberCommand;
import frc.robot.subsystems.ClimberSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ClimberRelease extends ParallelDeadlineGroup {
  /** Creates a new ClimberRelease. */
  public ClimberRelease(ClimberSubsystem climber) {
    // Add the deadline command in the super() call. Add other commands using
    // addCommands().
    super(new simpleWaitCommand(0.1));
//addRequirements(climber);
    addCommands(new ClimberCommand(climber,.4));
    // addCommands(new FooCommand(), new BarCommand());
  }
}
