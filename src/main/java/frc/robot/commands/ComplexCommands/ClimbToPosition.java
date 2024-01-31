// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.ComplexCommands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.commands.SimpleCommands.ClimberCommand;
import frc.robot.subsystems.ClimberSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ClimbToPosition extends ParallelCommandGroup {
  /** Creates a new ClimbToPosition. */
  public ClimbToPosition(ClimberSubsystem climber, int encoder1, int encoder2) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    // true = right
    // false = left
    addRequirements(climber);
    addCommands( new ClimberCommand(climber,encoder2,true),
         new ClimberCommand(climber,encoder1,false)

    );
  }
}
