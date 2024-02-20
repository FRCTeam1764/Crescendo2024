// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.ComplexCommands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import frc.robot.commands.SimpleCommands.IntakeCommand;
import frc.robot.commands.SimpleCommands.RollerCommand;
import frc.robot.constants.CommandConstants;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.Shooter;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class indexRingCommand extends ParallelDeadlineGroup {
  /** Creates a new indexRingCommand. */
  public indexRingCommand(Shooter shooter, IntakeSubsystem intake) {
    // Add the deadline command in the super() call. Add other commands using
    // addCommands().
    super(new RollerCommand(shooter,CommandConstants.SHOOTER_INTAKE_SPEED,true));
    addRequirements(shooter,intake);
     addCommands(new IntakeCommand(intake,-CommandConstants.INTAKE_FAST_SPEED,false));
  }
}