// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.ComplexCommands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.Shooter;
import frc.robot.commands.SimpleCommands.IntakeCommand;
import frc.robot.commands.SimpleCommands.WristCommand;
import frc.robot.constants.CommandConstants;
import frc.robot.state.IntakeState;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class GroundPickup extends SequentialCommandGroup {
  

  /** Creates a new GroundPickup. */
  public GroundPickup(Shooter shooter, IntakeSubsystem intakeSubsystem, IntakeState intakeState) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addRequirements(intakeSubsystem,shooter);
    addCommands (
      new ParallelDeadlineGroup(new WristCommand(intakeSubsystem,intakeState, CommandConstants.INTAKE_DOWN_ENCODERVALUE,true,false), 
      new IntakeCommand(intakeSubsystem, CommandConstants.INTAKE_PICKUP_SPEED,false)
 ),
      

      new ParallelCommandGroup(
        new IntakeCommand(intakeSubsystem, CommandConstants.INTAKE_PICKUP_SPEED,false),
        new WristCommand(intakeSubsystem,intakeState, CommandConstants.INTAKE_DOWN_ENCODERVALUE,false,false)
    )

    );
  }
}
