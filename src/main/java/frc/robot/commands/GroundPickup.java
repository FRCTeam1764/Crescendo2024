// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.Shooter;
import frc.robot.state.IntakeState;
// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class GroundPickup extends SequentialCommandGroup {
  /** Creates a new GroundPickup. */
  public GroundPickup(Shooter shooter, IntakeSubsystem intakeSubsystem, IntakeState intakeState, boolean isFinished) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands (
    new WristCommand(intakeState, 10000), 
    new IntakeCommand(intakeSubsystem, 0.5), 
    new WristCommand(intakeState, -10000));
    
  }
}
