// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.AutoCommands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.simpleWaitCommand;
import frc.robot.commands.ComplexCommands.indexRingCommand;
import frc.robot.commands.DriveCommands.DriveBasic;
import frc.robot.commands.SimpleCommands.IntakeCommand;
import frc.robot.commands.SimpleCommands.WristCommand;
import frc.robot.constants.CommandConstants;
import frc.robot.state.IntakeState;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.SwerveSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AutoGroundPickUp extends SequentialCommandGroup {
  /** Creates a new AutoGroundPickUp. */
  public AutoGroundPickUp(SwerveSubsystem swerve,IntakeSubsystem intakeSubsystem, IntakeState intakeState,Shooter shooter) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
 
      new ParallelRaceGroup(new simpleWaitCommand(1),
        new IntakeCommand(intakeSubsystem, CommandConstants.INTAKE_PICKUP_SPEED,true),
        new WristCommand(intakeSubsystem,intakeState, CommandConstants.INTAKE_DOWN_ENCODERVALUE,false,false),
        new DriveBasic(swerve, 0.4)
    )

);



  }
}
