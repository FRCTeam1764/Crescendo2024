// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.ComplexCommands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.simpleWaitCommand;
import frc.robot.commands.SimpleCommands.IntakeCommand;
import frc.robot.commands.SimpleCommands.RollerCommand;
import frc.robot.commands.SimpleCommands.WristCommand;
import frc.robot.constants.CommandConstants;
import frc.robot.state.IntakeState;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.Shooter;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class returnGroundPickUp extends SequentialCommandGroup {
  /** Creates a new returnGroundPickUp. */
  public returnGroundPickUp(IntakeSubsystem intake, Shooter shooter, IntakeState intakeState) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addRequirements(intake, shooter);
  ParallelDeadlineGroup intakeBack = new ParallelDeadlineGroup(
      new WristCommand(intake,intakeState,CommandConstants.INTAKE_UP_ENCODERVALUE,true,2)
    , 
//    new InstantCommand()
      new IntakeCommand(intake,CommandConstants.INTAKE_SLOW_SPEED,false)
    );



ParallelRaceGroup movePiece =  new ParallelRaceGroup(

new simpleWaitCommand(0.3),
      new IntakeCommand(intake, -CommandConstants.INTAKE_FAST_SPEED,false),
      new RollerCommand(shooter, CommandConstants.SHOOTER_INTAKE_SPEED,true)
    );
    addCommands(
intakeBack,
movePiece



    );
  }
}
