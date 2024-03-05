// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.ComplexCommands;

import edu.wpi.first.wpilibj2.command.Command;
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
public class GoToAmpPositionCommand extends SequentialCommandGroup {
  /** Creates a new scoreAmpCommand. */
  public GoToAmpPositionCommand(IntakeState intakeState, IntakeSubsystem intake, Shooter shooter) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addRequirements(intake, shooter);
    ParallelRaceGroup rollInPiece = new ParallelRaceGroup(
      new simpleWaitCommand(.4),

      new IntakeCommand(intake, CommandConstants.INTAKE_SLOW_SPEED,true),// THIS MAY BE BAD
      new RollerCommand(shooter, -CommandConstants.SHOOTER_INTAKE_SPEED,false)
    
        );


    addCommands(
      rollInPiece,

      new ParallelCommandGroup(
        new WristCommand(intake,intakeState, CommandConstants.INTAKE_AMP_ENCODERVALUE, false, false)
      )
    ); 
  }
}
