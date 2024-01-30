// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.ComplexCommands;

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
      new simpleWaitCommand(.2),

      new IntakeCommand(intake, 0.1,true),
      new RollerCommand(shooter, -0.1,false)
    
        );


    addCommands(
      rollInPiece,
      new ParallelCommandGroup(
        new WristCommand(intakeState, 100, false, false),
        new IntakeCommand(intake,0.05,false)
      )
    ); 
  }
}