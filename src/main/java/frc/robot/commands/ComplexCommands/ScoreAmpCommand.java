// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.ComplexCommands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.simpleWaitCommand;
import frc.robot.commands.SimpleCommands.AmpCommand;
import frc.robot.commands.SimpleCommands.IntakeCommand;
import frc.robot.commands.SimpleCommands.RollerCommand;
import frc.robot.commands.SimpleCommands.ShooterAmpCommand;
import frc.robot.subsystems.AmpScoreSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.Shooter;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ScoreAmpCommand extends SequentialCommandGroup {
  /** Creates a new newScoreAmpCommand. */
  public ScoreAmpCommand(Shooter shooter, IntakeSubsystem intake, AmpScoreSubsystem amp) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    ParallelDeadlineGroup shoot = new ParallelDeadlineGroup(
      new simpleWaitCommand(1),
        new ParallelCommandGroup(
          new ShooterAmpCommand(shooter),
          new RollerCommand(shooter,0.2,false), // speed needs tuning, half of SHOOTER_INTAKE_SPEED
          new IntakeCommand(intake, -0.25,false) // speed needs tuning, half of INTAKE_FAST_SPEED
        )
    );
    
    addCommands(shoot, 
    new AmpCommand(amp, 0)); // EDITTTTTTTTTTT
  }
}
