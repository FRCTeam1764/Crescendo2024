// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.AutoCommands;

import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import frc.robot.commands.simpleWaitCommand;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.SwerveSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class LimeLightAuto extends ParallelRaceGroup {
  /** Creates a new LimeLightAuto. */
  public LimeLightAuto(SwerveSubsystem swerve, LimelightSubsystem limelight,int pipeline ) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
new simpleWaitCommand(.75),
    new LockOnAprilTagAuto(swerve, limelight, pipeline)
    );
  }
}
