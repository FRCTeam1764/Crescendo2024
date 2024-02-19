// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.SimpleCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.AmpScoreSubsystem;

public class AmpCommand extends Command {
  /** Creates a new AmpCommand. */
  AmpScoreSubsystem amp;
  double desired;

  public AmpCommand() {
    // Use addRequirements() here to declare subsystem dependencies.
    this.amp = amp;
    this.desired = desired;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    amp.setPidPosition(desired); // maybe just set it permanent up variable? figure out how high it has to go
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    amp.setPidPosition(desired);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    amp.setPidPosition(0); // go back down
  }
  
  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
