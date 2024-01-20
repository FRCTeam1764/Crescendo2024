// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.state.IntakeState;

public class WristCommand extends Command {
  /** Creates a new WristCommand. */
  IntakeState state;
  int desired;
  public WristCommand(IntakeState intakeState, int desiredEncoderValue) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.state = intakeState;
    this.desired = desiredEncoderValue;
  }


  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    state.setEncoderValue(desired);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
     state.setEncoderValue(desired);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
     state.setEncoderValue(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
