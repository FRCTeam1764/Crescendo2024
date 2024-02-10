// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ClimberSubsystem;

public class testClimberRight extends Command {
  /** Creates a new testClimber. */
  double speed;
  ClimberSubsystem climber;
  public testClimberRight(ClimberSubsystem climber, double speed) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.speed = speed;
    this.climber = climber;
    addRequirements(climber);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
climber.ClimberRightTest(speed);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
climber.ClimberRightTest(speed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    climber.ClimberOff();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
   // return false;
    return climber.getLimitSwitch2();
  }
}
