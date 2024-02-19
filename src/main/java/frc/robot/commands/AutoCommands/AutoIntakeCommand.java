// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.AutoCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.state.IntakeState;
import frc.robot.subsystems.IntakeSubsystem;

public class AutoIntakeCommand extends Command {
  /** Creates a new AutoIntakeCommand. */
  IntakeSubsystem intake;
  double speed;
  public AutoIntakeCommand(IntakeSubsystem intake, IntakeState IntakeState, double speed) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.intake = intake;
    this.speed = speed;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    intake.run(speed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    intake.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return intake.getIntakeBreakbeam();
  }
}
