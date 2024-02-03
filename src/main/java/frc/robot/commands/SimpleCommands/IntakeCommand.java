// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.SimpleCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.Shooter;

public class IntakeCommand extends Command {
  /** Creates a new ClimberCommand. */
  IntakeSubsystem intake;
  Shooter shooter;
  double speed;
  boolean stopatbreakbeam;
  public IntakeCommand(IntakeSubsystem intake, double speed, boolean stopatbreakbeam) {
   // addRequirements(intake);
    // Use addRequirements() here to declare subsystem dependencies.
    this.intake = intake;
    this.speed = speed;
    this.stopatbreakbeam = stopatbreakbeam;
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
  }
  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (stopatbreakbeam == true) {
      return intake.getIntakeBreakbeam();
    }
    return false;
  }
}
