// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.ComplexCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ClimberSubsystem;

public class ClimbDownCommand extends Command {
  /** Creates a new ClimbDownCommand. */
  ClimberSubsystem climber;
  public ClimbDownCommand(ClimberSubsystem climber) {
    this.climber = climber;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    climber.ClimberLefttTest(0.6);

    climber.ClimberRightTest(0.6);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
        climber.ClimberLefttTest(0.6);

    climber.ClimberRightTest(0.6);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
        climber.ClimberLefttTest(0);
    climber.ClimberRightTest(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return (climber.getLimitSwitch() && climber.getLimitSwitch2()) 
    ||(Math.abs(climber.getEncoderValue()) <=6 &&Math.abs(climber.getEncoderValue2()) <=6 );
  }
}
