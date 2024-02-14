// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.SimpleCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ClimberSubsystem;

public class ClimberCommand extends Command {
  /** Creates a new ClimberCommand. */
  ClimberSubsystem climber;
  double encoderValue;
double encoderValue2;
  public ClimberCommand(ClimberSubsystem climber,double encoderValue,double encoderValue2) {
this.climber = climber;
this.encoderValue =encoderValue;
this.encoderValue2 = encoderValue2;

    // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(climber);

  }
// hi
  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    climber.ClimberOnRight(encoderValue);

      climber.ClimberOnLeft(encoderValue2);


  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
 //   climber.ClimberOnRight(encoderValue);
  //  climber.ClimberOnLeft(encoderValue2);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    climber.ClimberOff(); //TODO: MAKE BETTER
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
