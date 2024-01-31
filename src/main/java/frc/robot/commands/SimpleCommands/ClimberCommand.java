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
  boolean whichSide;
  public ClimberCommand(ClimberSubsystem climber,int encoderValue,boolean whichSide) {
    addRequirements(climber);
this.climber = climber;
this.encoderValue =encoderValue;
this.whichSide = whichSide;

    // Use addRequirements() here to declare subsystem dependencies.
  }
// hi
  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
if(whichSide == true ){
    climber.ClimberOnRight(encoderValue);
}else{
      climber.ClimberOnLeft(encoderValue);

}
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
if(whichSide == true ){
    climber.ClimberOnRight(encoderValue);
}else{
      climber.ClimberOnLeft(encoderValue);

}  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    climber.ClimberOff();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
