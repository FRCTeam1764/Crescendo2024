// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSubsystem;

public class IntakeCommand extends Command {
  /** Creates a new ClimberCommand. */
  IntakeSubsystem intake;
double speed;
boolean negative;
  public IntakeCommand(IntakeSubsystem intake,boolean negative) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.intake = intake;
    this.speed = speed;
    this.negative = negative;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() 
  {if(negative){
intake.intakeOutRing();
  }else{
    intake.intakeTakeRing();
  }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
   if(negative){
intake.intakeOutRing();
  }else{
    intake.intakeTakeRing();
  }
    
    //intake.intakeTakeRing();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    intake.intakeOff();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
