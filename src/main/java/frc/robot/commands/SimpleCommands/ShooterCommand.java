// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.SimpleCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Shooter;

public class ShooterCommand extends Command {

  Shooter shooter;
  boolean OnOff;
  
  /** Creates a new Shooter. */
  public ShooterCommand(Shooter shooter, boolean OnOff) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.shooter = shooter;
    this.OnOff = OnOff;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if (OnOff == true){
    shooter.shooterOn();
    } else{
      shooter.shooterOff();
    }
  }
  
  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {  
    if (OnOff == true){
    shooter.shooterOn();
    } else{
      shooter.shooterOff();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    shooter.shooterOff();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
