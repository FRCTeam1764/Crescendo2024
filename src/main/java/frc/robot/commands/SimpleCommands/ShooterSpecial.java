// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.SimpleCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.ComplexCommands.Shoot;
import frc.robot.commands.ComplexCommands.returnGroundPickUp;
import frc.robot.subsystems.Shooter;

public class ShooterSpecial extends Command {
  /** Creates a new ShooterSpecial. */
  Shooter shooter;
  double speed;
  boolean stopatbreakbeam;
  public ShooterSpecial(Shooter shooter, double sped, boolean stopatbreakbeam) {
this.shooter = shooter;
    speed = sped;
this.stopatbreakbeam = stopatbreakbeam;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    shooter.shooterPID(speed);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    shooter.shooterPID(speed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    shooter.shooterPIDOff();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
if(stopatbreakbeam){
    return !shooter.RollerBreakBeamBroken();
}
return false;
  }
}
