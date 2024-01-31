// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.SimpleCommands;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.libraries.internal.LazyTalonFX;
import frc.robot.subsystems.Shooter;

public class RollerCommand extends Command {
  /** Creates a new RollerCommand. */
  Shooter shooter;
  double speed;
  public LazyTalonFX holderRoller;
  boolean EndWithBreakBeam;

  public RollerCommand(Shooter shooter, double speed, boolean EndWithBreakBeam) {
this.shooter = shooter;
this.speed = speed;
this.EndWithBreakBeam = EndWithBreakBeam;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    shooter.roller(speed);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
       shooter.roller(speed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
           shooter.roller(0);

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (EndWithBreakBeam == true) {
      return shooter.RollerBreakBeamBroken();
    }
    return false;
  }
  
}
