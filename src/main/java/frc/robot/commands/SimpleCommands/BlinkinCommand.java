// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.SimpleCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.BlinkinSubsystem;

public class BlinkinCommand extends Command {
  /** Creates a new BlinkinCommand. */
  BlinkinSubsystem Blinkin;
  double color; //https://www.revrobotics.com/content/docs/REV-11-1105-UM.pdf page 14
  
  public BlinkinCommand(BlinkinSubsystem Blinkin, double color) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.Blinkin = Blinkin;
    this.color = color;
    addRequirements(Blinkin);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    Blinkin.setLEDs(color); // was -.79
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    Blinkin.setLEDs(-.45); // was -.45
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
