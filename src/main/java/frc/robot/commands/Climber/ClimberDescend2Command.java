// // Copyright (c) FIRST and other WPILib contributors.
// // Open Source Software; you can modify and/or share it under the terms of
// // the WPILib BSD license file in the root directory of this project.

// package frc.robot.commands.Climber;

// import edu.wpi.first.wpilibj2.command.Command;
// import frc.robot.subsystems.ClimberSubsystem;

// public class ClimberDescend2Command extends Command {
//   /** Creates a new ClimberClimb1Command. */
//   ClimberSubsystem climber;

//   public ClimberDescend2Command(ClimberSubsystem climber) {
//     // Use addRequirements() here to declare subsystem dependencies.
//     this.climber = climber;
//   }

//   // Called when the command is initially scheduled.
//   @Override
//   public void initialize() {}

//   // Called every time the scheduler runs while the command is scheduled.
//   @Override
//   public void execute() {
//     climber.descend2(0.1);
//   }

//   // Called once the command ends or is interrupted.
//   @Override
//   public void end(boolean interrupted) {
//     climber.climberOff();
//   }

//   // Returns true when the command should end.
//   @Override
//   public boolean isFinished() {
//     return false;
//   }
// }
