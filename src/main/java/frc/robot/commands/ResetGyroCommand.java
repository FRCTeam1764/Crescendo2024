// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

// package frc.robot.commands;

// import edu.wpi.first.wpilibj2.command.Command;
// import frc.robot.libraries.external.math.Rotation2;
// import frc.robot.subsystems.DrivetrainSubsystem;

// public class ResetGyroCommand extends Command {
//   /** Creates a new ResetGyroCommand. */
//   DrivetrainSubsystem drivetrainSubsystem;
//   public ResetGyroCommand(DrivetrainSubsystem drivetrainSubsystem) {
//     // Use addRequirements() here to declare subsystem dependencies.
//     this.drivetrainSubsystem = drivetrainSubsystem;
//   }

//   // Called when the command is initially scheduled.
//   @Override
//   public void initialize() {}

//   // Called every time the scheduler runs while the command is scheduled.
//   @Override
//   public void execute() {
//     drivetrainSubsystem.resetGyroAngle(Rotation2.ZERO);
//   }

//   // Called once the command ends or is interrupted.
//   @Override
//   public void end(boolean interrupted) {}

//   // Returns true when the command should end.
//   @Override
//   public boolean isFinished() {
//     return false;
//   }
// }
