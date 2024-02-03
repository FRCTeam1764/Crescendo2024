// // Copyright (c) FIRST and other WPILib contributors.
// // Open Source Software; you can modify and/or share it under the terms of
// // the WPILib BSD license file in the root directory of this project.

// package frc.robot.commands;

// import edu.wpi.first.math.controller.PIDController;
// import edu.wpi.first.math.geometry.Translation2d;
// import edu.wpi.first.wpilibj2.command.CommandBase;
// import frc.robot.constants.SwerveConstants;
// import frc.robot.subsystems.LimelightSubsystem;
// import frc.robot.subsystems.SwerveSubsystem;

// public class LockOnAprilTag extends CommandBase {
//   /** Creates a new AllignOnGamePiece. */
   
//   private LimelightSubsystem m_LimeLight;
//   private SwerveSubsystem m_Drivetrain;

//   private PIDController thetaController = new PIDController(SwerveConstants.Auton.angleAutoPID.p, SwerveConstants.Auton.angleAutoPID.i, SwerveConstants.Auton.angleAutoPID.d);
//   public LockOnAprilTag(SwerveSubsystem drivetrain, LimelightSubsystem limelight, String pipeline) {
//     addRequirements(drivetrain);
//     m_Drivetrain = drivetrain;
//     this.m_LimeLight = limelight;
//   }

//   // Called when the command is initially scheduled.
//   @Override
//   public void initialize() {
//     thetaController.reset();
//     thetaController.setTolerance(Math.toRadians(3)); //fix later?
//   }

//   // Called every time the scheduler runs while the command is scheduled.
//   @Override
//   public void execute() {
//     double thetaOutput = 0;
//     double xOutput = 0;
//     double yOutput = 0;
// 		if (m_LimeLight.hasTarget()){
// 			double vertical_angle = m_LimeLight.getVerticalAngleOfErrorDegrees();
// 			double horizontal_amgle = -m_LimeLight.getHorizontalAngleOfErrorDegrees() ;
// 			double setpoint = Math.toRadians(horizontal_amgle)+ m_Drivetrain.getPose().getRotation().getRadians();
//       thetaController.setSetpoint(setpoint);

// 			if (!thetaController.atSetpoint() ){
// 				thetaOutput = thetaController.calculate(m_Drivetrain.getPose().getRotation().getRadians(), setpoint);
// 			} else {

//       }
// 		} else {
// 			System.out.println("NO TARGET");
// 		}
//     m_Drivetrain.drive(new Translation2d(xOutput,yOutput),thetaOutput,true);
//   }

//   // Called once the command ends or is interrupted.
//   @Override
//   public void end(boolean interrupted) {
//     m_Drivetrain.drive(new Translation2d(0,0),0,true);
//   }

//   // Returns true when the command should end.
//   @Override
//   public boolean isFinished() {
//     return false;
//   }
// }