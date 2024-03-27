// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.AutoCommands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.commands.testClimberRight;
import frc.robot.constants.SwerveConstantsYAGSL;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.SwerveSubsystem;

public class DriveAlign extends Command {
  /** Creates a new LockOnAprilTag. */
   
  private LimelightSubsystem LimeLight;
  private SwerveSubsystem Drivetrain;
  double time;
  Timer timer;

  private Joystick controller;
  private PIDController thetaController = new PIDController(SwerveConstantsYAGSL.Auton.angleAutoPID.kP, SwerveConstantsYAGSL.Auton.angleAutoPID.kI, SwerveConstantsYAGSL.Auton.angleAutoPID.kD);
  public DriveAlign(SwerveSubsystem drivetrain, LimelightSubsystem limelight, int pipeline) {
    addRequirements(drivetrain);
    this.Drivetrain = drivetrain;
    this.LimeLight = limelight;
        this.time = 3;
    timer = new Timer();
    limelight.setPipeline(pipeline);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    thetaController.reset();
    thetaController.setTolerance(Math.toRadians(1)); //fix later?
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double thetaOutput = 0;
    double xOutput =0.35;
    double yOutput = 0;
		if (LimeLight.hasTarget()){
			double horizontal_amgle = -LimeLight.getHorizontalAngleOfErrorDegrees() ;
			double setpoint = Math.toRadians(horizontal_amgle)+Drivetrain.getPose().getRotation().getRadians();
      thetaController.setSetpoint(setpoint);

			if (!thetaController.atSetpoint()){
				thetaOutput = thetaController.calculate(Drivetrain.getPose().getRotation().getRadians(), setpoint);
			}
      System.out.print(String.valueOf(thetaOutput));
		} 
    Drivetrain.drive(new Translation2d(-.35*Drivetrain.maximumSpeed,0),thetaOutput,false);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    Drivetrain.drive(new Translation2d(0,0),0,false);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
         if(timer.get()>time){
      return true;
    }
    return false;
  };
  
}