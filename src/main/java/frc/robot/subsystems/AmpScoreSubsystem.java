// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkAbsoluteEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Constants;
import frc.robot.state.IntakeState;

public class AmpScoreSubsystem extends SubsystemBase {
  /** Creates a new AmpScoreSubsystem. */
  private final CANSparkMax ampMotor = new CANSparkMax(Constants.AMP_MOTOR.id, MotorType.kBrushless);
  private SparkPIDController pidController;
  double desired =166;

  public AmpScoreSubsystem() {
    ampMotor.restoreFactoryDefaults();
    ampMotor.setIdleMode(IdleMode.kBrake);

    pidController = ampMotor.getPIDController();
    pidController.setP(0);
    pidController.setI(0);
    pidController.setD(0);
    
    pidController.setOutputRange(-.8, .8);

  }

  double setpoint;

  public void setPidPosition(double desired) {
    setpoint = desired;
    pidController.setReference(setpoint, ControlType.kPosition);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
