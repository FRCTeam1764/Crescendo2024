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
//168 up
//
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Constants;
import frc.robot.state.IntakeState;

public class IntakeSubsystem extends SubsystemBase {
  /** Creates a new IntakeSubsystem. */
  private final CANSparkMax m_flexMotor = new CANSparkMax(Constants.WRIST_MOTOR1.id, MotorType.kBrushless);
  private final IntakeState intakeState;
  private final CANSparkMax m_intakeMotor = new CANSparkMax(Constants.INTAKE_MOTOR.id, MotorType.kBrushless);
  private SparkPIDController pidController;
  // private final PIDController m_flexPIDController = new PIDController(1.1, 0,
  // 0.05);
  private final SparkAbsoluteEncoder m_angleEncoder = m_flexMotor
      .getAbsoluteEncoder(SparkAbsoluteEncoder.Type.kDutyCycle);// new
                                                                // RevThroughBoreEncoder(Constants.WRIST_ANGLE_ENCODER);
  // private PIDController m_flexPidController;
  private DigitalInput breakBeamIntake;

  public IntakeSubsystem(IntakeState intakeState) {

    m_intakeMotor.restoreFactoryDefaults();
    m_flexMotor.restoreFactoryDefaults();
    m_flexMotor.setInverted(false);

    m_flexMotor.setIdleMode(IdleMode.kBrake);
    pidController = m_flexMotor.getPIDController();
    pidController.setP(.025); // dropped in half, prev .05
    pidController.setD(0.00001);
    pidController.setFeedbackDevice(m_angleEncoder);
    pidController.setOutputRange(-1, 1);
    
    // pidController.setSmartMotionAllowedClosedLoopError(0, 0);

    this.intakeState = intakeState;

    m_angleEncoder.setPositionConversionFactor(360);
    m_angleEncoder.setZeroOffset(1);

    m_intakeMotor.setInverted(true);
    m_angleEncoder.setInverted(false);
    breakBeamIntake = new DigitalInput(Constants.INTAKE_BREAK_BEAM);
    SmartDashboard.putNumber("SetPointSet", setpoint);
    SmartDashboard.putNumber("kP", 0);

    SmartDashboard.putNumber("kD", 0);

  }

  double negative;

  public void wheelsIntake(double speed) {

    if (speed < 0) {
      negative = -1;
    } else {
      negative = 1;
    }

    SmartDashboard.putNumber("intake speed", m_intakeMotor.get());
    if (!breakBeamIntake.get()) {
      m_intakeMotor.set(speed);
    } else {
      m_intakeMotor.set(speed);
    }
  }

  public boolean getIntakeBreakbeam() {
    return !breakBeamIntake.get();
  }

  public void run(double speed) {
    m_intakeMotor.set(speed);
  }

  public void stop() {
    m_intakeMotor.set(0);
  }

  private PIDController pid;
  double pidValue;
  double setpoint;
  double kP;
  double ki;
  double kd;

  public void flexClosedLoop(double desired) {

    // pid = new PIDController(SmartDashboard.getNumber("kP", 0), 0.0,
    // SmartDashboard.getNumber("kD", 0));
    // pid = new PIDController(0.05, 0, 0.00001); //previouslt .05
    // double upfeedForward =
    // m_feedForward.calculate(m_angleEncoder.getPosition(),0.1);
    setpoint = desired;

    // setpoint = SmartDashboard.getNumber("SetPointSet",300);
    SmartDashboard.putNumber("da point", setpoint);
    // pid.setSetpoint(setpoint);
    SmartDashboard.putNumber("getPosition", m_angleEncoder.getPosition());

    // pidValue = pid.calculate(m_angleEncoder.getPosition()); //calculate feed
    // forward
    if (Math.abs(setpoint - m_angleEncoder.getPosition()) < .0175) {
      pidValue = 0;
    }
    SmartDashboard.putNumber("intakePID", pidValue);
    SmartDashboard.putNumber("totalMotorSet", pidValue);
    // m_flexMotor.setVoltage(Math.min(pidValue,4));// remove 4 now previously 6
    // pidController.setReference(setpoint, ControlType.kPosition);
  }

  public double getEncoderPos() {
    return m_angleEncoder.getPosition();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    // m_angleEncoder.setZeroOffset(140);
    SmartDashboard.putBoolean("IntakeBreakbeam", getIntakeBreakbeam());
    // flexClosedLoop(intakeState.getEncoderValue());
  }
}
