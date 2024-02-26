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

  ArmFeedforward armfeed = new ArmFeedforward(0,0.3,0);

  private final SparkAbsoluteEncoder m_angleEncoder = m_flexMotor
      .getAbsoluteEncoder(SparkAbsoluteEncoder.Type.kDutyCycle);// new
                                                                // RevThroughBoreEncoder(Constants.WRIST_ANGLE_ENCODER);
  private DigitalInput breakBeamIntake;
  private DigitalInput breakBeamIntakeOut;
  private DigitalInput breakBeamIntakeMid;

  public IntakeSubsystem(IntakeState intakeState) {

    m_intakeMotor.restoreFactoryDefaults();
    m_flexMotor.restoreFactoryDefaults();
    m_flexMotor.setInverted(true);

    m_flexMotor.setIdleMode(IdleMode.kBrake);
  
    pidController = m_flexMotor.getPIDController();
    pidController.setP(.007); 
    pidController.setD(0.5);
    
    pidController.setFeedbackDevice(m_angleEncoder);
    pidController.setOutputRange(-.8, .8);
  
    // pidController.setSmartMotionAllowedClosedLoopError(0, 0);

    this.intakeState = intakeState;

    m_angleEncoder.setPositionConversionFactor(360);
    m_angleEncoder.setZeroOffset(1);

    m_intakeMotor.setInverted(true);
    m_intakeMotor.setSecondaryCurrentLimit(50);
    m_angleEncoder.setInverted(false);

    breakBeamIntake = new DigitalInput(Constants.INTAKE_BREAK_BEAM_INNER);
    breakBeamIntakeOut = new DigitalInput(Constants.INTAKE_BREAK_BEAM_FEED);
    breakBeamIntakeMid = new DigitalInput(Constants.INTAKE_BREAK_BEAM_MIDDLE);
    //SmartDashboard.putNumber("SetPointSet", setpoint);


  }

  double negative;

  public void wheelsIntake(double speed) {

    if (speed < 0) {
      negative = -1;
    } else {
      negative = 1;
    }

     if (!breakBeamIntake.get() && negative ==1) {
       m_intakeMotor.set(0);
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
    setpoint = desired;

   //  setpoint = SmartDashboard.getNumber("SetPointSet",300);
   // SmartDashboard.putNumber("da point", setpoint);
    // pid.setSetpoint(setpoint);
   // SmartDashboard.putNumber("getPosition", m_angleEncoder.getPosition());


    // SmartDashboard.putNumber("intakePID", pidValue);
    // SmartDashboard.putNumber("totalMotorSet", pidValue);
      pidController.setReference(setpoint, ControlType.kPosition);
  }

  public double getEncoderPos() {
    return m_angleEncoder.getPosition();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    // m_angleEncoder.setZeroOffset(140);
    SmartDashboard.putBoolean("IntakeBreakbeam", getIntakeBreakbeam());
    SmartDashboard.putBoolean("Intake2", !breakBeamIntakeOut.get());
    SmartDashboard.putBoolean("Intake3",!breakBeamIntakeMid.get());
    SmartDashboard.putNumber("intakeCurre t", m_intakeMotor.getOutputCurrent());
    flexClosedLoop(intakeState.getEncoderValue());
  }
}
