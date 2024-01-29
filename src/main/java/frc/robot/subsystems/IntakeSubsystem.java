// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxAbsoluteEncoder;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Constants;
import frc.robot.state.IntakeState;


public class IntakeSubsystem extends SubsystemBase {
  /** Creates a new IntakeSubsystem. */
  private final CANSparkMax m_flexMotor1 = new CANSparkMax(Constants.WRIST_MOTOR1.id, MotorType.kBrushless);
  private final CANSparkMax m_flexMotor2 = new CANSparkMax(Constants.WRIST_MOTOR2.id, MotorType.kBrushless);
  private final CANSparkMax m_intakeMotor = new CANSparkMax(Constants.INTAKE_MOTOR.id, MotorType.kBrushless);
  // private final PIDController m_flexPIDController = new PIDController(1.1, 0, 0.05);
  private final SparkMaxAbsoluteEncoder m_angleEncoder1 = m_flexMotor1.getAbsoluteEncoder(SparkMaxAbsoluteEncoder.Type.kDutyCycle);// new RevThroughBoreEncoder(Constants.WRIST_ANGLE_ENCODER);
  private final SparkMaxAbsoluteEncoder m_angleEncoder2 = m_flexMotor2.getAbsoluteEncoder(SparkMaxAbsoluteEncoder.Type.kDutyCycle);
  private PIDController m_flexPidController;
  private ArmFeedforward m_feedForward = new ArmFeedforward(0.11202, 0.11202,2.0024); 
  // Need to change values
  // private ArmFeedforward down_Feedforward = new ArmFeedforward(0.15488,7.1406E+15 , 1.9548);
   private ArmFeedforward up_Feedforward = new ArmFeedforward(0.15245, 0.16345, 1.9801);
  private DigitalInput breakBeamIntake;
  private IntakeState intakeState;
  public IntakeSubsystem(IntakeState intakeState) {
    super();
    m_intakeMotor.restoreFactoryDefaults();
    m_flexMotor1.restoreFactoryDefaults();
    m_flexMotor2.restoreFactoryDefaults();
    m_flexMotor1.setInverted(false);
    m_flexMotor2.setInverted(false);
    // m_flexMotor.setInverted(true);
    m_flexMotor1.setIdleMode(IdleMode.kBrake);
    m_flexMotor2.setIdleMode(IdleMode.kBrake);
    m_angleEncoder1.setPositionConversionFactor(360);
    m_angleEncoder2.setPositionConversionFactor(360);
    // m_angleEncoder.setZeroOffset(140);//150
    m_angleEncoder1.setInverted(true);
    m_angleEncoder2.setInverted(true);
    breakBeamIntake = new DigitalInput(Constants.INTAKE_BREAK_BEAM);
    this.intakeState = intakeState;
    
  }

  double negative;
  public void wheelsIntake(double speed) {
        
  //  if(speed<0){
  //      negative=-1;
  //  } else{
  //      negative=1;
  //  }

  //SmartDashboard.putNumber("intake speed", m_intakeMotor.get());
    //  if(!breakBeamIntake.get()) {
      //    m_intakeMotor.set(.04*negative);
    //  } else {
      //    m_intakeMotor.set(speed);
    //  }
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
  public void flexClosedLoop(double desired) {
    pid = new PIDController(12, 0, 0.001);
    //:p
    double upfeedForward1 = m_feedForward.calculate(m_angleEncoder1.getPosition(),0.1);
    double upfeedForward2 = m_feedForward.calculate(m_angleEncoder2.getPosition(),0.1);
    
    setpoint = desired;
    SmartDashboard.putNumber("setpoint", setpoint);
    pid.setSetpoint(setpoint);
    SmartDashboard.putNumber("getPosition", m_angleEncoder1.getPosition());
    SmartDashboard.putNumber("getPosition", m_angleEncoder2.getPosition());
  
    pidValue = pid.calculate(m_angleEncoder1.getPosition()); //calculate feed forward
      if(Math.abs(setpoint - m_angleEncoder1.getPosition())< .0175){
        pidValue = 0;   
      }
    pidValue = pid.calculate(m_angleEncoder2.getPosition()); //calculate feed forward
      if(Math.abs(setpoint - m_angleEncoder2.getPosition())< .0175){
        pidValue = 0;   
      }
    SmartDashboard.putNumber("intakePID", pidValue);
    SmartDashboard.putNumber("totalMotorSet", pidValue+upfeedForward1);
    SmartDashboard.putNumber("totalMotorSet", pidValue+upfeedForward2);
    
    m_flexMotor1.setVoltage(Math.min(pidValue+upfeedForward1,6));
    m_flexMotor2.setVoltage(Math.min(pidValue+upfeedForward2,6));
    }
    //234902788.15639588
    //160, 0.4 - p, 0.005-d, 0.8 velo

    public double interpolate(double min, double max, double currentEncoder ){
        return (currentEncoder/140000)*(max-min)+min;
      }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
   // m_angleEncoder.setZeroOffset(140);
    flexClosedLoop(intakeState.getEncoderValue());
  }
}
