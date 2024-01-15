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


public class IntakeSubsystem extends SubsystemBase {
  /** Creates a new IntakeSubsystem. */
  private final CANSparkMax m_flexMotor = new CANSparkMax(Constants.WRIST_MOTOR.id, MotorType.kBrushless);
  private final CANSparkMax m_intakeMotor = new CANSparkMax(Constants.INTAKE_MOTOR.id, MotorType.kBrushless);
  // private final PIDController m_flexPIDController = new PIDController(1.1, 0, 0.05);
  private final SparkMaxAbsoluteEncoder m_angleEncoder = m_flexMotor.getAbsoluteEncoder(SparkMaxAbsoluteEncoder.Type.kDutyCycle);// new RevThroughBoreEncoder(Constants.WRIST_ANGLE_ENCODER);
  private PIDController m_flexPidController;
  private ArmFeedforward m_feedForward = new ArmFeedforward(0.11202, 0.11202,2.0024); 
  private ArmFeedforward down_Feedforward = new ArmFeedforward(0.15488,7.1406E+15 , 1.9548);
  private ArmFeedforward up_Feedforward = new ArmFeedforward(0.15245, 0.16345, 1.9801);
  private PivotySubsystem pivoty;
  private DigitalInput breakBeamIntake;
  
  public IntakeSubsystem() {
    super();
    m_intakeMotor.restoreFactoryDefaults();
    m_flexMotor.restoreFactoryDefaults();
    m_flexMotor.setInverted(false);
    // m_flexMotor.setInverted(true);
    m_flexMotor.setIdleMode(IdleMode.kBrake);
    m_angleEncoder.setPositionConversionFactor(360);
    m_angleEncoder.setZeroOffset(140);//150
    m_angleEncoder.setInverted(true);
    this.pivoty = pivoty;
    breakBeamIntake = new DigitalInput(Constants.INTAKE_BREAK_BEAM);
    
    Timer timer = new Timer();
  }
  double negative;
  public void wheelsIntake(double speed) {
        
    if(speed<0){
        negative=-1;
    } else{
        negative=1;
    }

  SmartDashboard.putNumber("intake speed", m_intakeMotor.get());
      if(!breakBeamIntake.get()) {
          m_intakeMotor.set(.04*negative);
      } else {
          m_intakeMotor.set(speed);
      }
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
  public void flexClosedLoop() {
    pid = new PIDController(12, 0, 0.001);
    
    double upfeedForward = m_feedForward.calculate(getAngleRadians(),0.1);
    
    setpoint = (((213*Math.PI)/180)-pivoty.getEncoderRadiansIntake());
    SmartDashboard.putNumber("setpoint", setpoint*180/Math.PI);
    pid.setSetpoint(setpoint);
    SmartDashboard.putNumber("getAngleRadians", getAngleRadians());
  
    pidValue = pid.calculate(getAngleRadians()); //calculate feed forward
      if(Math.abs(setpoint - getAngleRadians())< .0175){
        pidValue = 0;   
      }
    SmartDashboard.putNumber("intakePID", pidValue);
    SmartDashboard.putNumber("totalMotorSet", pidValue+upfeedForward);
    m_flexMotor.setVoltage(Math.min(pidValue+upfeedForward,6));
    }
    //234902788.15639588
    //160, 0.4 - p, 0.005-d, 0.8 velo
    public double getGroundRelativeWristPossitionRadians(){
        return  getAngleRadians()+pivoty.getEncoderRadiansIntake();
    }
    public double interpolate(double min, double max, double currentEncoder ){
        return (currentEncoder/140000)*(max-min)+min;
      }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    m_angleEncoder.setZeroOffset(140);
    flexClosedLoop();
  }
}
