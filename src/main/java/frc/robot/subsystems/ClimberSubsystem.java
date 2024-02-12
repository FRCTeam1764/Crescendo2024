// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;


import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Constants;
import frc.robot.libraries.internal.LazyTalonFX;

public class ClimberSubsystem extends SubsystemBase {
  /** Creates a new Climber. */

  public TalonFX ClimberMotor1;
  public TalonFX ClimberMotor2;
  public PIDController pidController1;
  public PIDController pidController2;
  public DigitalInput limitSwitch;
  public DigitalInput limitSwitch2;
  private PositionVoltage setVoltage;

  int negative;

  public ClimberSubsystem() {
    
    // 1 = right
    // 2 = left
    //Quick Info = Max encoder pos is 175000/2048 (encoder to motor spins)
    // length is 26.75 inches, can be used to convert encoder to distance if ur feelin special 
    ClimberMotor1 = new LazyTalonFX(Constants.CLIMBER_MOTOR1.id, Constants.CLIMBER_MOTOR1.busName);
    ClimberMotor2 = new LazyTalonFX(Constants.CLIMBER_MOTOR2.id, Constants.CLIMBER_MOTOR1.busName);

    limitSwitch = new DigitalInput(Constants.CLIMBER_SWITCH_LEFT);
    limitSwitch2 = new DigitalInput(Constants.CLIMBER_SWITCH_RIGHT);
    negative =1;
    pidController1 = new PIDController(0.00003, 0, 0.000005);//right
    pidController2 = new PIDController(0.00003, 0, 0.000005);
    setVoltage = new PositionVoltage(0).withSlot(0); 


   SetUpClimberMotors(); 
  }

public void SetUpClimberMotors(){
TalonFXConfiguration config = new TalonFXConfiguration();

config.Slot0.kP = 0.00003;
config.Slot0.kD = 0.000005;

config.MotorOutput.NeutralMode  = NeutralModeValue.Brake;
config.MotorOutput.PeakForwardDutyCycle = 10;
config.MotorOutput.PeakReverseDutyCycle = -10; // can bump up to 12 or something



ClimberMotor1.getConfigurator().apply(config);
ClimberMotor2.getConfigurator().apply(config);
}

  public void ClimberOnLeft(double desiredEncoderValue){


    // double variable = pidController2.calculate(getEncoderValue2(),desiredEncoderValue);
    // SmartDashboard.putNumber("LeftClimb", variable);

    //   variable = getSign(variable)*Math.min(1, Math.abs(variable));

     ClimberMotor2.setControl(setVoltage.withPosition(desiredEncoderValue));  
  }

 public void ClimberOnRight(double desiredEncoderValue){
  //   double variable = pidController1.calculate(getEncoderValue(),desiredEncoderValue);
  // SmartDashboard.putNumber("RightClimba", variable);
    //  variable = getSign(variable)*Math.min(1, Math.abs(variable)); //prev 7.2
      ClimberMotor1.setControl(setVoltage.withPosition(desiredEncoderValue));      
  }

  public void ClimberRightTest(double speed){
    ClimberMotor1.set(speed);
  }
    public void ClimberLefttTest(double speed){
    ClimberMotor2.set(speed);
    }
  public void zeroEncoder1(){
    ClimberMotor1.setPosition(0);
  }
    public void zeroEncoder2(){
    ClimberMotor2.setPosition(0);
  }

  public boolean getLimitSwitch(){
  return !limitSwitch.get();
  }
  public boolean getLimitSwitch2(){
    return !limitSwitch2.get();
  }
  public void ClimberOff(){
     ClimberMotor1.set( 0);
     ClimberMotor2.set(0);
  }
  public double getEncoderValue(){
     return ClimberMotor1.getPosition().getValueAsDouble();
   }

  public double getEncoderValue2(){
     return ClimberMotor2.getPosition().getValueAsDouble();
   }
  public int getSign(double num){
    if (num < 0){
      return -1;
  }
  return 1;
  }

  double setypointy;
  @Override
  public void periodic() {
    // 1 = left 2 = right
    SmartDashboard.putBoolean("LeftClimberSwitch", getLimitSwitch());
    SmartDashboard.putBoolean("RightClimberSwitch", getLimitSwitch2());
SmartDashboard.putNumber("rightEncoder", getEncoderValue());
//-175000=right
SmartDashboard.putNumber("leftEncoder", getEncoderValue2());
//setypointy = SmartDashboard.getNumber("setypoint",100);
//SmartDashboard.putNumber("somethin", setypointy);
    if (getLimitSwitch()){
      zeroEncoder2();
        }
    if (getLimitSwitch2()){
      zeroEncoder1();
    }
//    ClimberOnLeft(-setypointy);
  //  ClimberOnRight(-setypointy);
    // This method will be called once per scheduler run
  }
}
