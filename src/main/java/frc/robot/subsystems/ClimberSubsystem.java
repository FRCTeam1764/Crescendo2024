// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import org.ejml.equation.VariableMatrix;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatorCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Constants;
import frc.robot.libraries.internal.LazyTalonFX;

public class ClimberSubsystem extends SubsystemBase {
  /** Creates a new Climber. */

  public LazyTalonFX ClimberMotor1;
  public LazyTalonFX ClimberMotor2;
  public PIDController pidController1;
  public PIDController pidController2;
  public DigitalInput limitSwitch;
  public ArmFeedforward feedforward; 
  public DigitalInput limitSwitch2;
  double ClimberOffset;
  int MaxClimberEncoderLeft =-120000;
  int MaxClimberEncoderRight = 0;

  int negative;
//30.5 inches

  public ClimberSubsystem() {

    // 1 = right
    // 2 = left
    ClimberMotor1 = new LazyTalonFX(Constants.CLIMBER_MOTOR1.id, Constants.CLIMBER_MOTOR1.busName);
    ClimberMotor2 = new LazyTalonFX(Constants.CLIMBER_MOTOR2.id, Constants.CLIMBER_MOTOR1.busName);
    ClimberMotor1.configFactoryDefault();
    ClimberMotor1.setNeutralMode(NeutralMode.Brake);
        ClimberMotor2.setNeutralMode(NeutralMode.Brake);

    ClimberMotor2.configFactoryDefault();
   // ClimberMotor2.follow(ClimberMotor1);
    limitSwitch = new DigitalInput(Constants.CLIMBER_SWITCH_LEFT);
    limitSwitch2 = new DigitalInput(Constants.CLIMBER_SWITCH_RIGHT);
    negative =1;
    pidController1 = new PIDController(0.00003, 0, 0.000005);//right
    pidController2 = new PIDController(0.00003, 0, 0.000005);
  SmartDashboard.putNumber("setypoint",100);

    //feedforward = new ArmFeedforward(0.1, 0.1,0.1 );//needs characterization maybe do this?

  }
  public void ClimberOnLeft(double desiredEncoderValue){


double variable = pidController2.calculate(getEncoderValue2(),desiredEncoderValue);
SmartDashboard.putNumber("LeftClimb", variable);

 variable = getSign(variable)*Math.min(.9, Math.abs(variable));

      ClimberMotor2.set(variable);  
    
  }

 public void ClimberOnRight(double desiredEncoderValue){

    double variable = pidController1.calculate(getEncoderValue(),desiredEncoderValue);

  SmartDashboard.putNumber("RightClimba", variable);

     variable = getSign(variable)*Math.min(.9, Math.abs(variable)); //prev 7.2
      ClimberMotor1.set(variable);  
  SmartDashboard.putNumber("RightCurrent",     ClimberMotor1.getSupplyCurrent());
    
  }

  public void ClimberRightTest(double speed){
    ClimberMotor1.set(speed);
  }
    public void ClimberLefttTest(double speed){
    ClimberMotor2.set(speed);
  }
  public double getDesiredEncoder(){
    return getDesiredEncoder();
  }


  public void zeroEncoder1(){
    ClimberMotor1.getSensorCollection().setIntegratedSensorPosition(0.0,0);
  }
    public void zeroEncoder2(){
    ClimberMotor2.getSensorCollection().setIntegratedSensorPosition(0.0,0);
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
     return ClimberMotor1.getSelectedSensorPosition();
   }
  public double getEncoderValue2(){
     return ClimberMotor2.getSelectedSensorPosition();
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
//158482 = left
SmartDashboard.putNumber("leftEncoder", getEncoderValue2());
setypointy = SmartDashboard.getNumber("setypoint",100);
SmartDashboard.putNumber("somethin", setypointy);
    if (getLimitSwitch()){
      zeroEncoder2();
        }
    if (getLimitSwitch2()){
      zeroEncoder1();
    }
    ClimberOnLeft(-setypointy);
    ClimberOnRight(-setypointy);
    // This method will be called once per scheduler run
  }
}
