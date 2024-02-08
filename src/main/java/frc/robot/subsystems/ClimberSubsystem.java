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
  public PIDController pidController;
  public DigitalInput limitSwitch;
  public ArmFeedforward feedforward; 
  public DigitalInput limitSwitch2;
  double ClimberOffset;
  int MaxClimberEncoder =-120000;
  int negative;
//30.5 inches

  public ClimberSubsystem() {

    // 1 = left
    // 2 = right
    ClimberMotor1 = new LazyTalonFX(Constants.CLIMBER_MOTOR1.id, Constants.CLIMBER_MOTOR1.busName);
    ClimberMotor2 = new LazyTalonFX(Constants.CLIMBER_MOTOR2.id, Constants.CLIMBER_MOTOR1.busName);
    ClimberMotor2.follow(ClimberMotor1);
    limitSwitch = new DigitalInput(Constants.CLIMBER_SWITCH_LEFT);
    limitSwitch2 = new DigitalInput(Constants.CLIMBER_SWITCH_RIGHT);
    negative =1;
    pidController = new PIDController(0.00002, 0, 0);

    //feedforward = new ArmFeedforward(0.1, 0.1,0.1 );//needs characterization maybe do this?

  }
  public void ClimberOnRight(double desiredEncoderValue){


double variable = pidController.calculate(getEncoderValue(),desiredEncoderValue);

 variable = getSign(variable)*Math.min(7.2, Math.abs(variable));

      ClimberMotor2.set(ControlMode.PercentOutput,variable);  
    
  }

 public void ClimberOnLeft(double desiredEncoderValue){

    double variable = pidController.calculate(getEncoderValue(),desiredEncoderValue);


     variable = getSign(variable)*Math.min(7.2, Math.abs(variable));

      ClimberMotor1.set(ControlMode.PercentOutput,variable);  
    
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

  public int getSign(int num){
    if (num < 0){
      return -1;
  }
    return 1;
  }
  public int getSign(double num){
    if (num < 0){
      return -1;
  }
  return 1;
  }

  @Override
  public void periodic() {
    // 1 = left 2 = right
    SmartDashboard.putBoolean("LeftClimberSwitch", getLimitSwitch());
    SmartDashboard.putBoolean("RightClimberSwitch", getLimitSwitch2());

    if (getLimitSwitch()){
      zeroEncoder1();
    }
    if (getLimitSwitch2()){
      zeroEncoder2();
    }
    // This method will be called once per scheduler run
  }
}
