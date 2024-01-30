// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Constants;
import frc.robot.libraries.internal.LazyTalonFX;
import edu.wpi.first.math.controller.PIDController;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.LimitSwitchNormal;
import com.ctre.phoenix.motorcontrol.LimitSwitchSource;
import com.ctre.phoenix.motorcontrol.NeutralMode;

import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;

import edu.wpi.first.wpilibj.PneumaticsModuleType;

/** Add your docs here*/
public class ClimberUnUsed extends SubsystemBase {
  private LazyTalonFX climberMasterMotor;
  private LazyTalonFX climberFollowerMotor;
  private DoubleSolenoid climberSolenoid;
  private PIDController pidController; 
  private int offset;
  public DigitalInput rightLimitSwitch;
  public DigitalInput leftLimitSwitch;

  public ClimberUnUsed(){
    this.climberMasterMotor = new LazyTalonFX(Constants.CLIMBER_MOTOR1.id,Constants.CLIMBER_MOTOR1.busName);
    this.climberFollowerMotor = new LazyTalonFX(Constants.CLIMBER_MOTOR2.id,Constants.CLIMBER_MOTOR2.busName);
		this.climberMasterMotor.configFactoryDefault();
		this.climberFollowerMotor.configFactoryDefault();
    this.climberMasterMotor.setInverted(true);
    this.climberMasterMotor.setNeutralMode(NeutralMode.Brake);
    this.climberFollowerMotor.setNeutralMode(NeutralMode.Brake);
    this.climberFollowerMotor.follow(climberMasterMotor);

    this.pidController = new PIDController(.0001, 0 , 0);
    this.rightLimitSwitch = new DigitalInput(Constants.CLIMBER_LIMITSWITCH_RIGHT);
    this.leftLimitSwitch = new DigitalInput(Constants.CLIMBER_LIMITSWITCH_LEFT);
    this.climberFollowerMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_2_Feedback0, 171);
  }




  public void climberOn(double climberSpeed) {
    if(!rightLimitSwitch.get() || !leftLimitSwitch.get()){
      double newClimberSpeed = climberSpeed < 0 ? 0 : climberSpeed;
      climberMasterMotor.set(ControlMode.PercentOutput, newClimberSpeed);
    }
    else {
      climberMasterMotor.set(ControlMode.PercentOutput, climberSpeed);
    }
  }
  public void climberOff() {
    climberMasterMotor.set(ControlMode.PercentOutput, 0);
  }

  public void climb(){
    double calculation = pidController.calculate(getPosition());
    System.out.println(getPosition());
    double setpoint = pidController.getSetpoint();
    double signal = setpoint != 90000 ? calculation : calculation > 0.45 ? 0.45 : calculation;
    climberMasterMotor.set(signal);
  }

  public int getMasterEncoder(){
    int selSenPos = (int) climberMasterMotor.getSelectedSensorPosition(0);
    return selSenPos;
  }

  public void resetFalcon(){
offset = getMasterEncoder();
  }

  public boolean checkLimitSwitches(){
    if(!rightLimitSwitch.get() || !leftLimitSwitch.get()){
      return true;
    }
    else
    {
      return false;
    }
  }

  public void setPosition(int position) {
    pidController.setSetpoint(position);
  }

  public int getPosition() {
    return getMasterEncoder() - offset;
  }
}
