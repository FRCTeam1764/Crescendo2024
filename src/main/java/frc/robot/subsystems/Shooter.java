// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
// :)))))))
package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.networktables.NetworkTableType;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.constants.Constants;
import frc.robot.libraries.internal.LazyTalonFX;

public class Shooter extends SubsystemBase {
  /** Creates a new Shooter. */
  public LazyTalonFX flyWheel1;
  public LazyTalonFX flyWheel2;
  public LazyTalonFX holderRoller;
  private DigitalInput breakBeamHolder;
  private VelocityVoltage controlVoltage;
  private NeutralOut brake;

  public Shooter() {

    flyWheel1 = new LazyTalonFX(Constants.FLYWHEEL_MOTOR1.id, Constants.FLYWHEEL_MOTOR1.busName);
    flyWheel2 = new LazyTalonFX(Constants.FLYWHEEL_MOTOR2.id, Constants.FLYWHEEL_MOTOR2.busName);
    holderRoller = new LazyTalonFX(Constants.HOLDER_MOTOR.id, Constants.HOLDER_MOTOR.busName);
    SmartDashboard.putBoolean("RollerBreakBeam", false);

    setupMotors();

    flyWheel1.setInverted(true);
    flyWheel1.setNeutralMode(NeutralModeValue.Coast);
    flyWheel2.setNeutralMode(NeutralModeValue.Coast);

    holderRoller.setInverted(true);
    holderRoller.setNeutralMode(NeutralModeValue.Coast);

    breakBeamHolder = new DigitalInput(Constants.HOLDER_BREAK_BEAM);

    controlVoltage = new VelocityVoltage(0);
brake = new NeutralOut();
  }

  public void setupMotors() {
    TalonFXConfiguration config1 = new TalonFXConfiguration();



    TalonFXConfiguration config2 = new TalonFXConfiguration();

    config1.Slot0.kP = .4; //prev .65
    config1.Slot0.kD = .005;

    config2.Slot0.kP = .4;
    config2.Slot0.kD = .005;
    
    
  

    TalonFXConfiguration configRoller = new TalonFXConfiguration();

    flyWheel1.getConfigurator().apply(config1);
    flyWheel2.getConfigurator().apply(config2);

    holderRoller.getConfigurator().apply(configRoller);
  }

  public void shooterOn() {
    flyWheel1.set(1);
    flyWheel2.set(1);
  }

  public void shooterAmp() {
    flyWheel1.set(0.5);
    flyWheel1.set(0.5);
  }

  public void shooterOff() {
    flyWheel1.set(0);
    flyWheel2.set(0);
  }

  public void shooterPIDOff(){
flyWheel1.setControl(brake);
flyWheel2.setControl(brake);
  }
  public void shooterPID(double speed ){
flyWheel1.setControl(controlVoltage.withFeedForward(0.7).withSlot(0).withVelocity(speed).withLimitReverseMotion(true));
flyWheel2.setControl(controlVoltage.withFeedForward(0.7).withSlot(0).withVelocity(speed).withLimitReverseMotion(true));

  }

  public void roller(double speed) {

    int negative = 1;
    if (speed > 0) {
      negative = 1;
    } else {
      negative = -1;
    }
    double newspeed = getPercentFromBattery(speed);

    //  if (!breakBeamHolder.get() ) {
    //   newspeed =0; 
    //  }
    holderRoller.set(newspeed);
  }

  public void rollerOff() {
    holderRoller.set(0);
  }

  public boolean RollerBreakBeamBroken() {
    return !breakBeamHolder.get();
  }


  public double getPercentFromBattery(double speed){
        return speed * 12 / RobotController.getBatteryVoltage();
}
  @Override
  public void periodic() {
    SmartDashboard.putBoolean("RollerBreakBeam", RollerBreakBeamBroken());

    SmartDashboard.putNumber("flywheel1-velo", flyWheel1.getVelocity().getValueAsDouble());
     SmartDashboard.putNumber("flywheel2-velo", flyWheel2.getVelocity().getValueAsDouble());

    // This method will be called once per scheduler run
  }
}
