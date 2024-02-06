// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Constants;


public class IntakeSubsystem extends SubsystemBase {
  /** Creates a new IntakeSubsystem. */
  public CANSparkMax intakeMotor;
  public DigitalInput intakeBreakbeam;
  
  public IntakeSubsystem() {
    intakeMotor = new CANSparkMax(Constants.INTAKE_MOTOR.id, MotorType.kBrushless);
    intakeBreakbeam = new DigitalInput(Constants.INTAKE_BREAK_BEAM);
    }

  public void intakeTakeRing() {
intakeMotor.set(-0.4);
    // if (!breakBeam.get()) {
    //   intakeMotor.set(.5); // edit speed?
    // } else {
    //   intakeMotor.set(0.1);
    // }
  }

  public void intakeOutRing() {
intakeMotor.set(0.4);
    // if (!breakBeam.get()) {
    //   intakeMotor.set(.5); // edit speed?
    // } else {
    //   intakeMotor.set(0.1);
    // }
  }
  // wrist stuff?!? I have no clue :D
  public void intakeOff() {
    intakeMotor.set(0);
  }

  @Override
  public void periodic() {
    SmartDashboard.putBoolean("IntakeBreakbem", intakeBreakbeam.get());
    // This method will be called once per scheduler run
  }
}
