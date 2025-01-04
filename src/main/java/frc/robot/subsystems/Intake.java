// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;

public class Intake extends SubsystemBase {
  /** Creates a new Intake. */
  private CANSparkMax intakeMotor = new CANSparkMax(IntakeConstants.intakeMotorID, MotorType.kBrushless);
  private CANSparkMax kickupMotor = new CANSparkMax(IntakeConstants.kickupMotorID, MotorType.kBrushless);
  

  public Intake() {
    intakeMotor.restoreFactoryDefaults();
    kickupMotor.restoreFactoryDefaults();
    
    intakeMotor.setIdleMode(IdleMode.kCoast);
    kickupMotor.setIdleMode(IdleMode.kBrake);

    intakeMotor.setSmartCurrentLimit(IntakeConstants.currentLimit);
  }

  public void runIntake(double intakeMotorSpeed, double kickupMotorSpeed) {
    intakeMotor.set(intakeMotorSpeed);
    kickupMotor.set(kickupMotorSpeed);
  }

  public void stopIntake() {
    intakeMotor.stopMotor();
    kickupMotor.stopMotor();
  }

  // public boolean getIntakeOn() {
  //   return intakeMotor.get
  // }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
