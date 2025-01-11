// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;

public class Intake extends SubsystemBase {
  /** Creates a new Intake. */
  private SparkMax intakeMotor = new SparkMax(IntakeConstants.intakeMotorID, MotorType.kBrushless);
  private SparkMax kickupMotor = new SparkMax(IntakeConstants.kickupMotorID, MotorType.kBrushless);

  private SparkMaxConfig intakeMotorConfig = new SparkMaxConfig();
  private SparkMaxConfig kickupMotorConfig = new SparkMaxConfig();

  public Intake() {
    intakeMotorConfig.idleMode(IdleMode.kCoast);
    kickupMotorConfig.idleMode(IdleMode.kBrake);

    intakeMotorConfig.smartCurrentLimit(IntakeConstants.currentLimit);
    kickupMotorConfig.smartCurrentLimit(IntakeConstants.currentLimit);

    intakeMotor.configure(intakeMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    kickupMotor.configure(kickupMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    // intakeMotor.restoreFactoryDefaults();
    // kickupMotor.restoreFactoryDefaults();

    // intakeMotor.setIdleMode(IdleMode.kCoast);
    // kickupMotor.setIdleMode(IdleMode.kBrake);

    // intakeMotor.setSmartCurrentLimit(IntakeConstants.currentLimit);
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
  // return intakeMotor.get
  // }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
