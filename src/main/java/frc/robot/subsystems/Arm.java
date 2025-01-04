// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.CANcoder;
import com.revrobotics.spark.SparkLimitSwitch;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.ProfiledPIDSubsystem;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.ArmConstants.*;

public class Arm extends ProfiledPIDSubsystem {
  private SparkMax leftArmMotor = new SparkMax(ArmConstants.leftArmMotorID, MotorType.kBrushless);
  private SparkMax rightArmMotor = new SparkMax(ArmConstants.rightArmMotorID, MotorType.kBrushless);

  private SparkMaxConfig leftArmMotorConfig = new SparkMaxConfig();
  private SparkMaxConfig rightArmMotorConfig = new SparkMaxConfig();

  private CANcoder encoder = new CANcoder(ArmConstants.encoderID);
  private ArmFeedforward feedforward = new ArmFeedforward(FeedForwardValues.kS, FeedForwardValues.kG,
      FeedForwardValues.kV);

  private double ffOutput;
  private boolean podium = false;

  @SuppressWarnings("unused")
  private SparkLimitSwitch rightReverseLimitSwitch;

  /** Creates a new ProfPIDArm. */
  public Arm() {
    super(
        // The ProfiledPIDController used by the subsystem
        new ProfiledPIDController(
            PIDValues.p,
            PIDValues.i,
            PIDValues.d,
            // The motion profile constraints
            new TrapezoidProfile.Constraints(6, 5)));

    leftArmMotorConfig.idleMode(IdleMode.kBrake);
    rightArmMotorConfig.idleMode(IdleMode.kBrake);

    rightArmMotorConfig.inverted(true);
    leftArmMotorConfig.follow(ArmConstants.rightArmMotorID);

    rightArmMotor.configure(rightArmMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    leftArmMotor.configure(leftArmMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    
    // leftArmMotor.restoreFactoryDefaults();
    // rightArmMotor.restoreFactoryDefaults();

    // leftArmMotor.setIdleMode(IdleMode.kBrake);
    // rightArmMotor.setIdleMode(IdleMode.kBrake);

    // rightArmMotor.setInverted(true);
    // leftArmMotor.follow(rightArmMotor, true);

    rightReverseLimitSwitch = rightArmMotor.getReverseLimitSwitch(SparkLimitSwitch.Type.kNormallyClosed);

    setGoal(getEncoderRadians());
    enable();
  }

  @Override
  public void useOutput(double output, TrapezoidProfile.State setpoint) {
    ffOutput = -feedforward.calculate(setpoint.position, setpoint.velocity);
    output = -output;
    rightArmMotor.setVoltage(ffOutput + output);
    // SmartDashboard.putNumber("Fead Firword", ffOutput);
    // SmartDashboard.putNumber("Arm PID output", output);
    // SmartDashboard.putNumber("PID + FF", ffOutput + output);
  }

  public void setTarget(double target) {
    // ArmPositions.upper is lower than ArmPositions.lower
    setTarget(target, false);
  }

  public void setTarget(double target, boolean podium) {
    // ArmPositions.upper is lower than ArmPositions.lower
    this.podium = podium;
    this.setGoal(MathUtil.clamp(target, ArmPositions.lowerRad, ArmPositions.upperRad));
  }

  public void setTargetRotations(double target) {
    setTarget(target * 2 * Math.PI);
  }

  public void adjustTarget(double delta) {
    if (Math.abs(delta) > 0.01) {
      podium = false;
    }

    setTarget(this.getController().getGoal().position + delta, podium);
  }

  public void armPodium() {
    if (podium) {
      setTarget(ArmPositions.lowerRad);
    } else {
      setTarget(ArmPositions.podium, true);
    }
  }

  public void armUp() {
    setTarget(ArmPositions.upperRad);
  }

  public void armDown() {
    setTarget(ArmPositions.lowerRad);
  }

  public double getEncoder() {
    return encoder.getAbsolutePosition().getValueAsDouble();
  }

  public double getEncoderRadians() {
    return getEncoder() * 2 * Math.PI;
  }

  @Override
  public double getMeasurement() {
    // Return the process variable measurement here
    return getEncoderRadians();
  }

  @Override
  public void periodic() {
    super.periodic();

    SmartDashboard.putNumber("ArmGoal", this.getController().getGoal().position);
    SmartDashboard.putNumber("pos", getMeasurement());
    // SmartDashboard.putNumber("encoder", getEncoder());
  }
}
