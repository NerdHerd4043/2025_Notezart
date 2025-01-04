// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.CANcoder;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkLimitSwitch;
import com.revrobotics.CANSparkBase.IdleMode;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.ProfiledPIDSubsystem;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.ArmConstants.*;

public class Arm extends ProfiledPIDSubsystem {
  private CANSparkMax leftArmMotor = new CANSparkMax(ArmConstants.leftArmMotorID, MotorType.kBrushless);
  private CANSparkMax rightArmMotor = new CANSparkMax(ArmConstants.rightArmMotorID, MotorType.kBrushless);
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

    leftArmMotor.restoreFactoryDefaults();
    rightArmMotor.restoreFactoryDefaults();

    leftArmMotor.setIdleMode(IdleMode.kBrake);
    rightArmMotor.setIdleMode(IdleMode.kBrake);

    rightArmMotor.setInverted(true);
    leftArmMotor.follow(rightArmMotor, true);

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
