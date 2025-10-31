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
import com.revrobotics.spark.config.LimitSwitchConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.ArmConstants.ArmPositions;
import frc.robot.Constants.ArmConstants.FeedForwardValues;
import frc.robot.Constants.ArmConstants.PIDValues;

@Logged
public class Arm extends SubsystemBase {
  private SparkMax leftArmMotor = new SparkMax(ArmConstants.leftArmMotorID,
      MotorType.kBrushless);
  private SparkMax rightArmMotor = new SparkMax(ArmConstants.rightArmMotorID,
      MotorType.kBrushless);

  private CANcoder encoder = new CANcoder(ArmConstants.encoderID);
  private ArmFeedforward feedforward = new ArmFeedforward(FeedForwardValues.kS,
      FeedForwardValues.kG,
      FeedForwardValues.kV);

  private double ffOutput;
  private boolean podium = false;

  // sendablechoosers

  private SendableChooser<Double> PChooser = new SendableChooser<>();
  private SendableChooser<Double> IChooser = new SendableChooser<>();
  private SendableChooser<Double> DChooser = new SendableChooser<>();

  @SuppressWarnings("unused")
  private SparkLimitSwitch rightReverseLimitSwitch;

  private ProfiledPIDController pidController = new ProfiledPIDController(
      PIDValues.p,
      PIDValues.i,
      PIDValues.d,
      new TrapezoidProfile.Constraints(6, 5));

  // /** Creates a new ProfPIDArm. */
  public Arm() {
    SparkMaxConfig leftArmMotorConfig = new SparkMaxConfig();
    SparkMaxConfig rightArmMotorConfig = new SparkMaxConfig();

    leftArmMotorConfig.idleMode(IdleMode.kBrake);
    rightArmMotorConfig.idleMode(IdleMode.kBrake);

    // Declare PID sendables for P
    this.PChooser.addOption("2", 2.0);
    this.PChooser.addOption("3", 3.0);
    this.PChooser.setDefaultOption("4", 4.0);
    this.PChooser.addOption("5", 5.0);
    this.PChooser.addOption("6", 6.0);

    // Declare PID sendables for I
    this.IChooser.setDefaultOption("0", 0.0);
    this.IChooser.addOption("0.2", 0.2);
    this.IChooser.addOption("0.4", 0.4);
    this.IChooser.addOption("0.6", 0.6);
    this.IChooser.addOption("0.8", 0.8);

    // Declare PID sendables for D
    this.DChooser.addOption("1.0", 1.0);
    this.DChooser.addOption("1.3", 1.3);
    this.DChooser.setDefaultOption("1.7", 1.7);
    this.DChooser.addOption("1.9", 1.9);
    this.DChooser.addOption("2.1", 2.1);

    // Push to dashboard
    SmartDashboard.putData(this.PChooser);
    SmartDashboard.putData(this.IChooser);
    SmartDashboard.putData(this.DChooser);

    // update variables for selected chooser
    double doubleP = PChooser.getSelected();
    double doubleI = IChooser.getSelected();
    double doubleD = DChooser.getSelected();

    // update pidcontroller every tick from variables
    pidController.setP(doubleP);
    pidController.setI(doubleI);
    pidController.setD(doubleD);

    // current limit
    leftArmMotorConfig.smartCurrentLimit(ArmConstants.motorCurrentLimit);
    rightArmMotorConfig.smartCurrentLimit(ArmConstants.motorCurrentLimit);

    // set follow
    leftArmMotorConfig.follow(ArmConstants.rightArmMotorID, true);
    // leftArmMotorConfig.inverted(true);
    // rightArmMotorConfig.inverted(false);

    rightArmMotorConfig.limitSwitch.reverseLimitSwitchType(LimitSwitchConfig.Type.kNormallyClosed);

    leftArmMotor.configure(leftArmMotorConfig, ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);
    rightArmMotor.configure(rightArmMotorConfig, ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);

    this.rightReverseLimitSwitch = this.rightArmMotor.getReverseLimitSwitch();

    this.pidController.setGoal(getEncoderRadians());
  }

  private void useOutput(double output, TrapezoidProfile.State setpoint) {
    ffOutput = -feedforward.calculate(setpoint.position, setpoint.velocity);
    output = -output;
    rightArmMotor.setVoltage(ffOutput + output);
  }

  public void setTarget(double target) {
    // ArmPositions.upper is lower than ArmPositions.lower
    setTarget(target, false);
  }

  public void setTarget(double target, boolean podium) {
    // ArmPositions.upper is lower than ArmPositions.lower
    this.podium = podium;
    this.pidController.setGoal(MathUtil.clamp(target, ArmPositions.lowerRad,
        ArmPositions.upperRad));
  }

  public void setTargetRotations(double target) {
    setTarget(target * 2 * Math.PI);
  }

  public void adjustTarget(double delta) {
    if (Math.abs(delta) > 0.01) {
      podium = false;
    }

    setTarget(this.pidController.getGoal().position + delta, podium);
  }

  public void armPodium() {
    if (podium) {
      setTarget(ArmPositions.upperRad);
    } else {
      setTarget(ArmPositions.podium, true);
    }
  }

  public double getPChooser() {
    return this.PChooser.getSelected();
  }

  public double getIChooser() {
    return this.IChooser.getSelected();
  }

  public double getDChooser() {
    return this.DChooser.getSelected();
  }

  public void armUp() {
    setTarget(ArmPositions.lowerRad);
  }

  public void armDown() {
    setTarget(ArmPositions.upperRad);
  }

  public double getEncoder() {
    return encoder.getAbsolutePosition().getValueAsDouble();
  }

  public double getEncoderRadians() {
    return getEncoder() * 2 * Math.PI;
  }

  public double getMeasurement() {
    // Return the process variable measurement here
    return getEncoderRadians();
  }

  public Command runArmDown() {
    return this.runOnce(() -> this.armDown());
  }

  @Override
  public void periodic() {
    useOutput(pidController.calculate(getMeasurement()),
        pidController.getSetpoint());

    SmartDashboard.putNumber("ArmGoal", this.pidController.getGoal().position);
    SmartDashboard.putNumber("pos", getMeasurement());
    SmartDashboard.putNumber("encoder", getEncoder());
  }
}
