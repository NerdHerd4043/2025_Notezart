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
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
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

  // cchooser
  private SendableChooser<Double> pChooser = new SendableChooser<>();
  private SendableChooser<Double> iChooser = new SendableChooser<>();
  private SendableChooser<Double> dChooser = new SendableChooser<>();
  private SendableChooser<Double> gChooser = new SendableChooser<>();

  // set name

  private double p = 0.0;
  private double i = 0.0;
  private double d = 0.0;
  private double g = 0.0;

  private CANcoder encoder = new CANcoder(ArmConstants.encoderID);
  private ArmFeedforward feedforward = new ArmFeedforward(FeedForwardValues.kS,
      this.g,
      FeedForwardValues.kV);

  private double ffOutput;
  private boolean podium = false;

  @SuppressWarnings("unused")
  private SparkLimitSwitch rightReverseLimitSwitch;

  private ProfiledPIDController pidController = new ProfiledPIDController(
      this.p,
      this.i,
      this.d,
      new TrapezoidProfile.Constraints(6, 5));

  // /** Creates a new ProfPIDArm. */
  public Arm() {
    SparkMaxConfig leftArmMotorConfig = new SparkMaxConfig();
    SparkMaxConfig rightArmMotorConfig = new SparkMaxConfig();

    // update pid variable thing
    // pCHooser

    this.pChooser.setDefaultOption("1.0", 1.0);
    this.pChooser.addOption("0", 0.0);
    this.pChooser.addOption("0.5", 0.5);
    this.pChooser.addOption("1.0", 1.0);
    this.pChooser.addOption("1.5", 1.5);
    this.pChooser.addOption("2.0", 2.0);
    this.pChooser.addOption("2.5", 2.5);
    this.pChooser.addOption("3", 3.0);
    this.pChooser.addOption("3.5", 3.5);
    // iCHooser
    this.iChooser.setDefaultOption("0 dude.0", 0.0);
    this.iChooser.addOption("0.0", 0.0);
    this.iChooser.addOption("0.5", 0.5);
    this.iChooser.addOption("1.0", 1.0);
    this.iChooser.addOption("1.5", 1.5);
    this.iChooser.addOption("2.0", 2.0);
    this.iChooser.addOption("2.5", 2.5);
    this.iChooser.addOption("3.0", 3.0);
    this.iChooser.addOption("3.5", 3.5);
    // dCHooser
    this.dChooser.setDefaultOption("zero", 0.0);
    this.dChooser.addOption("0.0", 0.0);
    this.dChooser.addOption("0.5", 0.5);
    this.dChooser.addOption("1.0", 1.0);
    this.dChooser.addOption("1.5", 1.5);
    this.dChooser.addOption("2", 2.0);
    this.dChooser.addOption("2.5", 2.5);
    this.dChooser.addOption("3", 3.0);
    this.dChooser.addOption("3.5", 3.5);
    // dCHooser
    this.gChooser.setDefaultOption("1", 1.0);
    this.gChooser.addOption("0", 0.0);
    this.gChooser.addOption("0.5", 0.5);
    this.gChooser.addOption("1", 1.0);
    this.gChooser.addOption("1.5", 1.5);
    this.gChooser.addOption("2", 2.0);
    this.gChooser.addOption("2.5", 2.5);
    this.gChooser.addOption("3", 3.0);
    this.gChooser.addOption("3.5", 3.5);

    // tabber
    ShuffleboardTab tab = Shuffleboard.getTab("PID-chooser");

    tab.add("P-Chooser", pChooser)
        .withWidget(BuiltInWidgets.kComboBoxChooser)
        .withPosition(0, 0)
        .withSize(2, 1);

    tab.add("I-Chooser", iChooser)
        .withWidget(BuiltInWidgets.kComboBoxChooser)
        .withPosition(0, 0)
        .withSize(2, 1);

    tab.add("D-Chooser", dChooser)
        .withWidget(BuiltInWidgets.kComboBoxChooser)
        .withPosition(0, 0)
        .withSize(2, 1);

    tab.add("G-Chooser", gChooser)
        .withWidget(BuiltInWidgets.kComboBoxChooser)
        .withPosition(0, 0)
        .withSize(2, 1);

    SmartDashboard.putData(this.pChooser);
    SmartDashboard.putData(this.iChooser);
    SmartDashboard.putData(this.dChooser);
    SmartDashboard.putData(this.gChooser);

    leftArmMotorConfig.idleMode(IdleMode.kBrake);
    rightArmMotorConfig.idleMode(IdleMode.kBrake);

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

    this.p = this.pChooser.getSelected();
    this.i = this.iChooser.getSelected();
    this.d = this.dChooser.getSelected();
    this.g = this.gChooser.getSelected();

    SmartDashboard.putNumber("ArmGoal", this.pidController.getGoal().position);
    SmartDashboard.putNumber("pos", getMeasurement());
    SmartDashboard.putNumber("encoder", getEncoder());
  }
}
