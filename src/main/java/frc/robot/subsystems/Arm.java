// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.EnumSet;
import java.util.concurrent.atomic.AtomicReference;

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
import edu.wpi.first.epilogue.NotLogged;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEvent;
import edu.wpi.first.networktables.NetworkTableInstance;
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

  ///// declare network table stuff
  final DoubleSubscriber gSub;
  final AtomicReference<Double> gValue = new AtomicReference<Double>();
  // weird handler thing
  int connListenerHandle;
  int valueListenerHandle;
  int topicListenerHandle;
  ///////// end network table declarations

  private CANcoder encoder = new CANcoder(ArmConstants.encoderID);

  private ArmFeedforward feedforward = new ArmFeedforward(FeedForwardValues.kS,
      FeedForwardValues.kG,
      FeedForwardValues.kV);

  private double ffOutput;
  private boolean podium = false;

  @SuppressWarnings("unused")
  private SparkLimitSwitch rightReverseLimitSwitch;

  private ProfiledPIDController pidController = new ProfiledPIDController(
      PIDValues.p,
      PIDValues.i,
      PIDValues.d,
      new TrapezoidProfile.Constraints(6, 5));

  // /** Creates a new ProfPIDArm. */
  public Arm() {
    NetworkTableInstance inst = NetworkTableInstance.getDefault();

    connListenerHandle = inst.addConnectionListener(true, event -> {
      if (event.is(NetworkTableEvent.Kind.kConnected)) {
        System.out.println("Connected to " + event.connInfo.remote_id);
      } else if (event.is(NetworkTableEvent.Kind.kDisconnected)) {
        System.out.println("Disconnected from " + event.connInfo.remote_id);
      }
    });
    // get the subtable called "datatable"
    NetworkTable datatable = inst.getTable("datatable");
    // subscribe to the topic in "datatable" called "Y"
    gSub = datatable.getDoubleTopic("G").subscribe(FeedForwardValues.kG);
    // add a listener to only value changes on the Y subscriber
    valueListenerHandle = inst.addListener(
        gSub,
        EnumSet.of(NetworkTableEvent.Kind.kValueAll),
        event -> {
          // can only get doubles because it's a DoubleSubscriber, but
          // could check value.isDouble() here too
          gValue.set(event.valueData.value.getDouble());
        });
    // add a listener to see when new topics are published within datatable
    // the string array is an array of topic name prefixes.
    topicListenerHandle = inst.addListener(
        new String[] { datatable.getPath() + "/" },
        EnumSet.of(NetworkTableEvent.Kind.kTopic),
        event -> {
          if (event.is(NetworkTableEvent.Kind.kPublish)) {
            // topicInfo.name is the full topic name, e.g. "/datatable/X"
            System.out.println("newly published " + event.topicInfo.name);
          }
        });

    // end network tables stuffs
    SparkMaxConfig leftArmMotorConfig = new SparkMaxConfig();
    SparkMaxConfig rightArmMotorConfig = new SparkMaxConfig();

    leftArmMotorConfig.idleMode(IdleMode.kBrake);
    rightArmMotorConfig.idleMode(IdleMode.kBrake);

    // tune the arm
    SmartDashboard.putData(this.pidController);

    // current limit
    leftArmMotorConfig.smartCurrentLimit(ArmConstants.motorCurrentLimit);
    rightArmMotorConfig.smartCurrentLimit(ArmConstants.motorCurrentLimit);

    // set follow
    leftArmMotorConfig.follow(ArmConstants.rightArmMotorID, true);
    // leftArmMotorConfig.inverted(true);
    // rightArmMotorConfig.inverted(false);

    rightArmMotorConfig.limitSwitch.reverseLimitSwitchType(LimitSwitchConfig.Type.kNormallyClosed);

    leftArmMotor.configure(leftArmMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    rightArmMotor.configure(rightArmMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

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

  @Logged
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

    // update gravity PID

    // get the latest value by reading the AtomicReference; set it to null
    // when we read to ensure we only get value changes
    Double value = gValue.getAndSet(null);
    if (value != null) {
      feedforward.setKg(value);
    }

  }

  public void close() {

    NetworkTableInstance inst = NetworkTableInstance.getDefault();
    inst.removeListener(topicListenerHandle);
    inst.removeListener(valueListenerHandle);
    inst.removeListener(connListenerHandle);
    gSub.close();

  }

}
