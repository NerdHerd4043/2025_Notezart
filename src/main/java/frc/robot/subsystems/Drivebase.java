// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.List;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.Waypoint;
import com.pathplanner.lib.util.PathPlannerLogging;
import com.studica.frc.AHRS;

import cowlib.SwerveModule;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.BooleanEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.RobotConfigInfo;
import frc.robot.Constants.DriveConstants.ModuleLocations;
import frc.robot.Constants.DriveConstants.SwerveModules;
import frc.robot.Constants.PathPlannerConstants.RotationPID;
import frc.robot.Constants.PathPlannerConstants.TranslationPID;
import frc.robot.util.LimelightHelpers;
import frc.robot.util.LimelightHelpers.LimelightResults;
import frc.robot.util.LimelightHelpers.LimelightTarget_Fiducial;

public class Drivebase extends SubsystemBase {
  private final double DRIVE_REDUCTION = 1.0 / 6.75;
  private final double NEO_FREE_SPEED = 5820.0 / 60.0;
  private final double WHEEL_DIAMETER = 0.1016;
  private final double MAX_VELOCITY = NEO_FREE_SPEED * DRIVE_REDUCTION * WHEEL_DIAMETER * Math.PI;
  private final double MAX_ANGULAR_VELOCITY = MAX_VELOCITY / (ModuleLocations.dist / Math.sqrt(2.0));

  private final double MAX_VOLTAGE = 12;

  private final AHRS gyro = new AHRS(AHRS.NavXComType.kMXP_SPI);

  private SwerveModule frontLeft = new SwerveModule(SwerveModules.frontLeft, MAX_VELOCITY, MAX_VOLTAGE);
  private SwerveModule frontRight = new SwerveModule(SwerveModules.frontRight, MAX_VELOCITY, MAX_VOLTAGE);
  private SwerveModule backLeft = new SwerveModule(SwerveModules.backLeft, MAX_VELOCITY, MAX_VOLTAGE);
  private SwerveModule backRight = new SwerveModule(SwerveModules.backRight, MAX_VELOCITY, MAX_VOLTAGE);

  private SwerveModule[] modules = new SwerveModule[] { frontLeft, frontRight, backLeft, backRight };

  private SwerveDriveKinematics kinematics = new SwerveDriveKinematics(
      ModuleLocations.frontLeft,
      ModuleLocations.frontRight,
      ModuleLocations.backLeft,
      ModuleLocations.backRight);

  private SwerveDriveOdometry odometry;

  private Field2d field = new Field2d();

  private SlewRateLimiter slewRateX = new SlewRateLimiter(DriveConstants.slewRate);
  private SlewRateLimiter slewRateY = new SlewRateLimiter(DriveConstants.slewRate);

  private BooleanEntry fieldOrientedEntry;

  private SendableChooser<Double> driveSpeedChooser = new SendableChooser<>();
  private SendableChooser<Boolean> fieldOriented = new SendableChooser<>();

  /** Creates a new Drivebase. */
  public Drivebase() {
    var inst = NetworkTableInstance.getDefault();
    var table = inst.getTable("SmartDashboard");
    this.fieldOrientedEntry = table.getBooleanTopic("Field Oriented").getEntry(true);

    odometry = new SwerveDriveOdometry(kinematics, gyro.getRotation2d(), getPositions());

    this.driveSpeedChooser.setDefaultOption("Full Speed", 1.0);
    this.driveSpeedChooser.addOption("Three-Quarter Speed", 0.75);
    this.driveSpeedChooser.addOption("Half Speed", 0.5);
    this.driveSpeedChooser.addOption("Quarter Speed", 0.25);
    this.driveSpeedChooser.addOption("No Speed", 0.0);

    this.fieldOriented.setDefaultOption("Field Oriented", true);
    this.fieldOriented.addOption("Robot Oriented", false);

    SmartDashboard.putData(this.driveSpeedChooser);
    SmartDashboard.putData(this.fieldOriented);

    RobotConfig config;
    try {
      config = RobotConfig.fromGUISettings();
    } catch (Exception e) {
      // Handle exception as needed
      e.printStackTrace();
      config = RobotConfigInfo.robotConfig;
    }

    AutoBuilder.configure(
        this::getPose,
        this::resetPose,
        this::getCurrentSpeeds,
        this::drive,
        new PPHolonomicDriveController(
            new PIDConstants(TranslationPID.p, TranslationPID.i, TranslationPID.d),
            new PIDConstants(RotationPID.p, RotationPID.i, RotationPID.d)),
        config,
        () -> {
          var alliance = DriverStation.getAlliance();
          if (alliance.isPresent()) {
            return alliance.get() == DriverStation.Alliance.Red;
          } else {
            return false;
          }
        },
        this);

    PathPlannerLogging.setLogActivePathCallback((poses) -> field.getObject("path").setPoses(poses));
    SmartDashboard.putData("Field", field);
  }

  public double getFieldAngle() {
    return -gyro.getYaw();
  }

  public void fieldOrientedDrive(double speedX, double speedY, double rot) {
    ChassisSpeeds speeds = ChassisSpeeds.fromFieldRelativeSpeeds(speedX, speedY, rot,
        Rotation2d.fromDegrees(getFieldAngle()));
    this.drive(speeds);
  }

  public void robotOrientedDrive(double speedX, double speedY, double rot) {
    ChassisSpeeds speeds = new ChassisSpeeds(speedX, speedY, rot);
    this.drive(speeds);
  }

  public void defaultDrive(double speedX, double speedY, double rot) {
    defaultDrive(speedX, speedY, rot, true);
  }

  public void defaultDrive(double speedX, double speedY, double rot, boolean slew) {
    if (slew) {
      speedX = slewRateX.calculate(speedX);
      speedY = slewRateY.calculate(speedY);
    }

    if (this.fieldOrientedEntry.get(this.getDefaultDrive())) {
      fieldOrientedDrive(speedX, speedY, rot);
    } else {
      robotOrientedDrive(speedX, speedY, rot);
    }
  }

  public void drive(ChassisSpeeds speeds) {
    SwerveModuleState[] moduleStates = kinematics.toSwerveModuleStates(speeds,
        new Translation2d(Units.inchesToMeters(4), 0));

    SwerveDriveKinematics.desaturateWheelSpeeds(moduleStates, MAX_VELOCITY * getRobotSpeedRatio());

    this.frontLeft.drive(moduleStates[0]);
    this.frontRight.drive(moduleStates[1]);
    this.backLeft.drive(moduleStates[2]);
    this.backRight.drive(moduleStates[3]);

    SmartDashboard.putNumber("FL Target Angle", moduleStates[0].angle.getDegrees());
  }

  public double getMaxVelocity() {
    return MAX_VELOCITY;
  }

  public double getMaxAngleVelocity() {
    return MAX_ANGULAR_VELOCITY;
  }

  public Pose2d getPose() {
    return odometry.getPoseMeters();
  }

  public void resetPose(Pose2d pose2d) {
    odometry.resetPosition(gyro.getRotation2d(), getPositions(), pose2d);
  }

  public ChassisSpeeds getCurrentSpeeds() {
    return kinematics.toChassisSpeeds(getModuleStates());
  }

  public SwerveModuleState[] getModuleStates() {
    SwerveModuleState[] states = new SwerveModuleState[modules.length];
    for (int i = 0; i < modules.length; i++) {
      states[i] = modules[i].getState();
    }
    return states;
  }

  public SwerveModulePosition[] getPositions() {
    SwerveModulePosition[] positions = new SwerveModulePosition[modules.length];
    for (int i = 0; i < modules.length; i++) {
      positions[i] = modules[i].getPosition();
    }
    return positions;
  }

  public Command getAlignCommand() {
    var initPos = new Pose2d(0, 2, Rotation2d.fromDegrees(0));

    this.resetPose(initPos);

    List<Waypoint> waypoints = PathPlannerPath.waypointsFromPoses(
        initPos,
        new Pose2d(1, 0, Rotation2d.fromDegrees(0)),
        new Pose2d(-1, 0, Rotation2d.fromDegrees(0)),
        new Pose2d(0, -2, Rotation2d.fromDegrees(0)));

    PathConstraints constraints = new PathConstraints(
        2.750, // Max Velocity
        2.183, // Max Acceleration
        360, // Max Angular Velocity
        360 // Max Angular Acceleration
    );

    PathPlannerPath path = new PathPlannerPath(waypoints, constraints, null,
        new GoalEndState(0.0, Rotation2d.fromDegrees(0)));

    path.preventFlipping = true;

    return AutoBuilder.followPath(path);
  }

  public double getRobotSpeedRatio() {
    return this.driveSpeedChooser.getSelected();
  }

  public boolean getDefaultDrive() {
    return this.fieldOriented.getSelected();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    var positions = getPositions();

    odometry.update(gyro.getRotation2d(), positions);
    var pose = getPose();

    var translation = pose.getTranslation();
    var x = translation.getX();
    var y = translation.getY();
    var rotation = pose.getRotation().getDegrees();
    SmartDashboard.putNumber("x", x);
    SmartDashboard.putNumber("y", y);
    SmartDashboard.putNumber("rot", rotation);
    field.setRobotPose(getPose());

    SmartDashboard.putNumber("module output", modules[0].getDriveOutput());

    SmartDashboard.putNumber("FL Encoder", frontLeft.getEncoder());
    SmartDashboard.putNumber("FR Encoder", frontRight.getEncoder());
    SmartDashboard.putNumber("BR Encoder", backRight.getEncoder());
    SmartDashboard.putNumber("BL Encoder", backLeft.getEncoder());

    SmartDashboard.putNumber("Speed Ratio", getRobotSpeedRatio());
  }
}
