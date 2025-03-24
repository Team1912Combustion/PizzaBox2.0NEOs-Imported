// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.Pigeon2;

import edu.wpi.first.math.filter.MedianFilter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.VisionConstants;

public class DriveTrain extends SubsystemBase {
  private final SwerveModule m_frontLeft =
    new SwerveModule(
      DriveConstants.FRONT_LEFT_DRIVE_MOTOR_ID,
      DriveConstants.FRONT_LEFT_TURNING_MOTOR_ID,
      DriveConstants.FRONT_LEFT_TURNING_ENCODER_ID,
      DriveConstants.FRONT_LEFT_MAGNET_OFFSET);

  private final SwerveModule m_rearLeft =
    new SwerveModule(
      DriveConstants.REAR_LEFT_DRIVE_MOTOR_ID,
      DriveConstants.REAR_LEFT_TURNING_MOTOR_ID,
      DriveConstants.REAR_LEFT_TURNING_ENCODER_ID,
      DriveConstants.REAR_LEFT_MAGNET_OFFSET);

  private final SwerveModule m_frontRight =
    new SwerveModule(
      DriveConstants.FRONT_RIGHT_DRIVE_MOTOR_ID,
      DriveConstants.FRONT_RIGHT_TURNING_MOTOR_ID,
      DriveConstants.FRONT_RIGHT_TURNING_ENCODER_ID,
      DriveConstants.FRONT_RIGHT_MAGNET_OFFSET);

  private final SwerveModule m_rearRight =
    new SwerveModule(
      DriveConstants.REAR_RIGHT_DRIVE_MOTOR_ID,
      DriveConstants.REAR_RIGHT_TURNING_MOTOR_ID,
      DriveConstants.REAR_RIGHT_TURNING_ENCODER_ID,
      DriveConstants.REAR_RIGHT_MAGNET_OFFSET);

  public final Pigeon2 m_gyro = new Pigeon2(0, "1912pizzavore");

  LimelightFrontLeft limelightFrontLeft;
  LimelightFrontRight limelightFrontRight;

  public boolean fieldRelative;

  MedianFilter limelightXFilter;
  MedianFilter limelightYFilter;
  MedianFilter limelightYawFilter;

  MedianFilter targetSpaceXFilter;
  MedianFilter targetSpaceYFilter;
  MedianFilter targetSpaceYawFilter;

  double compositeLatency;
  Pose2d compositeVisionPose;
  Pose2d compositeTargetSpacePose;

  boolean isVisionValid;
  boolean isTargetSpacePoseValid;

  public XboxController driverController;

  /** Creates a new DriveTrain. */
  public DriveTrain(LimelightFrontLeft lfl, LimelightFrontRight lfr) {

    limelightFrontLeft = lfl;
    limelightFrontRight = lfr;
    fieldRelative = true;

    limelightXFilter = new MedianFilter(3);
    limelightYFilter = new MedianFilter(3);
    limelightYawFilter = new MedianFilter(3);

    targetSpaceXFilter = new MedianFilter(3);
    targetSpaceYFilter = new MedianFilter(3);
    targetSpaceYawFilter = new MedianFilter(3);

    compositeLatency = 0;
    compositeVisionPose = new Pose2d();
    compositeTargetSpacePose = new Pose2d();

    isVisionValid = true;
    isTargetSpacePoseValid = true;

    driverController = new XboxController(0);
  }

  @Override
  public void periodic() {
    processFrame();
    SmartDashboard.putNumber("gyro", getHeading());
    SmartDashboard.putNumber("front left position", m_frontLeft.getPosition().angle.getDegrees());
    // This method will be called once per scheduler run
  }

  public void drive(double xSpeed, double ySpeed, double rot,
    boolean fieldRelative)
  {
    double m_xSpeed = xSpeed * DriveConstants.MAX_SPEED_METERS_PER_SECOND;
    double m_ySpeed = ySpeed * DriveConstants.MAX_SPEED_METERS_PER_SECOND;
    double m_rot = rot * DriveConstants.MAX_ANGULAR_SPEED_RADIANS_PER_SECOND;

    var swerveModuleStates =
      DriveConstants.DRIVE_KINEMATICS.toSwerveModuleStates(
        (fieldRelative)
          //   Teleop otherwise auto
          ? ChassisSpeeds.fromFieldRelativeSpeeds(m_xSpeed, m_ySpeed, m_rot,
              getRotation2d())
          : new ChassisSpeeds(m_xSpeed, m_ySpeed, m_rot));

    setModuleStates(swerveModuleStates);
  }

  public SwerveModuleState[] getModuleStates() {
    var myModuleStates = new SwerveModuleState[4];
    myModuleStates[0] = m_frontLeft.getState();
    myModuleStates[1] = m_rearLeft.getState();
    myModuleStates[2] = m_rearRight.getState();
    myModuleStates[3] = m_frontRight.getState();
    return myModuleStates;
  }

  public void setModuleStates(SwerveModuleState[] desiredStates) {
    SwerveDriveKinematics.desaturateWheelSpeeds(
        desiredStates, DriveConstants.MAX_SPEED_METERS_PER_SECOND);
    m_frontLeft.setDesiredState(desiredStates[0]);
    m_rearLeft.setDesiredState(desiredStates[1]);
    m_rearRight.setDesiredState(desiredStates[2]);
    m_frontRight.setDesiredState(desiredStates[3]);
  }

  public ChassisSpeeds getChassisSpeeds() {
    return DriveConstants.DRIVE_KINEMATICS.toChassisSpeeds(getModuleStates());
  }

  public void resetEncoders() {
    m_frontLeft.resetEncoders();
    m_rearLeft.resetEncoders();
    m_frontRight.resetEncoders();
    m_rearRight.resetEncoders();
  }

  public void stop() {
    var swerveModuleStates =
      DriveConstants.DRIVE_KINEMATICS.toSwerveModuleStates(
        new ChassisSpeeds(0., 0., 0.));
    setModuleStates(swerveModuleStates);
  }

  public Rotation2d getRotation2d() {
    return Rotation2d.fromDegrees(m_gyro.getYaw().getValueAsDouble());
  }

  public double getHeading() {
    return m_gyro.getYaw().getValueAsDouble();
  }

  public void zeroHeading() {
    m_gyro.reset();
  }

  public SwerveModulePosition[] get_positions() {
    SwerveModulePosition[] m_positions = {m_frontLeft.getPosition(),m_rearLeft.getPosition(),
    m_rearRight.getPosition(),m_frontRight.getPosition()};
    return m_positions;
  }

  public void processFrame() {
    double x = 0;
    double y = 0;
    double yaw = 0;
    double totalArea = 0;
    isVisionValid = false;

    if (limelightFrontLeft.getTagId() > 0) {
      if (limelightFrontLeft.getTargetArea() > VisionConstants.TARGET_AREA_THRESHHOLD) {
        totalArea += limelightFrontLeft.getTargetArea();
        x += limelightFrontLeft.getBotPose2d().getX() * limelightFrontLeft.getTargetArea();
        y += limelightFrontLeft.getBotPose2d().getY() * limelightFrontLeft.getTargetArea();
        yaw += limelightFrontLeft.getBotPose2d().getRotation().getDegrees() * limelightFrontLeft.getTargetArea();
        compositeLatency += limelightFrontLeft.getLatency();
      }
    }

    if (limelightFrontRight.getTagId() > 0) {
      if (limelightFrontRight.getTargetArea() > VisionConstants.TARGET_AREA_THRESHHOLD) {
        totalArea += limelightFrontRight.getTargetArea();
        x += limelightFrontRight.getBotPose2d().getX() * limelightFrontRight.getTargetArea();
        y += limelightFrontRight.getBotPose2d().getY() * limelightFrontRight.getTargetArea();
        yaw += limelightFrontRight.getBotPose2d().getRotation().getDegrees() * limelightFrontRight.getTargetArea();
        compositeLatency += limelightFrontRight.getLatency();
      }
    } 

    if (totalArea < VisionConstants.TOTAL_TARGET_AREA_THRESHHOLD) {
      isVisionValid = false;
      compositeLatency = 0;
    } else {
      isVisionValid = true;
      x /= totalArea;
      y /= totalArea;
      yaw /= totalArea;
      compositeLatency /= totalArea;

      compositeVisionPose = new Pose2d(
        limelightXFilter.calculate(x),
        limelightYFilter.calculate(y),
        Rotation2d.fromDegrees(limelightYawFilter.calculate(yaw))
      );
    }

  }

  public void calculateFrameTargetSpace() {
    double x = 0;
    double y = 0;
    double yaw = 0;
    double totalArea = 0;
    isTargetSpacePoseValid = false;

    if (limelightFrontLeft.getTagId() > 0) {
      if (limelightFrontLeft.getTargetArea() > VisionConstants.TARGET_AREA_THRESHHOLD) {
        totalArea += limelightFrontLeft.getTargetArea();
        x += limelightFrontLeft.getBotPose2dTargetSpace().getX() * limelightFrontLeft.getTargetArea();
        y += limelightFrontLeft.getBotPose2dTargetSpace().getY() * limelightFrontLeft.getTargetArea();
        yaw += limelightFrontLeft.getBotPose2dTargetSpace().getRotation().getDegrees() * limelightFrontLeft.getTargetArea();
        compositeLatency += limelightFrontLeft.getLatency();
      }
    }

    if (limelightFrontRight.getTagId() > 0) {
      if (limelightFrontRight.getTargetArea() > VisionConstants.TARGET_AREA_THRESHHOLD) {
        totalArea += limelightFrontRight.getTargetArea();
        x += limelightFrontRight.getBotPose2dTargetSpace().getX() * limelightFrontRight.getTargetArea();
        y += limelightFrontRight.getBotPose2dTargetSpace().getY() * limelightFrontRight.getTargetArea();
        yaw += limelightFrontRight.getBotPose2dTargetSpace().getRotation().getDegrees() * limelightFrontRight.getTargetArea();
        compositeLatency += limelightFrontRight.getLatency();
      }
    } 

    if (totalArea < VisionConstants.TOTAL_TARGET_AREA_THRESHHOLD) {
      isTargetSpacePoseValid = false;
      compositeLatency = 0;
    } else {
      isTargetSpacePoseValid = true;
      x /= totalArea;
      y /= totalArea;
      yaw /= totalArea;
      compositeLatency /= totalArea;

      compositeTargetSpacePose = new Pose2d(
        targetSpaceXFilter.calculate(x),
        targetSpaceYFilter.calculate(y),
        Rotation2d.fromDegrees(targetSpaceYawFilter.calculate(yaw))
      );
    }

  }

  public Pose2d getPose2dTargetSpace() {
    return compositeTargetSpacePose;
  }
  
  public boolean isTargetSpacePoseValid() {
    return isTargetSpacePoseValid;
  }
}
