// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LimelightFrontLeft extends SubsystemBase {
  private final NetworkTable table =
	  NetworkTableInstance.getDefault().getTable(getName());
  private final NetworkTableEntry pipeline = table.getEntry("pipeline");
  private final NetworkTableEntry xOffset = table.getEntry("tx");
  private final NetworkTableEntry yOffset = table.getEntry("ty");
  private final NetworkTableEntry latency = table.getEntry("tl");
  private final NetworkTableEntry botPose = table.getEntry("botpose_wpiblue");
  private final NetworkTableEntry botPoseTargetSpace = table.getEntry("botpose_targetspace");
  private final NetworkTableEntry tagId = table.getEntry("tid");
  private final NetworkTableEntry targetArea = table.getEntry("ta");
  private double[] lastBotPose = new double[6];
  private int currentPipeline = 0;
  private int currentTagId = 0;

  public String getName() {
    return "limelight-fl";
  }
  public String getPosition() {
    return "frontLeft";
  }

  @Override
  public void periodic() {
    currentTagId = getTagId();
    lastBotPose = getBotPose();
    if(currentTagId > 0) {
     // lastPose2d = new Pose2d(lastBotPose[0],lastBotPose[1], new Rotation2d(lastBotPose[5]));
      SmartDashboard.putNumber(getPosition()+"Botpose X: ",lastBotPose[0]);
      SmartDashboard.putNumber(getPosition()+"Botpose Y: ",lastBotPose[1]);

      SmartDashboard.putNumber(getPosition()+"Botpose Yaw: ",lastBotPose[5]);

    }       
    SmartDashboard.putNumber(getPosition()+"Latency: ",(double) latency.getNumber(0));
    SmartDashboard.putNumber(getPosition()+"TagID: ", tagId.getInteger(-1));
    SmartDashboard.putBoolean(getPosition()+"HasBotPose: ", (currentTagId > 0));
    // This method will be called once per scheduler run
  }

  public void togglezoom() {
    if (currentPipeline == 0)
      currentPipeline = 1;
    else if (currentPipeline ==1)
      currentPipeline = 0;
  }

  /**
   * Returns the Apriltags estimated pose or null if there is no pose.
   * @return
   */
  public Pose2d getBotPose2d() {
    return new Pose2d(getBotPose()[0],getBotPose()[1], new Rotation2d(Math.toRadians(getBotPose()[5])));
  }

  public Pose2d getBotPose2dTargetSpace() {
    return new Pose2d(getBotPoseTargetSpace()[0], getBotPoseTargetSpace()[1], new Rotation2d(Math.toRadians(getBotPoseTargetSpace()[5])));
  }

  /**
   * Returns the latency of the pipeline 
   * @return
   */
  public double getLatency() {
    return latency.getDouble(0);
  }

  /**
   * Returns null if we do not see any apriltags.
   * https://docs.limelightvision.io/en/latest/networktables_api.html
   * @return
   */
  public double[] getBotPose() {
    return botPose.getDoubleArray(new double[7]);
  }

  public double[] getBotPoseTargetSpace() {
    return botPoseTargetSpace.getDoubleArray(new double[7]);
  }

  public int getTagId() {
    return (int)tagId.getInteger(0);
  }

  public double getXOffset() {
    if (tagId.getInteger(-1) == 7) {
      return xOffset.getDouble(0);
    } else {
      return 0;
    }
  }

  public double getYOffset() {
    return yOffset.getDouble(0);
  }

  public void setLedsOn() {
    NetworkTableInstance.getDefault().getTable(getName())
      .getEntry("ledMode").setNumber(3);
  }

  public void setPipeline(int id) {
    pipeline.setNumber(id);
  }

  public double getTargetArea() {
    return targetArea.getDouble(0);
  }

}
