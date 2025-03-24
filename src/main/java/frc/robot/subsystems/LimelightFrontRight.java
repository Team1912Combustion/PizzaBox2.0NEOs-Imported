// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LimelightFrontRight extends LimelightFrontLeft {
  /** Creates a new LimelightFrontRight. */
  public LimelightFrontRight() {}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  @Override
  public String getName() {
    return "limelight-rear";
  }

  @Override
  public String getPosition() {
    return "rear";
  }
}
