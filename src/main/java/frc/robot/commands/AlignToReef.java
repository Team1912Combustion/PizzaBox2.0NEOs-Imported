// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.LimelightFrontLeft;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class AlignToReef extends Command {
  DriveTrain driveTrain;
  LimelightFrontLeft limelight;

  private PIDController xController, yController, rotController;
  private boolean isRightScore;
  /** Creates a new AlignToReefRight. */
  public AlignToReef(DriveTrain dt, LimelightFrontLeft l, boolean isRightScore) {
    driveTrain = dt;
    limelight = l;
    addRequirements(driveTrain);

    xController = new PIDController(0.35, 0, 0);
    yController = new PIDController(0.35, 0, 0);
    rotController = new PIDController(0.006, 0, 0);

    this.isRightScore = isRightScore;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    xController.setSetpoint(-0.5);
    xController.setTolerance(0.02);

    yController.setSetpoint(isRightScore ? 0.16 : -0.16);
    yController.setTolerance(0.02);

    rotController.setSetpoint(0);
    rotController.setTolerance(1);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (driveTrain.isTargetSpacePoseValid()) {
      double[] positions = limelight.getBotPoseTargetSpace();

      double xSpeed;
      if (xController.atSetpoint()) {
        xSpeed = -driveTrain.driverController.getLeftY();
      } else {
        xSpeed = xController.calculate(driveTrain.getPose2dTargetSpace().getY());
      }
      double ySpeed = -yController.calculate(driveTrain.getPose2dTargetSpace().getX());
      double rotSpeed = -rotController.calculate(driveTrain.getPose2dTargetSpace().getRotation().getDegrees());
      driveTrain.drive(xSpeed, ySpeed, rotSpeed, false);
    } else {
      driveTrain.drive(0, 0, 0, false);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    driveTrain.drive(0, 0, 0, true);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
