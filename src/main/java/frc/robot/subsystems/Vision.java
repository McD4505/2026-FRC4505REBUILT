// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Vision extends SubsystemBase {

  private final PhotonCamera aprilTagCamera; // Feel free to rename in whatever way you want

  /** Creates a new ExampleSubsystem. */
  public Vision() {
    aprilTagCamera = new PhotonCamera("aprilTagCamera"); // Accessing camera by its name in photonvision.local:5800

    SmartDashboard.setDefaultBoolean("aprilTagCameraHasTargets", false);
    
    SmartDashboard.setDefaultNumber("aprilTagCameraTargetYaw", 0);
    SmartDashboard.setDefaultNumber("aprilTagCameraTargetPitch", 0);
    SmartDashboard.setDefaultNumber("aprilTagCameraTargetArea", 0);
    SmartDashboard.setDefaultNumber("aprilTagCameraTargetID", 0);
    SmartDashboard.setDefaultNumber("aprilTagCameraTargetPoseAmbiguity", 0);

    //PhotonVision Documentation Adjusted

    // var aprilTagCameraResult = aprilTagCamera.getLatestResult(); // Query the latest result from PhotoVision
    // boolean aprilTagCameraHasTargets = aprilTagCameraResult.hasTargets(); // Check if the latest result has any targets.

    // List<PhotonTrackedTarget> aprilTagCameraTargets = aprilTagCameraResult.getTargets(); // Get a list of currently tracked targets.
    // PhotonTrackedTarget aprilTagCameraTarget = aprilTagCameraResult.getBestTarget(); // Get the current best target.
      // double aprilTagCameraTargetYaw = aprilTagCameraTarget.getYaw(); // Get the yaw of the current best target (positive left).
      // double aprilTagCameraTargetPitch = aprilTagCameraTarget.getPitch(); // Get the pitch of the current best target (positive up).
      // double aprilTagCameraTargetArea = aprilTagCameraTarget.getArea(); // Get the area (how much of the camera feed the bounding box takes up) as a percent (0-100).
      // Not Available // double aprilTagCameraTargetSkew = aprilTagCameraTarget.getSkew(); Get the skew of the current best target (counter-clockwise positive).
      // Transform2d aprilTagCameraTargetPose = aprilTagCameraTarget.getCameraToTarget(); // The camera to target transform. See 2d transform documentation.
      // List<TargetCorner> aprilTagCameraTargetCorners = aprilTagCameraTarget.getCorners(); // The 4 corners of the minimum bounding box rectangle.

      // int aprilTagCameraTargetID = aprilTagCameraTarget.getFiducialID; // The ID of the detected fiducial marker.
      // double aprilTagCameraTargetPoseAmbiguity = aprilTagCameraTarget.getPoseAmbiguity(); // How ambiguous the pose of the target is.
      // Transform3d bestCameraToTarget = target.getBestCameraToTarget(); // Get the transform that maps camera space (X = forward, Y = left, Z = up) with the lowest reprojection error.
      // Transform3d alternateCameraToTarget = target.getAlternateCameraToTarget(); // Get the transform that maps camera space (X = forward, Y = Left, Z = up) with the highest reprojection error.
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    var aprilTagCameraResult = aprilTagCamera.getLatestResult(); // Query the latest result from PhotoVision

    SmartDashboard.putBoolean("aprilTagCameraHasTargets", aprilTagCameraResult.hasTargets());
    
    PhotonTrackedTarget aprilTagCameraTarget = aprilTagCameraResult.getBestTarget(); // Get the current best target.

    SmartDashboard.putNumber("aprilTagCameraTargetYaw", aprilTagCameraTarget.getYaw());
    SmartDashboard.putNumber("aprilTagCameraTargetPitch", aprilTagCameraTarget.getPitch());
    SmartDashboard.putNumber("aprilTagCameraTargetArea", aprilTagCameraTarget.getArea());
    SmartDashboard.putNumber("aprilTagCameraTargetID", aprilTagCameraTarget.getFiducialId());
    SmartDashboard.putNumber("aprilTagCameraTargetPoseAmbiguity", aprilTagCameraTarget.getPoseAmbiguity());

  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}