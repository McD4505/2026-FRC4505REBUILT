// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.proto.Photon;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import org.photonvision.targeting.PhotonTrackedTarget;


import java.util.List;


public class Vision extends SubsystemBase {
  private double lastVisionX = 0.0;
  private double lastVisionY = 0.0;
  private double lastVisionHeadingDeg = 0.0;
  private double lastVisionTimestamp = 0.0;
  private int lastVisionTagCount = 0;
  private boolean hasVisionPose = false;

  //Transform of the camera rotation and translation relative to the center of the robot
  private final Transform3d robotToCamTransform;
  
  // April Tag Field Layout
  public static final AprilTagFieldLayout kTagLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltAndymark);

  // Photon Vision Simulation
  public VisionSystemSim visionSim;

  public static final Transform3d kRobotToCam =
                new Transform3d(new Translation3d(0.5, 0.0, 0.5), new Rotation3d(0, 0, 0));

  // Cameras heheheha
  PhotonCamera color;
  PhotonCamera bw1;
  PhotonCamera bw2;
  
  PhotonCameraSim cameraSim;
  PhotonPoseEstimator photonEstimator;



  public Vision(VisionMeasurementConsumer estConsumer) {
     photonEstimator = new PhotonPoseEstimator(kTagLayout, kRobotToCam);
     color = new PhotonCamera("Color");
      bw1 = new PhotonCamera("BandW1");
      bw2 = new PhotonCamera("BandW2");



      this.estConsumer = estConsumer;

    if (Robot.isSimulation()) {
      initSimField();
      // Our camera is mounted 0.1 meters forward and 0.5 meters up from the robot pose,
      // (Robot pose is considered the center of rotation at the floor level, or Z = 0)
      Translation3d robotToCameraTrl = new Translation3d(0.1, 0, 0.5);
      // and pitched 15 degrees up.
      Rotation3d robotToCameraRot = new Rotation3d(0, Math.toRadians(-15), 0);
      Transform3d robotToCamera = new Transform3d(robotToCameraTrl, robotToCameraRot);

      // Add this camera to the vision system simulation with the given robot-to-camera transform.
      visionSim.addCamera(cameraSim, robotToCamera);
    }
  }

  /**
   * Example command factory method.
   *
   * @return a command
   */
  public Command exampleMethodCommand() {
    // Inline construction of command goes here.
    // Subsystem::RunOnce implicitly requires `this` subsystem.
    return runOnce(
        () -> {
          /* one-time action goes here */
        });
  }

  /**
   * An example method querying a boolean state of the subsystem (for example, a digital sensor).
   *
   * @return value of some boolean subsystem state, such as a digital sensor.
   */
  public boolean exampleCondition() {
    // Query some boolean state, such as a digital sensor.
    return false;
  }

  @FunctionalInterface
  public interface VisionMeasurementConsumer {
    void accept(
        edu.wpi.first.math.geometry.Pose2d pose,
        double timestampSeconds,
        Matrix<N3, N1> stdDevs);
  }

  private final VisionMeasurementConsumer estConsumer;

  // For sim visualization
  private final Field2d simField = new Field2d();

  // Current std devs used for the next vision measurement
  private Matrix<N3, N1> estimationStdDevs = VecBuilder.fill(1.5, 1.5, 3.0);

  /** Call this in your constructor after fields are constructed */
  private void initSimField() {
    SmartDashboard.putData("Field", simField);
    // TargetModel targetModel = new TargetModel(0.5, 0.25);
    // // The pose of where the target is on the field.
    // // Its rotation determines where "forward" or the target x-axis points.
    // // Let's say this target is flat against the far wall center, facing the blue driver stations.
    // Pose3d targetPose = new Pose3d(16, 4, 2, new Rotation3d(0, 0, Math.PI));
    // // The given target model at the given pose
    // VisionTargetSim visionTarget = new VisionTargetSim(targetPose, targetModel);

    // Add this vision target to the vision system simulation to make it visible
    //visionSim.addVisionTargets(visionTarget);
    visionSim.addAprilTags(kTagLayout);
    SimCameraProperties cameraProp = new SimCameraProperties();
    setCamProp(cameraProp);
    cameraSim = new PhotonCameraSim(bw1, cameraProp);
  }
  private void setCamProp(SimCameraProperties cameraProp) {
    // The simulated camera properties
    // A 640 x 480 camera with a 100 degree diagonal FOV.
    cameraProp.setCalibration(640, 480, Rotation2d.fromDegrees(100));
    // Approximate detection noise with average and standard deviation error in pixels.
    cameraProp.setCalibError(0.25, 0.08);
    // Set the camera image capture framerate (Note: this is limited by robot loop rate).
    cameraProp.setFPS(20);
    // The average and standard deviation in milliseconds of image data latency.
    cameraProp.setAvgLatencyMs(35);
    cameraProp.setLatencyStdDevMs(5);
  }

  private Field2d getSimDebugField() {
    return simField;
  }

  private Matrix<N3, N1> getEstimationStdDevs() {
    return estimationStdDevs;
  }

  /**
   * Very common heuristic:
   * - more tags => trust more
   * - farther tags => trust less
   */
  private void updateEstimationStdDevs(
      Optional<EstimatedRobotPose> visionEst,
      List<PhotonTrackedTarget> targets) {

    // If we didn't get a pose, keep it very untrusted
    if (visionEst.isEmpty() || targets.isEmpty()) {
      estimationStdDevs = VecBuilder.fill(4.0, 4.0, 8.0);
      return;
    }

    // Average distance to targets (meters)
    double avgDist = 0.0;
    for (var t : targets) {
      avgDist += t.getBestCameraToTarget().getTranslation().getNorm();
    }
    avgDist /= targets.size();

    int tagCount = targets.size();

    // Base trust
    double xy = 0.8;
    double theta = 2.5;

    // More tags => better
    if (tagCount >= 2) {
      xy *= 0.6;
      theta *= 0.6;
    }
    if (tagCount >= 3) {
      xy *= 0.6;
      theta *= 0.7;
    }

    // Farther => worse (tune these numbers to your camera + mounting)
    double distFactor = 1.0 + (avgDist * avgDist) / 4.0; // grows with distance^2
    xy *= distFactor;
    theta *= distFactor;

    // Clamp so it doesn't go insane
    xy = Math.min(Math.max(xy, 0.1), 6.0);
    theta = Math.min(Math.max(theta, 0.2), 12.0);

    estimationStdDevs = VecBuilder.fill(xy, xy, theta);
  }
  public void addToVisionSim(VisionSystemSim systemSim)
    {
      if (Robot.isSimulation())
      {
        systemSim.addCamera(cameraSim, robotToCamTransform);
      }
    }


@Override
public void periodic() {
  
  var result_color = color.getLatestResult();
  var result_bw1 = bw1.getLatestResult();
  var result_bw2 = bw2.getLatestResult();

  boolean hasTargets_color = result_color.hasTargets();
  boolean hasTargets_bw1 = result_bw1.hasTargets();
  boolean hasTargets_bw2 = result_bw2.hasTargets();
  

  
  // Optional<EstimatedRobotPose> lastEst = Optional.empty();
  // int lastTagCount = 0;

  // for (var result : color.getAllUnreadResults()) {
  //   Optional<EstimatedRobotPose> visionEst = photonEstimator.estimateCoprocMultiTagPose(result);
  //   if (visionEst.isEmpty()) {
  //     visionEst = photonEstimator.estimateLowestAmbiguityPose(result);
  //   }

  //   updateEstimationStdDevs(visionEst, result.getTargets());

  //   if (Robot.isSimulation()) {
  //     visionEst.ifPresentOrElse(
  //         est ->
  //             getSimDebugField()
  //                 .getObject("VisionEstimation")
  //                 .setPose(est.estimatedPose.toPose2d()),
  //         () -> getSimDebugField().getObject("VisionEstimation").setPoses());
  //   }

  //   // Feed drivetrain pose estimator (odometry fusion)
  //   visionEst.ifPresent(est -> {
  //     var estStdDevs = getEstimationStdDevs();
  //     estConsumer.accept(est.estimatedPose.toPose2d(), est.timestampSeconds, estStdDevs);
  //   });

  //   // Save only the newest one for dashboard output
  //   if (visionEst.isPresent()) {
     
  //     var est = visionEst.get();
  //     var pose2d = est.estimatedPose.toPose2d();
  //     estConsumer.accept(pose2d, est.timestampSeconds, getEstimationStdDevs());
  //     hasVisionPose = true;
  //     lastVisionX = pose2d.getX();
  //     lastVisionY = pose2d.getY();
  //     lastVisionHeadingDeg = pose2d.getRotation().getDegrees();
  //     lastVisionTimestamp = est.timestampSeconds;
  //     lastVisionTagCount = result.getTargets().size();
  //   }
  // }
  // SmartDashboard.putBoolean("VisionPose/HasPose", hasVisionPose);
  // // Dashboard output ONCE (latest estimate)
  // if (hasVisionPose) {
  //   SmartDashboard.putNumber("VisionPose/X", lastVisionX);
  //   SmartDashboard.putNumber("VisionPose/Y", lastVisionY);
  //   SmartDashboard.putNumber("VisionPose/HeadingDeg", lastVisionHeadingDeg);
  //   SmartDashboard.putNumber("VisionPose/TagCount", lastVisionTagCount);
  //   SmartDashboard.putNumber("VisionPose/Timestamp", lastVisionTimestamp);
  // }
  // else {
  //   SmartDashboard.putNumber("VisionPose/X", 0);
  //   SmartDashboard.putNumber("VisionPose/Y", 0);
  //   SmartDashboard.putNumber("VisionPose/HeadingDeg", 0);
  //   SmartDashboard.putNumber("VisionPose/TagCount", 0);
  //   SmartDashboard.putNumber("VisionPose/Timestamp", 0);
  // }
}


  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
    visionSim.update(robotPoseMeters);
    visionSim.getDebugField();
  }
}
