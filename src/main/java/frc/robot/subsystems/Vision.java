// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;
import java.util.List;
import java.util.Optional;
import java.util.function.Supplier;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;

public class Vision extends SubsystemBase {
  /** Creates a new ExampleSubsystem. */


    // Inner class to represent each camera
  private class VisionCamera {
    public final PhotonCamera camera;
    public final String cameraName;
    public final Transform3d robotToCamera;
    public final PhotonPoseEstimator poseEstimator;
    public PhotonCameraSim cameraSim;
    
    public VisionCamera(String name, Transform3d transform) {
      this.cameraName = name;
      this.camera = new PhotonCamera(name);
      this.robotToCamera = transform;
      this.poseEstimator = new PhotonPoseEstimator(kTagFieldLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, transform);
    }
  }
  
  private VisionCamera[] cameras;

  private EstimateConsumer estConsumer; //accepts pose
  private final Supplier<Pose2d> poseProvider; //gives pose

  private final AprilTagFieldLayout kTagFieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltAndymark);
  public static final Transform3d kRobotToCam =
                new Transform3d(new Translation3d(0.2921, 0.0, 0.2667), new Rotation3d(0, 0, 0));

  private Matrix<N3,N1> curStdDevs;
  
  StructPublisher<Pose3d> publisher = NetworkTableInstance.getDefault()
    .getStructTopic("VisionPose", Pose3d.struct).publish();
  


  
  // The standard deviations of our vision estimated poses, which affect correction rate
  // (Fake values. Experiment and determine estimation noise on an actual robot.)
  public static final Matrix<N3, N1> kSingleTagStdDevs = VecBuilder.fill(0.5, 0.5, 1);
  public static final Matrix<N3, N1> kMultiTagStdDevs = VecBuilder.fill(0.1, 0.1, 0.15);

  private VisionSystemSim visionSim;

  public Vision(EstimateConsumer estConsumer, Supplier<Pose2d> poseProvider) {
    this.poseProvider = poseProvider;
    this.estConsumer = estConsumer;
        // Initialize cameras with their transforms relative to robot center
    // Camera 1: Left side, forward facing
    Transform3d leftCameraTransform = new Transform3d(
      new Translation3d(0.2921, 0.0, 0.2667), 
      new Rotation3d(0, 0, 0)
    );
    
    // Camera 2: Right side (example - adjust based on your actual camera position)
    Transform3d rightCameraTransform = new Transform3d(
      new Translation3d(0.2921, -0.3, 0.2667), // Example offset: 30cm to the right
      new Rotation3d(0, 0, 0)
    );
    
    cameras = new VisionCamera[] {
      new VisionCamera("color", leftCameraTransform),      // Your existing camera
      // new VisionCamera("color_right", rightCameraTransform) // New camera (adjust name/position)
    };
    if (Robot.isSimulation()) {
          setupSimulation();
    }
  }
  private void setupSimulation() {
    visionSim = new VisionSystemSim("main");
    visionSim.addAprilTags(kTagFieldLayout);
    
    for (VisionCamera visionCam : cameras) {
      var cameraProp = new SimCameraProperties();
      cameraProp.setCalibError(0, 0);
      cameraProp.setAvgLatencyMs(0);
      cameraProp.setLatencyStdDevMs(0);
      cameraProp.setCalibration(640, 480, Rotation2d.fromDegrees(90));
      cameraProp.setFPS(15);
      
      visionCam.cameraSim = new PhotonCameraSim(visionCam.camera, cameraProp);
      visionSim.addCamera(visionCam.cameraSim, visionCam.robotToCamera);
      visionCam.cameraSim.enableDrawWireframe(true);
    }
  }

public Pose2d getTagPose(int tagId) {
  // Return null if invalid, so we can check for it later
  if(tagId <= 0) return null; 
  
  var tagOpt = kTagFieldLayout.getTagPose(tagId);
  if (tagOpt.isPresent()) {
      return tagOpt.get().toPose2d();
  }
  return null;
}
  private void processCamera(VisionCamera visionCam) {
    Optional<EstimatedRobotPose> visionEst = Optional.empty();
    
    for (var result : visionCam.camera.getAllUnreadResults()) {
      visionEst = visionCam.poseEstimator.estimateCoprocMultiTagPose(result);
      if (visionEst.isEmpty()) {
        visionEst = visionCam.poseEstimator.estimateLowestAmbiguityPose(result);
      }
      
      updateEstimationStdDevs(visionEst, result.getTargets());
      
      if (Robot.isSimulation()) {
        visionEst.ifPresentOrElse(
          est -> getSimDebugField()
            .getObject("VisionEstimation_" + visionCam.cameraName)
            .setPose(est.estimatedPose.toPose2d()),
          () -> getSimDebugField().getObject("VisionEstimation_" + visionCam.cameraName).setPoses()
        );
      }
      
      visionEst.ifPresent(est -> {
        publisher.set(est.estimatedPose);
        var estStdDevs = getEstimationStdDevs();
        estConsumer.accept(est.estimatedPose.toPose2d(), est.timestampSeconds, estStdDevs);
      });
    }
  }

  public int getTagId(boolean isRed) {
    PhotonTrackedTarget bestTarget = null;
    VisionCamera bestCamera = null;
    
    // Check all cameras for the best target
    for (VisionCamera visionCam : cameras) {
      var result = visionCam.camera.getLatestResult();
      
      if (!result.hasTargets()) continue;
      
      for (var target : result.getTargets()) {
        // Filter for alliance side tags
        if ((target.getFiducialId() <= 16 && isRed) || (target.getFiducialId() > 16 && !isRed)) {
          if (bestTarget == null || target.getPoseAmbiguity() < bestTarget.getPoseAmbiguity()) {
            bestTarget = target;
            bestCamera = visionCam;
          }
        }
      }
    }
    
    if (bestTarget == null) return 0;
    return bestTarget.getFiducialId();
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
  public Optional<Pose2d> getEstimatedRobotPose() {
    for (VisionCamera visionCam : cameras) {
      var result = visionCam.camera.getLatestResult();
      if (!result.hasTargets()) continue;

      var estimate = visionCam.poseEstimator.estimateCoprocMultiTagPose(result);
      if (estimate.isEmpty()) {
          estimate = visionCam.poseEstimator.estimateLowestAmbiguityPose(result);
      }

      if (estimate.isPresent()) {
          return Optional.of(estimate.get().estimatedPose.toPose2d());
      }
    }
    return Optional.empty();
  }

  @Override
  public void periodic() {
    for (VisionCamera visionCam : cameras) {
      processCamera(visionCam);
    }
    var poseOpt = getEstimatedRobotPose();
    if (poseOpt.isPresent()) {
      Pose2d pose = poseOpt.get();
      SmartDashboard.putNumber("Vision X", pose.getX());
      SmartDashboard.putNumber("Vision Y", pose.getY());
      SmartDashboard.putNumber("Vision Rotation", pose.getRotation().getDegrees());
    } else {
      SmartDashboard.putString("Vision Status", "No Pose");
    }
  }
      /**
     * Calculates new standard deviations This algorithm is a heuristic that creates dynamic standard
     * deviations based on number of tags, estimation strategy, and distance from the tags.
     *
     * @param estimatedPose The estimated pose to guess standard deviations for.
     * @param targets All targets in this camera frame
     */
  private void updateEstimationStdDevs(Optional<EstimatedRobotPose> estimatedPose, List<PhotonTrackedTarget> targets) {
    if (estimatedPose.isEmpty()){
      //no pose input. default to single-tag dev
      curStdDevs = kSingleTagStdDevs;
    } else { //pose present
      double avgDist = 0;
      var estStdDevs = kSingleTagStdDevs;
      int numTags = 0;

      //Precalculation: see how many tags we found and calculate an average distance metric
      for (var tgt : targets) {
        var tagPose = kTagFieldLayout.getTagPose(tgt.getFiducialId());
        if (tagPose.isEmpty()) continue;
        numTags++;
        avgDist += tagPose.get().toPose2d().getTranslation().getDistance(estimatedPose.get().estimatedPose.toPose2d().getTranslation());
      }
      if (numTags == 0) {
        //no tags visible default to single tag std dev
        curStdDevs = kSingleTagStdDevs;
      } else {
        //one or more tags available, run the alg
        avgDist /= numTags;
                // Decrease std devs if multiple targets are visible
        if (numTags > 1) estStdDevs = kMultiTagStdDevs;
        // Increase std devs based on (average) distance
        estStdDevs = estStdDevs.times(1 + (avgDist * avgDist / 30));
        curStdDevs = estStdDevs;
      }
    }
  }
     /**
     * Returns the latest standard deviations of the estimated pose from {@link
     * #getEstimatedGlobalPose()}, for use with {@link
     * edu.wpi.first.math.estimator.SwerveDrivePoseEstimator SwerveDrivePoseEstimator}. This should
     * only be used when there are targets visible.
     */
    public Matrix<N3, N1> getEstimationStdDevs() {
        return curStdDevs;
    }

    //SIMULATION

    public void simulationPeriodic() {
        Pose2d currentSimPose = poseProvider.get();
        visionSim.update(currentSimPose);

    }

    /** Reset pose history of the robot in the vision system simulation. */
    public void resetSimPose(Pose2d pose) {
        if (Robot.isSimulation()) visionSim.resetRobotPose(pose);
    }

    /** A Field2d for visualizing our robot and objects on the field. */
    public Field2d getSimDebugField() {
        if (!Robot.isSimulation()) return null;
        return visionSim.getDebugField();
    }

  @FunctionalInterface
  public static interface EstimateConsumer {
      public void accept(Pose2d pose, double timestamp, Matrix<N3, N1> estimationStdDevs);
  }
}
