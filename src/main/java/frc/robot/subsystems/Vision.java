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
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;

public class Vision extends SubsystemBase {
  /** Creates a new ExampleSubsystem. */

  private EstimateConsumer estConsumer; //accepts pose
  private final Supplier<Pose2d> poseProvider; //gives pose

  private PhotonCamera camera_left;
  private final String CAMERA_LEFT_NAME = "color";
  private final AprilTagFieldLayout kTagFieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltAndymark);
  private PhotonPoseEstimator photonEstimator;
  public static final Transform3d kRobotToCam =
                new Transform3d(new Translation3d(0.2921, 0.0, 0.2667), new Rotation3d(0, 0, 0));

  private Matrix<N3,N1> curStdDevs;
  
StructPublisher<Pose3d> publisher = NetworkTableInstance.getDefault()
  .getStructTopic("VisionPose", Pose3d.struct).publish();
  


  
  // The standard deviations of our vision estimated poses, which affect correction rate
  // (Fake values. Experiment and determine estimation noise on an actual robot.)
  public static final Matrix<N3, N1> kSingleTagStdDevs = VecBuilder.fill(1, 1, 2);
  public static final Matrix<N3, N1> kMultiTagStdDevs = VecBuilder.fill(0.5, 0.5, 1);

  private PhotonCameraSim cameraSim;
  private VisionSystemSim visionSim;

  public Vision(EstimateConsumer estConsumer, Supplier<Pose2d> poseProvider) {
    this.poseProvider = poseProvider;
    this.estConsumer = estConsumer;
    this.camera_left = new PhotonCamera(CAMERA_LEFT_NAME);
    photonEstimator = new PhotonPoseEstimator(kTagFieldLayout, kRobotToCam);
    if (Robot.isSimulation()) {
          visionSim = new VisionSystemSim("main");
          visionSim.addAprilTags(kTagFieldLayout);
          // Create simulated camera properties. These can be set to mimic your actual camera.
          var cameraProp = new SimCameraProperties();
          cameraProp.setCalibError(0, 0); 
          cameraProp.setAvgLatencyMs(0);
          cameraProp.setLatencyStdDevMs(0);
          cameraProp.setCalibration(960, 720, Rotation2d.fromDegrees(90));
          // cameraProp.setCalibError(0.35, 0.10);
          cameraProp.setFPS(15);
          // cameraProp.setAvgLatencyMs(50);
          // cameraProp.setLatencyStdDevMs(15);
          // Create a PhotonCameraSim which will update the linked PhotonCamera's values with visible
          // targets.
          cameraSim = new PhotonCameraSim(camera_left, cameraProp);
          // Add the simulated camera to view the targets on this simulated field.
          visionSim.addCamera(cameraSim, kRobotToCam);

          cameraSim.enableDrawWireframe(true);
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

public int getTagId(boolean isRed) {
    // Use getLatestResult for the most current data
    var result = camera_left.getLatestResult();
    
    if (!result.hasTargets()) {
        return 0;
    }
    PhotonTrackedTarget bestTarget = null;
    
    for (var target : result.getTargets()) {
        if ((target.getFiducialId() <= 16 && isRed) || (target.getFiducialId() > 16 && !isRed)){ //if the tag is on our alliance side
          if (bestTarget == null) {
              bestTarget = target;
          } 
      //We want LESS ambiguity (<), not more (>)
          else if (target.getPoseAmbiguity() < bestTarget.getPoseAmbiguity()) {
              bestTarget = target;
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

  @Override
  public void periodic() {
    Optional<EstimatedRobotPose> visionEst = Optional.empty(); //sets visionEstimator var
    for (var result: camera_left.getAllUnreadResults()) { //for each new apriltag result
      visionEst = photonEstimator.estimateCoprocMultiTagPose(result);
      if (visionEst.isEmpty()) {
        visionEst = photonEstimator.estimateLowestAmbiguityPose(result); //change pipeline estimation to find one with least uncertainty
      }
      updateEstimationStdDevs(visionEst, result.getTargets());
      
      if (Robot.isSimulation()) {
          visionEst.ifPresentOrElse(
                  est ->
                          getSimDebugField()
                                  .getObject("VisionEstimation")
                                  .setPose(est.estimatedPose.toPose2d()),
                  () -> {
                      getSimDebugField().getObject("VisionEstimation").setPoses();
                  });
      }
       visionEst.ifPresent(
                    est -> {
                      publisher.set(est.estimatedPose);
                        // Change our trust in the measurement based on the tags we can see
                        var estStdDevs = getEstimationStdDevs();
                        estConsumer.accept(est.estimatedPose.toPose2d(), est.timestampSeconds, estStdDevs);
                    });
    }
    
    // This method will be called once per scheduler run
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
        var tagPose = photonEstimator.getFieldTags().getTagPose(tgt.getFiducialId());
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
        if (numTags == 1 && avgDist > 4)
            estStdDevs = VecBuilder.fill(Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE);
        else estStdDevs = estStdDevs.times(1 + (avgDist * avgDist / 30));
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
