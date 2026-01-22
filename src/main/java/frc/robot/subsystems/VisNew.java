// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.function.Supplier;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.math.Matrix;

public class VisNew extends SubsystemBase {
  /** Creates a new ExampleSubsystem. */
  public static final AprilTagFieldLayout fieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltAndymark);
  Supplier<Pose2d> currentPose;
  Field2d field;
  VisionSystemSim visionSim;

  public VisNew(Supplier<Pose2d> currentPose, Field2d field) {
    this.currentPose = currentPose;
    this.field = field;

    if (Robot.isSimulation())
    {
      visionSim = new VisionSystemSim("Vision");
      visionSim.addAprilTags(fieldLayout);

      for (Cameras c : Cameras.values())
      {
        c.addToVisionSim(visionSim);
      }
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

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
  enum Cameras
  {
    /**
     * Left Camera
     */
    LEFT_CAM("left",
             new Rotation3d(0, Math.toRadians(-24.094), Math.toRadians(30)),
             new Translation3d(Units.inchesToMeters(12.056),
                               Units.inchesToMeters(10.981),
                               Units.inchesToMeters(8.44)),
             VecBuilder.fill(4, 4, 8), VecBuilder.fill(0.5, 0.5, 1)),
    /**
     * Right Camera
     */
    RIGHT_CAM("right",
              new Rotation3d(0, Math.toRadians(-24.094), Math.toRadians(-30)),
              new Translation3d(Units.inchesToMeters(12.056),
                                Units.inchesToMeters(-10.981),
                                Units.inchesToMeters(8.44)),
              VecBuilder.fill(4, 4, 8), VecBuilder.fill(0.5, 0.5, 1)),
    /**
     * Center Camera
     */
    CENTER_CAM("center",
               new Rotation3d(0, Units.degreesToRadians(18), 0),
               new Translation3d(Units.inchesToMeters(-4.628),
                                 Units.inchesToMeters(-10.687),
                                 Units.inchesToMeters(16.129)),
               VecBuilder.fill(4, 4, 8), VecBuilder.fill(0.5, 0.5, 1));

    //enum vars
    public final PhotonCamera camera;
    public final Alert latencyAlert;
    private final Transform3d robotToCamTransform;
    public final PhotonPoseEstimator poseEstimator;
    private final Matrix<N3,N1> singleTagStdDevs;
    private final Matrix<N3, N1> multiTagStdDevs;
    public Matrix<N3, N1> curStdDevs;
    public PhotonCameraSim cameraSim;

    Cameras(String name, Rotation3d robotToCamRotation, Translation3d robotToCamTranslation,
            Matrix<N3, N1> singleTagStdDevs, Matrix<N3, N1> multiTagStdDevsMatrix)
    {

      latencyAlert = new Alert("'" + name + "' Camera is experiencing high latency.", AlertType.kWarning);

      camera = new PhotonCamera(name);

      // https://docs.wpilib.org/en/stable/docs/software/basic-programming/coordinate-system.html
      robotToCamTransform = new Transform3d(robotToCamTranslation, robotToCamRotation);

      poseEstimator = new PhotonPoseEstimator(VisNew.fieldLayout,
                                              PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
                                              robotToCamTransform);
      poseEstimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);

      this.singleTagStdDevs = singleTagStdDevs;
      this.multiTagStdDevs = multiTagStdDevsMatrix;

      if (Robot.isSimulation())
      {
        SimCameraProperties cameraProp = new SimCameraProperties();
        // A 640 x 480 camera with a 100 degree diagonal FOV.
        cameraProp.setCalibration(960, 720, Rotation2d.fromDegrees(100));
        // Approximate detection noise with average and standard deviation error in pixels.
        cameraProp.setCalibError(0.25, 0.08);
        // Set the camera image capture framerate (Note: this is limited by robot loop rate).
        cameraProp.setFPS(30);
        // The average and standard deviation in milliseconds of image data latency.
        cameraProp.setAvgLatencyMs(35);
        cameraProp.setLatencyStdDevMs(5);

        cameraSim = new PhotonCameraSim(camera, cameraProp);
        cameraSim.enableDrawWireframe(true);
      }
    } 
    public void addToVisionSim(VisionSystemSim systemSim)
    {
      if (Robot.isSimulation())
      {
        systemSim.addCamera(cameraSim, robotToCamTransform);
      }
    }  
  }
}
