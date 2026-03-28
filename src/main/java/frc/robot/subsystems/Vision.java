/*
 * MIT License
 *
 * Copyright (c) PhotonVision
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

package frc.robot.subsystems;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;

import static frc.robot.constants.VisionConstants.Vision.*;

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

public class Vision extends SubsystemBase{
    private final PhotonCamera camera;
    private final PhotonPoseEstimator photonEstimator;

    private final PhotonCamera camera2;
    private final PhotonPoseEstimator photonEstimator2;

    private final PhotonCamera camera3;
    private final PhotonPoseEstimator photonEstimator3;


    private Matrix<N3, N1> curStdDevs;

    private final EstimateConsumer estConsumer;
    private final Supplier<Pose2d> poseSupplier;

    private PhotonCamera[] cameras;
    private PhotonPoseEstimator[] estimators;

    private List<PhotonPipelineResult> latestResult;

    // Simulation
    private PhotonCameraSim cameraSim;
    private VisionSystemSim visionSim;

    private final Field2d visionField = new Field2d();

    /**
     * @param estConsumer Lamba that will accept a pose estimate and pass it to your desired {@link
     *     edu.wpi.first.math.estimator.SwerveDrivePoseEstimator}
     */
    public Vision(EstimateConsumer estConsumer,  Supplier<Pose2d> poseSupplier) { //pose supplier for vision sim
        this.estConsumer = estConsumer;
        this.poseSupplier = poseSupplier;
        camera = new PhotonCamera(kCameraName);
        photonEstimator = new PhotonPoseEstimator(kTagLayout, kRobotToCam);

        this.camera2 = new PhotonCamera(kCameraName2);
        this.photonEstimator2 = new PhotonPoseEstimator(kTagLayout, kRobotToCam2);

        this.camera3 = new PhotonCamera(kCameraName3);
        this.photonEstimator3 = new PhotonPoseEstimator(kTagLayout, kRobotToCam3);


        cameras = new PhotonCamera[] {camera, camera2, camera3};
        estimators = new PhotonPoseEstimator[] {photonEstimator, photonEstimator2, photonEstimator3};

        SmartDashboard.putData("Vision Field", visionField);

        // ----- Simulation
        if (Robot.isSimulation()) {
            // Create the vision system simulation which handles cameras and targets on the field.
            visionSim = new VisionSystemSim("main");
            // Add all the AprilTags inside the tag layout as visible targets to this simulated field.
            visionSim.addAprilTags(kTagLayout);
            for (int i = 0; i < cameras.length; i++) {
                PhotonCamera cam = cameras[i];

                var cameraProp = new SimCameraProperties();
                cameraProp.setCalibration(640, 480, Rotation2d.fromDegrees(90));
                // cameraProp.setCalibError(0.35, 0.10);
                cameraProp.setFPS(30);
                cameraProp.setAvgLatencyMs(50);
                cameraProp.setLatencyStdDevMs(15);

                cameraSim = new PhotonCameraSim(cam, cameraProp);

                var transform = (i == 0) ? kRobotToCam : kRobotToCam2;
                visionSim.addCamera(cameraSim, transform);

                cameraSim.enableDrawWireframe(true);
            }
        }
    }


    @Override
    public void periodic() {
        for (int i = 0; i < cameras.length; i++) {
            PhotonCamera cam = cameras[i];
            PhotonPoseEstimator estimator = estimators[i];
            Optional<EstimatedRobotPose> visionEst = Optional.empty();
            latestResult = cam.getAllUnreadResults();
            for (var result : latestResult) {
                visionEst = estimator.estimateCoprocMultiTagPose(result);
                if (visionEst.isEmpty()) {
                    visionEst = estimator.estimateLowestAmbiguityPose(result);
                }
                updateEstimationStdDevs(visionEst, result.getTargets());
                            // --- POSE DATA ---


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
                            // Change our trust in the measurement based on the tags we can see
                            var estStdDevs = getEstimationStdDevs();
                            Pose2d pose = est.estimatedPose.toPose2d();
                            visionField.setRobotPose(pose);
                            estConsumer.accept(pose, est.timestampSeconds, estStdDevs);
                        });
            }
        }
    }
public Pose2d getTagPose(int tagId) {
    if (tagId <= 0) return null;
    
    var tagOpt = kTagLayout.getTagPose(tagId);
    if (tagOpt.isPresent()) {
        return tagOpt.get().toPose2d();
    }
    return null;
}
public int getTagId() {
    PhotonTrackedTarget bestTarget = null;

    for (var result : latestResult) {
        if (!result.hasTargets()) continue;
        for (var target : result.getTargets()) {
            if (bestTarget == null || target.getPoseAmbiguity() < bestTarget.getPoseAmbiguity()) {
                bestTarget = target;
            }
        }
    }
    if (bestTarget == null) return 0;
    return bestTarget.getFiducialId();
}

    /**
     * Calculates new standard deviations This algorithm is a heuristic that creates dynamic standard
     * deviations based on number of tags, estimation strategy, and distance from the tags.
     *
     * @param estimatedPose The estimated pose to guess standard deviations for.
     * @param targets All targets in this camera frame
     */
    private void updateEstimationStdDevs(
            Optional<EstimatedRobotPose> estimatedPose, List<PhotonTrackedTarget> targets) {
        if (estimatedPose.isEmpty()) {
            // No pose input. Default to single-tag std devs
            curStdDevs = kSingleTagStdDevs;

        } else {
            // Pose present. Start running Heuristic
            var estStdDevs = kSingleTagStdDevs;
            int numTags = 0;
            double avgDist = 0;

            // Precalculation - see how many tags we found, and calculate an average-distance metric
            for (var tgt : targets) {
                var tagPose = photonEstimator.getFieldTags().getTagPose(tgt.getFiducialId());
                if (tagPose.isEmpty()) continue;
                numTags++;
                avgDist +=
                        tagPose
                                .get()
                                .toPose2d()
                                .getTranslation()
                                .getDistance(estimatedPose.get().estimatedPose.toPose2d().getTranslation());
            }

            if (numTags == 0) {
                // No tags visible. Default to single-tag std devs
                curStdDevs = kSingleTagStdDevs;
            } else {
                // One or more tags visible, run the full heuristic.
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

    // ----- Simulation

    public void simulationPeriodic() {
        visionSim.update(poseSupplier.get());
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

    // public bool detectBall(){
    //     camera.
    // }
}