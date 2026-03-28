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

package frc.robot.constants;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;

public class VisionConstants {
    public static class Vision {
        public static final String kCameraName = "AprilTagCamera1"; //left camera
        // Cam mounted facing forward, half a meter forward of center, half a meter up from center.
        public static final Transform3d kRobotToCam =
            new Transform3d(
                new Translation3d(
                    Units.inchesToMeters(-8.75),     // forward
                    Units.inchesToMeters(-10.5),     // left
                    Units.inchesToMeters(20.5)     // up
                ),
                new Rotation3d(
                    0,                             // roll
                    0,  // pitch downward (adjust as needed)
                    Units.degreesToRadians(195)                           // yaw
                )
            );

        public static final String kCameraName2 = "AprilTagCamera2"; //right camera
        public static final Transform3d kRobotToCam2 = 
            new Transform3d(
                new Translation3d(
                    Units.inchesToMeters(-8.75),
                    Units.inchesToMeters(5),
                    Units.inchesToMeters(20.5)
                ),
                new Rotation3d(
                    0,
                    0,
                    Units.degreesToRadians(165)
                )
            );
        public static final String kCameraName3 = "AprilTagCamera3"; //front camera
        public static final Transform3d kRobotToCam3 = 
            new Transform3d(
                new Translation3d(
                    Units.inchesToMeters(12),
                    Units.inchesToMeters(12),
                    Units.inchesToMeters(0)
                ),
                new Rotation3d(
                    0,
                    Units.degreesToRadians(40),
                    Units.degreesToRadians(45)
                )
            );



        // The layout of the AprilTags on the field
        public static final AprilTagFieldLayout kTagLayout =
                AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);

        // The standard deviations of our vision estimated poses, which affect correction rate
        // (Fake values. Experiment and determine estimation noise on an actual robot.)
        public static final Matrix<N3, N1> kSingleTagStdDevs = VecBuilder.fill(4, 4, 8);
        public static final Matrix<N3, N1> kMultiTagStdDevs = VecBuilder.fill(0.5, 0.5, 1);
    }
}