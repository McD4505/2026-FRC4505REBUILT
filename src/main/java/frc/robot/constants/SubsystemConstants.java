package frc.robot.constants;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;

import com.pathplanner.lib.config.ModuleConfig;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.system.plant.DCMotor;
import java.util.Map;

public class SubsystemConstants {

    //NEED TO CHANGE VALUES
    // -------What it does--------
    // Mapping distance to rpm so use real robot testing to find rpm and distance
    public static final InterpolatingDoubleTreeMap SHOOTER_VELOCITY_LOOKUP = InterpolatingDoubleTreeMap.ofEntries(
    Map.entry(2.86, 42.31), Map.entry(2.30, 38.40), Map.entry(3.3, 46.08), Map.entry(3.96, 51.0),
    Map.entry(5.09, 70.0));

    // ------ SHOOOTING CONSTANTS ------
    public static final int INDEXER_SHOOT_RPS = 100;
    public static final double PASS_OFFSET = 10;
    public static final int INTAKE_SHOOT_RPS = 100;

    public static class DrivetrainConstants {
        public static final double WHEEL_COF = 1.1;
        public static final double MAX_DRIVE_SPEED = 10; // TODO: this needs to be tested
        public static final double MAX_ROTATIONAL_SPEED = 5; // TODO: this needs to be tested

        public static final double DRIVE_P = 4.0;
        public static final double DRIVE_I = 0.0;
        public static final double DRIVE_D = 0.01;

        public static final double ROTATION_P = 10.0;
        public static final double ROTATION_I = 0.0;
        public static final double ROTATION_D = 0.0;
  }
}

