package org.firstinspires.ftc.teamcode;

import com.sun.tools.javac.util.List;

// many variables are not final so we can change them in FTC Dashboard
public class Settings {
    private Settings() {
        throw new UnsupportedOperationException("This is a settings class and cannot be instantiated");
    }

    // Deadzone threshold for joystick or controller inputs
    public static double DEADZONE_THRESHOLD = 0.1;

    public static class Trolley {
        public static double IN_POSITION = 0.75;
        public static double OUT_POSITION = 0.456;
    }


    public static class LinearSlide {
        public static double POWER = 1.0;
        public static double FINE_CONTROL_POWER = 0.33;

        public static int STARTING_POSITION = 50;
        public static int FIRST_BUCKET_POSITION = 2700;
        public static int SECOND_BUCKET_POSITION = 4100;

        public static int SPECIMEN_LOWERED_POSITION = 1375;
        public static int SPECIMEN_APPROACH_POSITION = 1800;

        // Immutable list of position presets
        public static final List<Integer> POSITIONS = List.of(
                STARTING_POSITION,
                SPECIMEN_APPROACH_POSITION,
                FIRST_BUCKET_POSITION,
                SECOND_BUCKET_POSITION
        );
    }


    public static class Claw {
        public static double OPEN_POSITION = 0.53;
        public static double CLOSED_POSITION = 0.68;
    }


    public static class Wrist {
        public static double INTAKE_POSITION = 0.08;
        public static double RELEASE_POSITION = 0.8;
        public static double DRIVING_POSITION = 0.5;

        // Immutable list of wrist positions
        public static final List<Double> POSITIONS = List.of(
                INTAKE_POSITION,
                DRIVING_POSITION,
                RELEASE_POSITION
        );
    }


    public static class Bucket {
        public static double PICKUP_POSITION = 0.485;
        public static double RELEASE_POSITION = 0.53;
    }


    public static class Intake {
        public static double MAX_POWER = 0.5;
    }


    public static class Odometry {
        public static final double WHEEL_TICKS_PER_REVOLUTION = 537.6;
        public static final double WHEEL_DIAMETER_IN = 3.77952;
        public static final double WHEEL_IN_PER_TICK = (WHEEL_DIAMETER_IN * Math.PI) / WHEEL_TICKS_PER_REVOLUTION;

        public static final double ODOMETRY_WHEEL_TICKS_PER_REVOLUTION = 2000;
        public static final double ODOMETRY_WHEEL_DIAMETER_IN = 1.88976;
        public static final double ODOMETRY_WHEEL_IN_PER_TICK = (ODOMETRY_WHEEL_DIAMETER_IN * Math.PI) / ODOMETRY_WHEEL_TICKS_PER_REVOLUTION;
    }

    public static class Autonomous {
        public static int DEFAULT_DRIVE_TIMEOUT_MS = 3000;
        public static double DEFAULT_DRIVE_MAX_POWER = 0.5;
    }

        /**
         * PID constants for the drive and turn methods:
         * - kP: Proportional constant (robot will slow down the closer it gets to the target)
         * - kI: Integral constant (robot will correct for small errors over time)
         * - kD: Derivative constant (robot will slow down as it approaches the target)
         * - ELIPSON: The error threshold for the robot to stop moving
         */

    public static class DrivePID {
        // critical gain = .2
        // oscillation period = .76
        public static double kP = 0.084;
        public static double kI = 0.252631579;
        public static double kD = 0.057;
        public static double[] COEFFICIENTS = {kP, kI, kD};
        public static double TOLERANCE = 1;
        public static double TIME_TO_SETTLE = 0.5;
    }

    public static class StrafePID {
        public static double kP = 0.084;
        public static double kI = 0.252631579;
        public static double kD = 0.057;
        public static double[] COEFFICIENTS = {kP, kI, kD};
        public static double TOLERANCE = 1;
        public static double TIME_TO_SETTLE = 0.5;
    }

    // all turning done in degrees
    public static class TurnPID {
        public static double kP = 1.55;
        public static double kI = 0.5;
        public static double kD = 1.0;
        public static double[] COEFFICIENTS = {kP, kI, kD};
        public static double TOLERANCE = 2;
        public static double TIME_TO_SETTLE = 0.5;

    }
}
