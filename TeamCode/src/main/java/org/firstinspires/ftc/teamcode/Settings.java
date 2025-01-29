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
        public static double IN_POSITION = 1.0;
        public static double OUT_POSITION = 0.55;
    }


    public static class LinearSlide {
        public static double POWER = 1.0;
        public static double FINE_CONTROL_POWER = 0.33;

        public static int STARTING_POSITION = 50;
        public static int FIRST_BUCKET_POSITION = 2700;
        public static int SECOND_BUCKET_POSITION = 4100;

        public static int SPECIMEN_LOWERED_POSITION = 1250;
        public static int SPECIMEN_APPROACH_POSITION = 1700;

        // Immutable list of position presets
        public static final List<Integer> POSITIONS = List.of(
                STARTING_POSITION,
                SPECIMEN_APPROACH_POSITION,
                FIRST_BUCKET_POSITION,
                SECOND_BUCKET_POSITION
        );
    }


    public static class Claw {
        public static double OPEN_POSITION = 1;
        public static double CLOSED_POSITION = 0;
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
        public static double PICKUP_POSITION = 0.514;
        public static double RELEASE_POSITION = 0.48;
    }


    public static class Intake {
        public static double MAX_POWER = 0.5;
    }


    public static class Autonomous {
        public static final double TICKS_PER_REVOLUTION = 537.6;
        public static final double WHEEL_DIAMETER_IN = 3.77952;
        public static final double TICKS_PER_IN = TICKS_PER_REVOLUTION / (WHEEL_DIAMETER_IN * Math.PI);

        public static int DEFAULT_DRIVE_TIMEOUT_MS = 3000;
        public static double DEFAULT_DRIVE_MAX_POWER = 0.5;
        public static double DEFAULT_DRIVE_MIN_POWER = 0.1;

        public static int DEFAULT_TURN_TIMEOUT_MS = 2000;
        public static double DEFAULT_TURN_MAX_POWER = 0.5;
        public static double DEFAULT_TURN_MIN_POWER = 0.1;

        /**
         * PID constants for the drive and turn methods:
         * - kP: Proportional constant (robot will slow down the closer it gets to the target)
         * - kI: Integral constant (robot will correct for small errors over time)
         * - kD: Derivative constant (robot will slow down as it approaches the target)
         * - ELIPSON: The error threshold for the robot to stop moving
         */

        public static class DrivePID {
            public static double kP = 1.5;

            public static double ELIPSON = 3;

        }

        public static class TurnPID {
            public static double kP = 1.5;

            public static double ELIPSON = 3;
        }
    }
}
