package org.firstinspires.ftc.teamcode;

import com.sun.tools.javac.util.List;


public final class Settings {
    private Settings() {
        throw new UnsupportedOperationException("This is a settings class and cannot be instantiated");
    }

    // Deadzone threshold for joystick or controller inputs
    public static final double DEADZONE_THRESHOLD = 0.1;

    public static final class Trolley {
        public static final double IN_POSITION = 0.658;
        public static final double OUT_POSITION = 0.188;
    }


    public static final class LinearSlide {
        public static final double POWER = 0.66;

        public static final int STARTING_POSITION = 0;
        public static final int FIRST_BUCKET_POSITION = 2700;
        public static final int SECOND_BUCKET_POSITION = 4220;

        // Immutable list of position presets
        public static final List<Integer> POSITIONS = List.of(
                STARTING_POSITION,
                FIRST_BUCKET_POSITION,
                SECOND_BUCKET_POSITION
        );
    }


    public static final class Claw {
        public static final double OPEN_POSITION = 0.5;
        public static final double CLOSED_POSITION = 0.68;
    }


    public static final class Wrist {
        public static final double INTAKE_POSITION = 0.8;
        public static final double RELEASE_POSITION = 0.21;
        public static final double DRIVING_POSITION = 0.38;

        // Immutable list of wrist positions
        public static final List<Double> POSITIONS = List.of(
                INTAKE_POSITION,
                DRIVING_POSITION,
                RELEASE_POSITION
        );
    }


    public static final class Bucket {
        public static final double PICKUP_POSITION = 0.514;
        public static final double RELEASE_POSITION = 0.48;
    }


    public static final class Intake {
        public static final double MAX_POWER = 0.33;
    }
}
