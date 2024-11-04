package org.firstinspires.ftc.teamcode;

public final class Settings {
    private Settings() {
        throw new java.lang.UnsupportedOperationException(
                "This is a settings class and cannot be instantiated"
        );
    }
    public static final double DEADZONE_THRESHOLD = 0.1;

    public static final double ARM_IN_POSITION = 0.658;
    public static final double ARM_OUT_POSITION = 0.188;

    public static final int LINEAR_SLIDE_STARTING_POSITION = 0;
    public static final int LINEAR_SLIDE_FIRST_BUCKET_POSITION = 2700;
    public static final int LINEAR_SLIDE_SECOND_BUCKET_POSITION = 4220;
    public static final int[] LINEAR_SLIDE_POSITIONS = {
            LINEAR_SLIDE_STARTING_POSITION,
            LINEAR_SLIDE_FIRST_BUCKET_POSITION,
            LINEAR_SLIDE_SECOND_BUCKET_POSITION
    };

    public static final double CLAW_OPEN_POSITION = .5;
    public static final double CLAW_CLOSED_POSITION = .68;

    public static final double WRIST_INTAKE_POSITION = 0.8;
    public static final double WRIST_RELEASE_POSITION = 0.21;
    public static final double WRIST_DRIVING_POSITION = 0.38;
    public static final double[] WRIST_POSITIONS = {
            WRIST_INTAKE_POSITION,
            WRIST_DRIVING_POSITION,
            WRIST_RELEASE_POSITION
    };

    public static final double BUCKET_PICKUP_POSITION = 0.514;
    public static final double BUCKET_RELEASE_POSITION = 0.48;

    public static final double LINEAR_SLIDE_POWER = 0.66;
    public static final double MAX_INTAKE_POWER = 0.33;

    public static final double kP = 0.5;
}
