package org.firstinspires.ftc.teamcode;

public final class Settings {
    private Settings() {
        throw new java.lang.UnsupportedOperationException(
                "This is a settings class and cannot be instantiated"
        );
    }
    public static final double DEADZONE_THRESHOLD = 0.1;

    public static final double armInPosition = 0.658;
    public static final double armOutPosition = 0.188;

    public static final int linearSlideStartingPosition = 0;
    public static final int linearSlideFirstBucketPosition = 2500;
    public static final int linearSlideSecondBucketPosition = 4250;
    public static final int[] linearSlidePositions = {
            linearSlideStartingPosition,
            linearSlideFirstBucketPosition,
            linearSlideSecondBucketPosition
    };

    public static final double clawOpenPosition = .5;
    public static final double clawClosedPosition = .68;

    public static final double wristIntakePosition = 0.8;
    public static final double wristReleasePosition = 0.21;
    public static final double wristDrivingPosition = 0.38;
    public static final double[] wristPositions = {
            wristIntakePosition,
            wristDrivingPosition,
            wristReleasePosition
    };

    public static final double bucketPickupPosition = 0.514;
    public static final double bucketReleasePosition = 0.48;

    public static final double motorCPR = 1;
}
