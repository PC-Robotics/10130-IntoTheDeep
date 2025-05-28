package org.firstinspires.ftc.teamcode.jake;


public class OdometryConstants {
    public static final double ODOM_INCHES_PER_COUNT = 0.001978992598;    // GoBilda Odometry Pod       (old number 0.002969)
    public static final double FLOOR_TILE_INCHES = 24;

    //TODO: Check that these booleans are correct using simple teleop process with output

    public static final boolean INVERT_DRIVE_ODOMETRY = true;       // When driving FORWARD, odometry value must INCREASE. If not, flip this
    public static final boolean INVERT_STRAFE_ODOMETRY = true;      // When strafing to the LEFT, the odometry value must INCREASE. If not, flip this

    //TODO: Consider these values and if they need to be changed. These values are the defaults from the example code

    // Constants that set limits on the closed loop drive train
    // Values can be altered based on system
    public static final double DRIVE_ACCEL         = 2.0;     // Acceleration limit.  Percent Power change per second.  1.0 = 0-100% power in 1 sec.
    public static final double DRIVE_TOLERANCE     = 0.5;     // Controller is is "inPosition" if position error is < +/- this amount
    public static final double DRIVE_DEADBAND      = 0.2;     // Error less than this causes zero output.  Must be smaller than DRIVE_TOLERANCE
    public static final double DRIVE_MAX_AUTO      = 0.8;     // "default" Maximum Axial power limit during autonomous

    public static final double STRAFE_ACCEL        = 1.5;     // Acceleration limit.  Percent Power change per second.  1.0 = 0-100% power in 1 sec.
    public static final double STRAFE_TOLERANCE    = 0.5;     // Controller is is "inPosition" if position error is < +/- this amount
    public static final double STRAFE_DEADBAND     = 0.2;     // Error less than this causes zero output.  Must be smaller than DRIVE_TOLERANCE
    public static final double STRAFE_MAX_AUTO     = 0.8;     // "default" Maximum Lateral power limit during autonomous

    // Note that these yaw values are using radians, which is why the scale values are smaller

    public static final double YAW_ACCEL           = 3.0;     // Acceleration limit.  Percent Power change per second.  1.0 = 0-100% power in 1 sec.
    public static final double YAW_TOLERANCE       = 0.035;   // Controller is is "inPosition" if position error is < +/- this amount
    public static final double YAW_DEADBAND        = 0.01;    // Error less than this causes zero output.  Must be smaller than DRIVE_TOLERANCE
    public static final double YAW_MAX_AUTO        = 1;       // "default" Maximum Yaw power limit during autonomous


}
