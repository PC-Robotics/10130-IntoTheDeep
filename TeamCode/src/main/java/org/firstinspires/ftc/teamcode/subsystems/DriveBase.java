package org.firstinspires.ftc.teamcode.subsystems;

import static org.firstinspires.ftc.teamcode.HardwareUtility.motorInit;
import static org.firstinspires.ftc.teamcode.Utility.normalizePowers;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.Settings;

/**
 * DriveBase subsystem implementation for managing the robot's drive motors.
 */
public class DriveBase {
    private LinearOpMode opMode;

    public DcMotor frontLeft, backLeft, frontRight, backRight;

    private double[] powers = new double[4];

    public DriveBase(LinearOpMode opMode) {
        this.opMode = opMode;
    }

    /**
     * Initialize the drive motors and configure their settings.
     */
    public void init() {
        frontLeft = motorInit(opMode.hardwareMap, "frontLeft", DcMotor.Direction.FORWARD);
        backLeft = motorInit(opMode.hardwareMap, "backLeft", DcMotor.Direction.FORWARD);
        frontRight = motorInit(opMode.hardwareMap, "frontRight", DcMotor.Direction.REVERSE);
        backRight = motorInit(opMode.hardwareMap, "backRight", DcMotor.Direction.REVERSE);

        opMode.telemetry.addData("> (INFO)", "DriveBase Initialized");
        opMode.telemetry.update();
    }

    /**
     * Prepare the DriveBase subsystem for operation by setting motors to a safe state.
     */
    public void start() {
        stop(); // Ensure motors are stopped when starting
    }

    /**
     * Stop all drive motors by setting their power to zero.
     */
    public void stop() {
        setMotorPowers(0, 0, 0, 0);
    }

    // DRIVE METHODS

    /**
     * Drive the robot using field-centric drive.
     *
     * @param straight Power for forward/backward movement
     * @param strafe   Power for left/right movement
     * @param turn     Power for turning
     * @see <a href="https://gm0.org/en/latest/docs/software/tutorials/mecanum-drive.html">gm0 guide</a>
     */
    public void fieldCentricDrive(double straight, double strafe, double turn, double heading, boolean fineControl) {

        // calculate rotation
        double rotY = strafe * Math.sin(-heading) + straight * Math.cos(-heading);
        double rotX = strafe * Math.cos(-heading) - straight * Math.sin(-heading);

        // calculate powers
        powers[0] = rotY + rotX + turn; // front left power
        powers[1] = rotY - rotX + turn; // back left power
        powers[2] = rotY - rotX - turn; // front right power
        powers[3] = rotY + rotX - turn; // back right power

        // powers array is updated inside this method
        normalizePowers(powers);

        for (int i = 0; i < 4; i++) {
            if (fineControl) {
                powers[i] *= 0.5;
            }
        }

        setMotorPowers(powers);
    }

    /**
     * Drive the robot using mecanum drive.
     *
     * @param straight Power for forward/backward movement
     * @param strafe   Power for left/right movement
     * @param turn     Power for turning
     */
    public void mecanumDrive(double straight, double strafe, double turn, boolean fineControl) {
        // calculate powers
        powers[0] = straight + strafe + turn; // front left power
        powers[1] = straight - strafe + turn; // back left power
        powers[2] = straight - strafe - turn; // front right power
        powers[3] = straight + strafe - turn; // back right power

        // powers array is updated inside this method
        normalizePowers(powers);

        for (int i = 0; i < 4; i++) {
            if (fineControl) {
                powers[i] *= 0.5;
            }
        }

        setMotorPowers(powers);
    }

    /**
     * Get the average position of all four drive motors.
     *
     * @return Average position of all four drive motors
     */
    public double getAverageDrivePosition() {
        return (double)
                (
                        frontLeft.getCurrentPosition() +
                                frontRight.getCurrentPosition() +
                                backLeft.getCurrentPosition() +
                                backRight.getCurrentPosition()
                ) / (4 * Settings.Autonomous.TICKS_PER_IN);
    }

    // SET MOTOR POWERS

    /**
     * Set motor power levels for all four drive motors.
     *
     * @param frontLeftPower  Power for the front left motor
     * @param backLeftPower   Power for the back left motor
     * @param frontRightPower Power for the front right motor
     * @param backRightPower  Power for the back right motor
     */
    public void setMotorPowers(double frontLeftPower, double backLeftPower, double frontRightPower, double backRightPower) {
        frontLeft.setPower(frontLeftPower);
        backLeft.setPower(backLeftPower);
        frontRight.setPower(frontRightPower);
        backRight.setPower(backRightPower);
    }

    /**
     * Set motor power levels using an array of powers.
     *
     * @param powers Array of power values [frontLeft, backLeft, frontRight, backRight]
     */
    public void setMotorPowers(double[] powers) {
        if (powers.length == 4) {
            setMotorPowers(powers[0], powers[1], powers[2], powers[3]);
        } else {
            throw new IllegalArgumentException("Powers array must have exactly 4 elements");
        }
    }

    /**
     * Set uniform motor power for all drive motors.
     *
     * @param power Power level for all drive motors
     */
    public void setMotorPowers(double power) {
        setMotorPowers(power, power, power, power);
    }

    public void telemetry() {
        opMode.telemetry.addData("Front Left Power", frontLeft.getPower());
        opMode.telemetry.addData("Back Left Power", backLeft.getPower());
        opMode.telemetry.addData("Front Right Power", frontRight.getPower());
        opMode.telemetry.addData("Back Right Power", backRight.getPower());
    }
}
