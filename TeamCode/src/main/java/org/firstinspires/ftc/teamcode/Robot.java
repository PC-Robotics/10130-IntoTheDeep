package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.Utility.clamp;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.subsystems.Bucket;
import org.firstinspires.ftc.teamcode.subsystems.Claw;
import org.firstinspires.ftc.teamcode.subsystems.DriveBase;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.LinearSlide;
import org.firstinspires.ftc.teamcode.subsystems.OurIMU;
import org.firstinspires.ftc.teamcode.subsystems.Trolley;
import org.firstinspires.ftc.teamcode.subsystems.Wrist;

/*
 * Control -
 *      Servo -
 *          0: wrist
 *          2: (CR) intake
 *      Motors -
 *          0: rightFront
 *          1: rightBack
 *          2: leftFront
 *          3: leftBack
 * Expansion -
 *      Servo -
 *          0: right
 *          2: left
 *          4: claw
 *      Motors -
 *          0: linearSlide
 */
public class Robot {
    private LinearOpMode myOpMode = null;   // gain access to methods in the calling OpMode.

    public ElapsedTime timer = new ElapsedTime();

    public DriveBase driveBase;
    public LinearSlide linearSlide;
    public Trolley trolley;
    public Wrist wrist;
    public Intake intake;
    public Bucket bucket;
    public Claw claw;
    public OurIMU imu;

    // Define a constructor that allows the OpMode to pass a reference to itself.
    public Robot(LinearOpMode opMode) {
        this.myOpMode = opMode;
        imu = new OurIMU(myOpMode);
        driveBase = new DriveBase(myOpMode);
        linearSlide = new LinearSlide(myOpMode);
        trolley = new Trolley(myOpMode);
        wrist = new Wrist(myOpMode);
        intake = new Intake(myOpMode);
        bucket = new Bucket(myOpMode);
        claw = new Claw(myOpMode);
    }

    // initialize (main function)
    public void init() {
        imu.init();
        driveBase.init();
        linearSlide.init();
        // trolley.init();
        wrist.init();
        intake.init();
        bucket.init();
        claw.init();
    }

    // DRIVE

    public void driveDistance(double distance_in) {
        driveDistance(distance_in, Settings.Autonomous.DEFAULT_DRIVE_TIMEOUT_MS);
    }

    public void driveDistance(double distance_in, int timeout) {
        double startingTime = timer.milliseconds(); // get the current time
        double elapsedTime = 0; // timer creation

        double drivePosition = driveBase.getAverageDrivePosition(); // get the current position of the drive
        double target = drivePosition - distance_in; // set the target position (current position + distance you want to travel)
        double error = -distance_in; // Error - the distance between the current position and the target position (how much we have left to travel)

        // check if opMode is active (so we can stop the robot)
        // check if the error is less than a certain value (else the robot will have to drive to inifinite presicion)
        // check if the timer hasn't reached the timeout (so the robot doesn't drive forever)
        while (Math.abs(error) > Settings.Autonomous.DrivePID.ELIPSON && elapsedTime < timeout && myOpMode.opModeIsActive()) {
            // for every loop, get the current position of the drive and recalculate the error
            drivePosition = driveBase.getAverageDrivePosition();
            error = target - drivePosition;

            // make sure the power is within the min and max power values (with the clamp function)
            double power = clamp(
                    (Math.abs(error) * Settings.Autonomous.DrivePID.kP) / 100,
                    Settings.Autonomous.DEFAULT_DRIVE_MIN_POWER,
                    Settings.Autonomous.DEFAULT_DRIVE_MAX_POWER
            );

            // set the motor powers to the power value
            driveBase.setMotorPowers(-power);

            // update the elapsed time
            elapsedTime = timer.milliseconds() - startingTime;

            myOpMode.telemetry.addData("Heading", imu.getHeading(AngleUnit.DEGREES));
            myOpMode.telemetry.addData("Drive Position", drivePosition);
            myOpMode.telemetry.addData("Target Position", target);
            myOpMode.telemetry.addData("Error", error);
            myOpMode.telemetry.addData("Power", power);
            myOpMode.telemetry.addData("Elapsed Time", elapsedTime);
            myOpMode.telemetry.update();
        }
    }

    public void turnAbsolute(double targetAngle_degrees) {
        turnAbsolute(targetAngle_degrees, Settings.Autonomous.DEFAULT_TURN_TIMEOUT_MS);
    }

    public void turnAbsolute(double targetAngle_degrees, int timeout) {
        double startingTime = timer.milliseconds(); // get the current time
        double elapsedTime = 0; // timer creation

        // since we are turning absolute, our target _is_ the parameter that we pass in.
        double error = targetAngle_degrees - imu.getHeading(AngleUnit.DEGREES); // Error - the difference between the target angle and the current angle (how much we have left to turn)

        // check if opMode is active (so we can stop the robot)
        // check if the error is less than a certain value (else the robot will have to turn to inifinite presicion)
        // check if the timer hasn't reached the timeout (so the robot doesn't turn forever)
        while (Math.abs(error) > Settings.Autonomous.TurnPID.ELIPSON && elapsedTime < timeout && myOpMode.opModeIsActive()) {
            error = targetAngle_degrees - imu.getHeading(AngleUnit.DEGREES); // recalculate the error

            // make sure the power is within the min and max power values (with the clamp function)
            double power = clamp(
                    (Math.abs(error) * Settings.Autonomous.TurnPID.kP) / 100,
                    Settings.Autonomous.DEFAULT_TURN_MIN_POWER,
                    Settings.Autonomous.DEFAULT_TURN_MAX_POWER
            );

            // set the motor powers to the power value
            driveBase.setMotorPowers(power, power, -power, -power);

            // update the elapsed time
            elapsedTime = timer.milliseconds() - startingTime;

            myOpMode.telemetry.addData("Target Angle", targetAngle_degrees);
            myOpMode.telemetry.addData("Error", error);
            myOpMode.telemetry.addData("Power", power);
            myOpMode.telemetry.addData("Elapsed Time", elapsedTime);
            myOpMode.telemetry.addData("Heading", imu.getHeading(AngleUnit.DEGREES));
            myOpMode.telemetry.update();
        }
    }
}