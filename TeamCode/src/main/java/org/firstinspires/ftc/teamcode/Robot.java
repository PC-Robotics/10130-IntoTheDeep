package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.Utility.clamp;
import static org.firstinspires.ftc.teamcode.Utility.normalizePowers;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

import org.firstinspires.ftc.teamcode.subsystems.DriveBase;
import org.firstinspires.ftc.teamcode.subsystems.LinearSlide;
import org.firstinspires.ftc.teamcode.subsystems.OurIMU;
import org.firstinspires.ftc.teamcode.subsystems.PIDF;
import org.firstinspires.ftc.teamcode.subsystems.Trolley;
import org.firstinspires.ftc.teamcode.subsystems.Wrist;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Bucket;
import org.firstinspires.ftc.teamcode.subsystems.Claw;

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

    private PIDF drivePID, strafePID, turnPID;

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

        drivePID = new PIDF(
                Settings.Autonomous.DrivePID.kP,
                Settings.Autonomous.DrivePID.kI,
                Settings.Autonomous.DrivePID.kD,
                Settings.Autonomous.DrivePID.TOLERANCE,
                Settings.Autonomous.DrivePID.TIME_TO_SETTLE
        );

        strafePID = new PIDF(
                Settings.Autonomous.StrafePID.kP,
                Settings.Autonomous.StrafePID.kI,
                Settings.Autonomous.StrafePID.kD,
                Settings.Autonomous.StrafePID.TOLERANCE,
                Settings.Autonomous.StrafePID.TIME_TO_SETTLE
        );

        turnPID = new PIDF(
                Settings.Autonomous.TurnPID.kP,
                Settings.Autonomous.TurnPID.kI,
                Settings.Autonomous.TurnPID.kD,
                Settings.Autonomous.TurnPID.TOLERANCE,
                Settings.Autonomous.TurnPID.TIME_TO_SETTLE
        );
    }

    // initialize (main function)
    public void init() {
        imu.init();
        driveBase.init();
        linearSlide.init();
        trolley.init();
        wrist.init();
        intake.init();
        bucket.init();
        claw.init();

        turnPID.setIsAngular(true);
    }

    // DRIVE

    public void driveDistance(double distance_in) {
        driveDistance(distance_in, Settings.Autonomous.DEFAULT_DRIVE_TIMEOUT_MS);
    }

    public void driveDistance(double distance_in, int timeout) {
        double startingTime = timer.milliseconds(); // get the current time
        double currentTime = timer.milliseconds();
        double startingHeading = imu.getHeading(AngleUnit.DEGREES);

        drivePID.reset();
        strafePID.reset();
        turnPID.reset();

        driveBase.resetOdometry();

        while (currentTime - startingTime < timeout && myOpMode.opModeIsActive()) {
            if (drivePID.isSettled() && strafePID.isSettled() && turnPID.isSettled()) {
                break;
            }

            double drivePower = drivePID.calculate(distance_in, driveBase.inchesTraveledY, currentTime);
            double strafePower = strafePID.calculate(0, driveBase.inchesTraveledX, currentTime);
            double turnPower = turnPID.calculate(startingHeading, imu.getHeading(AngleUnit.DEGREES), currentTime);

            driveBase.setMotorPowers(normalizePowers(new double[] {
                    drivePower + strafePower + turnPower,
                    drivePower - strafePower + turnPower,
                    drivePower - strafePower - turnPower,
                    drivePower + strafePower - turnPower
            }));

            driveBase.updateOdometry();

            myOpMode.telemetry.addData("Drive Power", drivePower)
                    .addData("Strafe Power", strafePower)
                    .addData("Turn Power", turnPower)
                    .addData("Drive Error", drivePID.getError())
                    .addData("Strafe Error", strafePID.getError())
                    .addData("Turn Error", turnPID.getError())
                    .addData("Target Position", distance_in)
                    .addData("Current Position", driveBase.inchesTraveledY)
                    .addData("Heading", imu.getHeading(AngleUnit.DEGREES));
            myOpMode.telemetry.update();
        }
    }

    public void strafeDistance(double distance_in) {
        strafeDistance(distance_in, Settings.Autonomous.DEFAULT_DRIVE_TIMEOUT_MS);
    }

    public void strafeDistance(double distance_in, int timeout) {
        double startingTime = timer.milliseconds(); // get the current time
        double currentTime = timer.milliseconds();
        double startingHeading = imu.getHeading(AngleUnit.DEGREES);

        drivePID.reset();
        strafePID.reset();
        turnPID.reset();

        driveBase.resetOdometry();

        while (currentTime - startingTime < timeout && myOpMode.opModeIsActive()) {
            if (drivePID.isSettled() && strafePID.isSettled() && turnPID.isSettled()) {
                break;
            }

            double drivePower = drivePID.calculate(0, driveBase.inchesTraveledY, currentTime);
            double strafePower = strafePID.calculate(distance_in, driveBase.inchesTraveledX, currentTime);
            double turnPower = turnPID.calculate(startingHeading, imu.getHeading(AngleUnit.DEGREES), currentTime);

            driveBase.setMotorPowers(normalizePowers(new double[] {
                    drivePower + strafePower + turnPower,
                    drivePower - strafePower + turnPower,
                    drivePower - strafePower - turnPower,
                    drivePower + strafePower - turnPower
            }));

            driveBase.updateOdometry();

            myOpMode.telemetry.addData("Drive Power", drivePower)
                    .addData("Strafe Power", strafePower)
                    .addData("Turn Power", turnPower)
                    .addData("Drive Error", drivePID.getError())
                    .addData("Strafe Error", strafePID.getError())
                    .addData("Turn Error", turnPID.getError())
                    .addData("Target Position", distance_in)
                    .addData("Current Position", driveBase.inchesTraveledX)
                    .addData("Heading", imu.getHeading(AngleUnit.DEGREES));
            myOpMode.telemetry.update();
        }
    }

    public void turnAbsolute(double targetAngle_degrees) {
        strafeDistance(targetAngle_degrees, Settings.Autonomous.DEFAULT_DRIVE_TIMEOUT_MS);
    }

    public void turnAbsolute(double targetAngle_degrees, int timeout) {
        double startingTime = timer.milliseconds(); // get the current time
        double currentTime = timer.milliseconds();

        drivePID.reset();
        strafePID.reset();
        turnPID.reset();

        driveBase.resetOdometry();

        while (currentTime - startingTime < timeout && myOpMode.opModeIsActive()) {
            if (drivePID.isSettled() && strafePID.isSettled() && turnPID.isSettled()) {
                break;
            }

            double drivePower = drivePID.calculate(0, driveBase.inchesTraveledY, currentTime);
            double strafePower = strafePID.calculate(0, driveBase.inchesTraveledX, currentTime);
            double turnPower = turnPID.calculate(targetAngle_degrees, imu.getHeading(AngleUnit.DEGREES), currentTime);

            driveBase.setMotorPowers(normalizePowers(new double[] {
                    drivePower + strafePower + turnPower,
                    drivePower - strafePower + turnPower,
                    drivePower - strafePower - turnPower,
                    drivePower + strafePower - turnPower
            }));

            driveBase.updateOdometry();

            myOpMode.telemetry.addData("Drive Power", drivePower)
                    .addData("Strafe Power", strafePower)
                    .addData("Turn Power", turnPower)
                    .addData("Drive Error", drivePID.getError())
                    .addData("Strafe Error", strafePID.getError())
                    .addData("Turn Error", turnPID.getError())
                    .addData("Target Position", targetAngle_degrees)
                    .addData("Current Position", imu.getHeading(AngleUnit.DEGREES));
            myOpMode.telemetry.update();
        }
    }
}