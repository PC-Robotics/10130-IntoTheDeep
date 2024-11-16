package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.Utility.clamp;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;


public class Robot {
    private LinearOpMode myOpMode = null;   // gain access to methods in the calling OpMode.

    public ElapsedTime timer = new ElapsedTime();

    // Define Motor and Servo objects  (Make them private so they can't be accessed externally)
    public DcMotor frontRight = null;
    public DcMotor frontLeft = null;
    public DcMotor backRight = null;
    public DcMotor backLeft = null;
    public DcMotor linearSlide = null;
    public Servo wrist = null;
    public CRServo intake = null;
    public Servo trolleyRight = null;
    public Servo trolleyLeft = null;
    public Servo claw = null;
    public Servo bucket = null;
    public IMU imu = null;

    // Define a constructor that allows the OpMode to pass a reference to itself.
    public Robot(LinearOpMode opmode) {
        myOpMode = opmode;
    }

    // initialize (main function)
    public void init() {
        // FTC Dashboard
        System.out.println(
                "Connect to the Control Hub's WiFi and go to the following URL to access FTC Dashboard:" +
                        "\n" +
                        "192.168.43.1:8080/dash"
        );

        // clear telemetry screen
        myOpMode.telemetry.clearAll();
        myOpMode.telemetry.addData("> (INFO) - ", "Telemetry Initialized");
        myOpMode.telemetry.update();

        // init
        initMotors();
        initSensors();

    }

    private void initMotors() {
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

        frontRight = motorInit("frontRight", DcMotor.Direction.FORWARD, DcMotor.ZeroPowerBehavior.BRAKE);
        frontLeft = motorInit("frontLeft", DcMotor.Direction.REVERSE, DcMotor.ZeroPowerBehavior.BRAKE);
        backRight = motorInit("backRight", DcMotor.Direction.FORWARD, DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft = motorInit("backLeft", DcMotor.Direction.REVERSE, DcMotor.ZeroPowerBehavior.BRAKE);
        linearSlide = motorInit("linearSlide", DcMotor.Direction.REVERSE, DcMotor.ZeroPowerBehavior.FLOAT);
        wrist = servoInit("wrist", Servo.Direction.FORWARD);
        intake = CRservoInit("intake", CRServo.Direction.FORWARD);
        trolleyRight = servoInit("right", Servo.Direction.FORWARD);
        trolleyLeft = servoInit("left", Servo.Direction.FORWARD);
        claw = servoInit("claw", Servo.Direction.FORWARD);
        bucket = servoInit("bucket", Servo.Direction.FORWARD);

        // telemetry
        myOpMode.telemetry.addData("> (INFO) - ", "Motors Initialized");
        myOpMode.telemetry.update();
    }

    private void initSensors() {
        // Initialize the IMU
        imu = myOpMode.hardwareMap.get(IMU.class, "imu");
        IMU.Parameters myIMUParameters = new IMU.Parameters(
                new RevHubOrientationOnRobot(
                        RevHubOrientationOnRobot.LogoFacingDirection.UP,
                        RevHubOrientationOnRobot.UsbFacingDirection.LEFT
                )
        );

        // telemetry
        myOpMode.telemetry.addData("> (INFO) - ", "Sensors Initialized");
        myOpMode.telemetry.update();
    }


    // get an object containing euler angles
    public YawPitchRollAngles getEulerAngles() {
        return imu.getRobotYawPitchRollAngles();
    }

    // get the heading of the robot in radians
    public double getHeading() {
        return getHeading(AngleUnit.RADIANS);
    }

    public double getHeading(AngleUnit unit) {
        return imu.getRobotYawPitchRollAngles().getYaw(unit);
    }


    // HELPERS

    // INIT HELPERS
    private DcMotor motorInit(String motorName, DcMotor.Direction direction, DcMotor.ZeroPowerBehavior zeroPowerBehavior) {
        DcMotor motor = myOpMode.hardwareMap.get(DcMotor.class, motorName);
        motor.setDirection(direction);
        motor.setZeroPowerBehavior(zeroPowerBehavior);

        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        return motor;
    }

    private Servo servoInit(String servoName, Servo.Direction direction) {
        Servo servo = myOpMode.hardwareMap.get(Servo.class, servoName);
        servo.setDirection(direction);
        return servo;
    }

    private CRServo CRservoInit(String CRservoName, CRServo.Direction direction) {
        CRServo CRservo = myOpMode.hardwareMap.get(CRServo.class, CRservoName);
        CRservo.setDirection(direction);
        return CRservo;
    }

    // MOVEMENT

    public void setMotorPowers(double[] powers) {
        frontLeft.setPower(powers[0]);
        backLeft.setPower(powers[1]);
        frontRight.setPower(powers[2]);
        backRight.setPower(powers[3]);
    }

    public void setMotorPowers(double power) {
        frontLeft.setPower(power);
        backLeft.setPower(power);
        frontRight.setPower(power);
        backRight.setPower(power);
    }

    public void setMotorPowers(double frontLeftPower, double backLeftPower, double frontRightPower, double backRightPower) {
        frontLeft.setPower(frontLeftPower);
        backLeft.setPower(backLeftPower);
        frontRight.setPower(frontRightPower);
        backRight.setPower(backRightPower);
    }

    public double getAverageDrivePosition() {
        return (double) (frontLeft.getCurrentPosition() + frontRight.getCurrentPosition() + backLeft.getCurrentPosition() + backRight.getCurrentPosition()) / (4 * Settings.Autonomous.TICKS_PER_IN);
    }

    public void stopMotors() {
        setMotorPowers(0);
    }

    public void closeClaw() {
        claw.setPosition(Settings.Claw.CLOSED_POSITION);
    }

    public void openClaw() {
        claw.setPosition(Settings.Claw.CLOSED_POSITION);
    }

    public void pickupBucket () {
        bucket.setPosition(Settings.Bucket.PICKUP_POSITION);
    }

    public void releaseBucket () {
        bucket.setPosition(Settings.Bucket.RELEASE_POSITION);
    }

    public void intake() {
        intake.setPower(Settings.Intake.MAX_POWER);
    }

    public void outtake() {
        intake.setPower(-Settings.Intake.MAX_POWER);
    }

    public void stopIntake() {
        intake.setPower(0);
    }

    public void trolleyIn() {
        setTrolleyPosition(Settings.Trolley.IN_POSITION);
    }

    public void setTrolleyPosition(double position) {
        trolleyRight.setPosition(position);
        trolleyLeft.setPosition(position);
    }

    public double getTrolleyPosition() {
        return (trolleyRight.getPosition() + trolleyLeft.getPosition()) / 2;
    }

    private int linearSlideIndex = 0;

    public void increaseLinearSlidePosition(double power) {
        if (linearSlideIndex < Settings.LinearSlide.POSITIONS.length() - 1) {
            linearSlideIndex++;
            runLinearSlideToPosition(Settings.LinearSlide.POSITIONS.get(linearSlideIndex), power);
        }

        if (linearSlideIndex == 1) {
            linearSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }
    }

    public void decreaseLinearSlidePosition(double power) {
        if (linearSlideIndex > 0) {
            linearSlideIndex--;
            runLinearSlideToPosition(Settings.LinearSlide.POSITIONS.get(linearSlideIndex), power);
        }

        if (linearSlideIndex == 0) {
            linearSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        }
    }

    public int getLinearSlideIndex() {
        return linearSlideIndex;
    }

    // HELPERS

    public void runLinearSlideToPosition(int position) {
        runLinearSlideToPosition(position, Settings.LinearSlide.POWER);
    }

    public void runLinearSlideToPosition(int position, double power) {
        linearSlide.setTargetPosition(position);
        linearSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        linearSlide.setPower(power);
    }

    public void stopLinearSlide() {
        linearSlide.setPower(0);
        linearSlide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void releaseSpecimen() {
        runLinearSlideToPosition(Settings.LinearSlide.SPECIMEN_APPROACH_POSITION);
        while (linearSlide.isBusy()) {
            // do nothing
        }

        runLinearSlideToPosition(Settings.LinearSlide.SPECIMEN_LOWERED_POSITION);
        while (linearSlide.isBusy()) {
            // do nothing
        }

        openClaw();
        myOpMode.sleep(500);

        driveDistance(-10);

        runLinearSlideToPosition(Settings.LinearSlide.STARTING_POSITION);
        linearSlideIndex = 0;
    }


    private int wristIndex = 0;

    public void increaseWristPosition() {
        if (wristIndex < 2) {
            wristIndex++;
            wrist.setPosition(Settings.Wrist.POSITIONS.get(wristIndex));
        }
    }

    public void decreaseWristPosition() {
        if (wristIndex > 0) {
            wristIndex--;
            wrist.setPosition(Settings.Wrist.POSITIONS.get(wristIndex));
        }
    }

    public int getWristIndex() {
        return wristIndex;
    }


    // DRIVE

    public void driveDistance(double distance_in) {
        driveDistance(distance_in, Settings.Autonomous.DEFAULT_DRIVE_TIMEOUT_MS);
    }

    public void driveDistance(double distance_in, int timeout) {
        double startingTime = timer.milliseconds(); // get the current time
        double elapsedTime = 0; // timer creation

        double drivePosition = getAverageDrivePosition(); // get the current position of the drive
        double target = drivePosition - distance_in; // set the target position (current position + distance you want to travel)
        double error = -distance_in; // Error - the distance between the current position and the target position (how much we have left to travel)

        // check if opMode is active (so we can stop the robot)
        // check if the error is less than a certain value (else the robot will have to drive to inifinite presicion)
        // check if the timer hasn't reached the timeout (so the robot doesn't drive forever)
        while (Math.abs(error) > Settings.Autonomous.DrivePID.ELIPSON && elapsedTime < timeout && myOpMode.opModeIsActive()) {
            // for every loop, get the current position of the drive and recalculate the error
            drivePosition = getAverageDrivePosition();
            error = target - drivePosition;

            // make sure the power is within the min and max power values (with the clamp function)
            double power = clamp(
                    (Math.abs(error) * Settings.Autonomous.DrivePID.kP) / 100,
                    Settings.Autonomous.DEFAULT_DRIVE_MIN_POWER,
                    Settings.Autonomous.DEFAULT_DRIVE_MAX_POWER
            );

            // set the motor powers to the power value
            setMotorPowers(-power);

            // update the elapsed time
            elapsedTime = timer.milliseconds() - startingTime;

            myOpMode.telemetry.addData("Heading", getHeading(AngleUnit.DEGREES));
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
        double error = targetAngle_degrees - getHeading(AngleUnit.DEGREES); // Error - the difference between the target angle and the current angle (how much we have left to turn)

        // check if opMode is active (so we can stop the robot)
        // check if the error is less than a certain value (else the robot will have to turn to inifinite presicion)
        // check if the timer hasn't reached the timeout (so the robot doesn't turn forever)
        while (Math.abs(error) > Settings.Autonomous.TurnPID.ELIPSON && elapsedTime < timeout && myOpMode.opModeIsActive()) {
            error = targetAngle_degrees - getHeading(AngleUnit.DEGREES); // recalculate the error

            // make sure the power is within the min and max power values (with the clamp function)
            double power = clamp(
                    (Math.abs(error) * Settings.Autonomous.TurnPID.kP) / 100,
                    Settings.Autonomous.DEFAULT_TURN_MIN_POWER,
                    Settings.Autonomous.DEFAULT_TURN_MAX_POWER
            );

            // set the motor powers to the power value
            setMotorPowers(power, power, -power, -power);

            // update the elapsed time
            elapsedTime = timer.milliseconds() - startingTime;

            myOpMode.telemetry.addData("Target Angle", targetAngle_degrees);
            myOpMode.telemetry.addData("Error", error);
            myOpMode.telemetry.addData("Power", power);
            myOpMode.telemetry.addData("Elapsed Time", elapsedTime);
            myOpMode.telemetry.addData("Heading", getHeading(AngleUnit.DEGREES));
            myOpMode.telemetry.update();
        }
    }
}