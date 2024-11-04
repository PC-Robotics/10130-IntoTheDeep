package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.IMU;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;


public class Robot {
    private LinearOpMode myOpMode = null;   // gain access to methods in the calling OpMode.

    // Define Motor and Servo objects  (Make them private so they can't be accessed externally)
    public DcMotor frontRight = null;
    public DcMotor frontLeft = null;
    public DcMotor backRight = null;
    public DcMotor backLeft = null;
    public DcMotor linearSlide = null;
    public Servo wrist = null;
    public CRServo intake = null;
    public Servo right = null;
    public Servo left = null;
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
        right = servoInit("right", Servo.Direction.FORWARD);
        left = servoInit("left", Servo.Direction.FORWARD);
        claw = servoInit("claw", Servo.Direction.FORWARD);
        bucket = servoInit("bucket", Servo.Direction.FORWARD);

        // telemetry
        myOpMode.telemetry.addData("> (INFO) - ", "Motors Initialized");
        myOpMode.telemetry.update();
    }

    private void initSensors() {
        // Initialize the IMU
        imu = myOpMode.hardwareMap.get(IMU.class, "imu");

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
        return imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
    }


    // HELPERS

    // INIT HELPERS
    private DcMotor motorInit(String motorName, DcMotor.Direction direction, DcMotor.ZeroPowerBehavior zeroPowerBehavior) {
        DcMotor motor = myOpMode.hardwareMap.get(DcMotor.class, motorName);
        motor.setDirection(direction);
        motor.setZeroPowerBehavior(zeroPowerBehavior);
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

    public void stopMotors() {
        setMotorPowers(0);
    }

    public void setArmPosition(double position) {
        right.setPosition(position);
        left.setPosition(position);
    }

    public double getArmPosition() {
        return (right.getPosition() + left.getPosition()) / 2;
    }

    private int linearSlideIndex = 0;

    public void increaseLinearSlidePosition(double power) {
        if (linearSlideIndex < 2) {
            linearSlideIndex++;
            linearSlide.setTargetPosition(Settings.LINEAR_SLIDE_POSITIONS[linearSlideIndex]);
            linearSlide.setPower(power);
        }

        if (linearSlideIndex == 1) {
            linearSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }
    }

    public void decreaseLinearSlidePosition(double power) {
        if (linearSlideIndex > 0) {
            linearSlideIndex--;
            linearSlide.setTargetPosition(Settings.LINEAR_SLIDE_POSITIONS[linearSlideIndex]);
            linearSlide.setPower(power);
        }

        if (linearSlideIndex == 0) {
            linearSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        }
    }

    public int getLinearSlideIndex() {
        return linearSlideIndex;
    }


    private int wristIndex = 0;

    public void increaseWristPosition() {
        if (wristIndex < 2) {
            wristIndex++;
            wrist.setPosition(Settings.WRIST_POSITIONS[wristIndex]);
        }
    }

    public void decreaseWristPosition() {
        if (wristIndex > 0) {
            wristIndex--;
            wrist.setPosition(Settings.WRIST_POSITIONS[wristIndex]);
        }
    }

    public int getWristIndex() {
        return wristIndex;
    }
}