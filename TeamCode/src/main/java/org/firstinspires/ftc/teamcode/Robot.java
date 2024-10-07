package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
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
    public IMU imu = null;

    // Define a constructor that allows the OpMode to pass a reference to itself.
    public Robot (LinearOpMode opmode) {
        myOpMode = opmode;
    }

    // initialize (main function)
    public void init() {
        // clear telemetry screen
        myOpMode.telemetry.clearAll();
        myOpMode.telemetry.addData("> (INFO) - ", "Telemetry Initialized");
        myOpMode.telemetry.update();

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
        frontLeft = motorInit("frontLeft", DcMotor.Direction.FORWARD, DcMotor.ZeroPowerBehavior.BRAKE);
        backRight = motorInit("backRight", DcMotor.Direction.FORWARD, DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft = motorInit("backLeft", DcMotor.Direction.FORWARD, DcMotor.ZeroPowerBehavior.BRAKE);
        linearSlide = motorInit("linearSlide", DcMotor.Direction.FORWARD, DcMotor.ZeroPowerBehavior.BRAKE);
        wrist = servoInit("wrist", Servo.Direction.FORWARD);
        intake = CRservoInit("intake", CRServo.Direction.FORWARD);
        right = servoInit("right", Servo.Direction.FORWARD);
        left = servoInit("left", Servo.Direction.FORWARD);
        claw = servoInit("claw", Servo.Direction.FORWARD);

        // telemetry
        myOpMode.telemetry.addData("> (INFO) - ", "Motors Initialized");
        myOpMode.telemetry.update();
    }

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

    private void initSensors() {
        // Initialize the IMU
        imu = myOpMode.hardwareMap.get(IMU.class,"imu");

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
}