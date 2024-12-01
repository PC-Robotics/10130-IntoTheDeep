package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.IMU;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

import java.util.List;


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
    public Servo trolleyRight = null;
    public Servo trolleyLeft = null;
    public Servo claw = null;
    public Servo bucket = null;
    public IMU imu = null;

    public List<LynxModule> allHubs;

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

        /**
         * Bulk Caching Mode (see ConceptMotorBulkRead.java in the external.samples directory)
         * Going into the Control Hub, asking it for a servo, motor, or encoder position, and returning it
         * every time we call a .get() method is slow.
         * Instead, we ask the Control Hub to save EVERYTHING in a big table at the start of each loop.
         * When we call a .get() method, we're just reading from that table.
         * This is called "bulk reading" and it's much faster.
         *
         * Analogy - Instead of going to the store every time you need a new ingredient,
         * you go once, buy everything you need, and store it at home. You're buying in "bulk".
         **/
        allHubs = myOpMode.hardwareMap.getAll(LynxModule.class);
        for (LynxModule module : allHubs) {
            module.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }

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

    public void stopMotors() {
        setMotorPowers(0);
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
        if (linearSlideIndex < 2) {
            linearSlideIndex++;
            runLinearSlideToPosition(Settings.LINEAR_SLIDE_POSITIONS[linearSlideIndex], power);
        }

        if (linearSlideIndex == 1) {
            linearSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }
    }

    public void decreaseLinearSlidePosition(double power) {
        if (linearSlideIndex > 0) {
            linearSlideIndex--;
            runLinearSlideToPosition(Settings.LINEAR_SLIDE_POSITIONS[linearSlideIndex], power);
        }

        if (linearSlideIndex == 0) {
            linearSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        }
    }

    public int getLinearSlideIndex() {
        return linearSlideIndex;
    }

    // HELPERS

    public void runLinearSlideToPosition(int position, double power) {
        linearSlide.setTargetPosition(position);
        linearSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        linearSlide.setPower(power);
    }

    public void stopLinearSlide() {
        linearSlide.setPower(0);
        linearSlide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
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