package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.Utility.applyDeadzone;
import static org.firstinspires.ftc.teamcode.Utility.clamp;
import static org.firstinspires.ftc.teamcode.Utility.normalizePowers;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

/**
 * bumpers (hold) - intake
 * joys - mecanum
 * dpad up down - wrist movement traversal
 * triggers (hold) - arm
 * square - bucket pickup
 * triangle - bucket out
 * circle - claw close
 * x - claw open
 * dpad left right - linearslide movement traversal
 */
@TeleOp(name="Main Teleop Program", group="Linear OpMode")
// TODO - figure out subsystems and seperate drive.
public class MainTeleop extends LinearOpMode {
    private int linearSlideIndex = 0;
    private int wristIndex = 0;

    private boolean dpadLeftPressed = false;
    private boolean dpadRightPressed = false;
    private boolean dpadUpPressed = false;
    private boolean dpadDownPressed = false;

    Robot robot = new Robot(this);

    private double[] powers = new double[4];
    private double straight, turn, strafe, heading;

    private double leftTrigger, rightTrigger;

    private double slidePosition;
    private double wristPosition, armPosition, clawPosition;

    public void runOpMode() {
        robot.init();
        waitForStart();
        robot.imu.resetYaw();

        initOpMode();

        while (opModeIsActive()) {
            readSensors();
            mecanumDrive(straight, turn, strafe);
            linearSlideControl();
            clawControl();
            armControl();
            wristControl();
            intakeControl();
            bucketControl();
            updateTelemetryData();
        }
    }

    private void initOpMode() {
        robot.linearSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.linearSlide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.linearSlide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.linearSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        robot.setArmPosition(Settings.ARM_IN_POSITION);
        robot.wrist.setPosition(Settings.WRIST_DRIVING_POSITION);
        robot.bucket.setPosition(Settings.BUCKET_PICKUP_POSITION);
        robot.claw.setPosition(Settings.CLAW_OPEN_POSITION);

        robot.linearSlide.setTargetPosition(Settings.LINEAR_SLIDE_STARTING_POSITION);
        robot.linearSlide.setPower(1.0);

    }

    private void readSensors() {
        straight = applyDeadzone(gamepad1.left_stick_y, Settings.DEADZONE_THRESHOLD);
        turn = applyDeadzone(gamepad1.right_stick_x, Settings.DEADZONE_THRESHOLD);
        strafe = applyDeadzone(-gamepad1.left_stick_x, Settings.DEADZONE_THRESHOLD); // invert pad left x
        heading = robot.getHeading(); // in radians

        rightTrigger = applyDeadzone(gamepad1.right_trigger, Settings.DEADZONE_THRESHOLD);
        leftTrigger = applyDeadzone(gamepad1.left_trigger, Settings.DEADZONE_THRESHOLD);
    }

    private void mecanumDrive(double straight, double turn, double strafe) {
        // calculate powers
        powers[0] = straight + turn + strafe; // front left power
        powers[1] = straight + turn - strafe; // back left power
        powers[2] = straight - turn - strafe; // front right power
        powers[3] = straight - turn + strafe; // back right power

        // powers array is updated inside this method
        powers = normalizePowers(powers);

        // set powers to motors
        robot.setMotorPowers(powers);
    }

    private void linearSlideControl() {
        if (gamepad1.dpad_left) {
            if (!dpadLeftPressed) {
                dpadLeftPressed = true;
                linearSlideIndex += 1;
            }
        } else {
            dpadLeftPressed = false;
        }

        if (gamepad1.dpad_right) {
            if (!dpadRightPressed) {
                dpadRightPressed = false;
                linearSlideIndex -= 1;
            }
        } else {
            dpadRightPressed = false;
        }

        linearSlideIndex =  clamp(linearSlideIndex, 0, 2);

        robot.linearSlide.setTargetPosition(Settings.LINEAR_SLIDE_POSITIONS[linearSlideIndex]);
        robot.linearSlide.setPower(0.66);
    }

    private void clawControl() {
        if (gamepad1.x) {
            robot.claw.setPosition(Settings.CLAW_CLOSED_POSITION);
        } else if (gamepad1.circle) {
            robot.claw.setPosition(Settings.CLAW_OPEN_POSITION);
        }
    }

    private void armControl() {
        double newArmPosition = robot.getArmPosition();
        
        if (rightTrigger != 0) {
            newArmPosition -= (rightTrigger / 300);
        } else if (leftTrigger != 0) {
            newArmPosition += (leftTrigger / 300);
        }
        
        newArmPosition = clamp(newArmPosition, Settings.ARM_OUT_POSITION, Settings.ARM_IN_POSITION);
        robot.setArmPosition(newArmPosition);
    }

    private void wristControl() {
        if (gamepad1.dpad_up) {
            if (!dpadUpPressed) {
                dpadUpPressed = true;
                wristIndex += 1;
            }
        } else {
            dpadUpPressed = false;
        }

        if (gamepad1.dpad_down) {
            if (!dpadDownPressed) {
                dpadDownPressed = true;
                wristIndex -= 1;
            }
        } else {
            dpadDownPressed = false;
        }

        wristIndex = clamp(wristIndex, 0, 2);

        robot.wrist.setPosition(Settings.WRIST_POSITIONS[wristIndex]);
    }

    private void intakeControl() {
        if (gamepad1.right_bumper) {
            robot.intake.setPower(.3);
        } else if (gamepad1.left_bumper) {
            robot.intake.setPower(-.3);
        } else {
            robot.intake.setPower(0);
        }
    }

    private void bucketControl() {
        if (gamepad1.square) {
            robot.bucket.setPosition(Settings.BUCKET_PICKUP_POSITION);
        } else if (gamepad1.triangle) {
            robot.bucket.setPosition(Settings.BUCKET_RELEASE_POSITION);
        }
    }

    private void updateTelemetryData() {
        telemetry.addData("Left Pad Y", straight)
                .addData("Left Pad X", strafe)
                .addData("Right Pad X", turn)
                .addData("Front Left Power", powers[0])
                .addData("Back Left Power", powers[1])
                .addData("Front Right Power", powers[2])
                .addData("Back Right Power", powers[3])
                .addData("Heading", Math.toDegrees(heading) + 180)
                .addData("slide", linearSlideIndex)
                .addData("wrist", wristIndex);

        telemetry.update();
    }
}
