package org.firstinspires.ftc.teamcode;

import static androidx.core.math.MathUtils.clamp;
import static org.firstinspires.ftc.teamcode.Utility.applyDeadzone;
import static org.firstinspires.ftc.teamcode.Utility.normalizePowers;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;

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
@TeleOp(name="First Competition Code", group="Linear OpMode")
// TODO - figure out subsystems and seperate drive.
public class FirstCompetitionDrive extends LinearOpMode {
    private int linearSlideIndex = 0;
    private int wristIndex = 0;
    private int bucketIndex = 0;
    private int clawIndex = 0;

    Robot robot = new Robot(this);

    private double[] powers = new double[4];
    private double straight, turn, strafe, heading;

    private double leftTrigger, rightTrigger;

    private double slidePosition;
    private double wristPosition, armPosition, clawPosition;

    private Gamepad currentGamepad = new Gamepad();
    private Gamepad previousGamepad = new Gamepad();

    public void runOpMode() {
        robot.init();
        waitForStart();
        robot.imu.resetYaw();

        initPositions();

        while (opModeIsActive()) {
            previousGamepad.copy(currentGamepad);

            readSensors();
            mecanumDrive(straight, turn, strafe);
            linearSlideControl();
            clawControl();
            armControl();
            wristControl();
            intakeControl();
            bucketControl();
            updateTelemetryData();

            currentGamepad.copy(gamepad1);
        }
    }

    private void initPositions() {
        robot.setArmPosition(Settings.armInPosition);
        robot.wrist.setPosition(Settings.wristDrivingPosition);
        robot.bucket.setPosition(Settings.bucketPickupPosition);
        robot.claw.setPosition(Settings.clawOpenPosition);
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
        robot.frontLeft.setPower(powers[0]);
        robot.backLeft.setPower(powers[1]);
        robot.frontRight.setPower(powers[2]);
        robot.backRight.setPower(powers[3]);
    }

    private void linearSlideControl() {
        if (currentGamepad.y && !previousGamepad.y) {
            linearSlideIndex += 1;
        } else if (currentGamepad.a && !previousGamepad.a) {
            linearSlideIndex -= 1;
        }

        linearSlideIndex = clamp(linearSlideIndex, 0, 2);
    }

    private void clawControl() {
        if (currentGamepad.circle && !previousGamepad.x) {
            robot.claw.setPosition(Settings.clawClosedPosition);
        } else if (currentGamepad.circle && !previousGamepad.circle) {
            robot.claw.setPosition(Settings.clawOpenPosition);
        }
    }

    private void armControl() {
        double newArmPosition = Settings.armInPosition;
        
        if (rightTrigger != 0) {
            newArmPosition = robot.getArmPosition() + leftTrigger;
        } else if (leftTrigger != 0) {
            newArmPosition = robot.getArmPosition() + leftTrigger;
        }
        
        newArmPosition = clamp(newArmPosition, Settings.armInPosition, Settings.armOutPosition);
        robot.setArmPosition(newArmPosition);
    }

    private void wristControl() {
        if (currentGamepad.dpad_up && !previousGamepad.dpad_up) {
            wristIndex += 1;
        }

        if (currentGamepad.dpad_down && !previousGamepad.dpad_down) {
            wristIndex -= 1;
        }

        wristIndex = clamp(wristIndex, 0, 2);

        robot.wrist.setPosition(Settings.wristPositions[wristIndex]);
    }

    private void intakeControl() {
        if (currentGamepad.right_bumper) {
            robot.intake.setPower(1.0);
        } else if (currentGamepad.left_bumper) {
            robot.intake.setPower(-1.0);
        }
    }

    private void bucketControl() {
        if (currentGamepad.square && !previousGamepad.square) {
            robot.bucket.setPosition(Settings.bucketPickupPosition);
        } else if (currentGamepad.triangle && !previousGamepad.triangle) {
            robot.bucket.setPosition(Settings.bucketReleasePosition);
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
                .addData("Heading", Math.toDegrees(heading) + 180);
        telemetry.update();
    }
}
