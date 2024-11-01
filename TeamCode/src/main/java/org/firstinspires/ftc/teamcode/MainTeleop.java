package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.Utility.applyDeadzone;
import static org.firstinspires.ftc.teamcode.Utility.clamp;
import static org.firstinspires.ftc.teamcode.Utility.normalizePowers;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

/**
 * Controller 1 - Driver
 * joysticks - mecanum drive
 * dpad up - linear slide go up
 * dpad down - linear slide go down
 * trigger right - arm go out
 * trigger left - arm go in
 *
 * Controller 2 - Operator
 * bumper left - wrist go left
 * bumper right - wrist go right
 * trigger right - intake spin
 * trigger left - intake spin backwards
 * triangle - bucket release
 * cross - bucket pickup
 * dpad left - claw close
 * dpad right - claw open
 */
@TeleOp(name="Main Teleop Program", group="Linear OpMode")
// TODO - figure out subsystems and seperate drive.
public class MainTeleop extends LinearOpMode {
    Robot robot = new Robot(this);

    private double[] powers = new double[4];
    private double straight, turn, strafe, heading;

    private double gamepad1LeftTrigger, gamepad1RightTrigger, gamepad2LeftTrigger, gamepad2RightTrigger;

    ControllerToggles controllerToggles1 = new ControllerToggles(gamepad1);
    ControllerToggles controllerToggles2 = new ControllerToggles(gamepad2);

    public void runOpMode() {
        robot.init();
        waitForStart();
        robot.imu.resetYaw();

        initOpMode();

        while (opModeIsActive()) {
            readSensors();
            mecanumDrive(straight, turn, strafe);
            linearSlideControl();
            armControl();
            wristControl();
            intakeControl();
            bucketControl();
            clawControl();
            updateTelemetryData();
            updateControllers();
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
        robot.linearSlide.setPower(Settings.LINEAR_SLIDE_POWER);
    }

    private void readSensors() {
        straight = applyDeadzone(gamepad1.left_stick_y, Settings.DEADZONE_THRESHOLD);
        turn = applyDeadzone(gamepad1.right_stick_x, Settings.DEADZONE_THRESHOLD);
        strafe = applyDeadzone(-gamepad1.left_stick_x, Settings.DEADZONE_THRESHOLD); // invert pad left x
        heading = robot.getHeading(); // in radians

        gamepad1RightTrigger = applyDeadzone(gamepad1.right_trigger, Settings.DEADZONE_THRESHOLD);
        gamepad1LeftTrigger = applyDeadzone(gamepad1.left_trigger, Settings.DEADZONE_THRESHOLD);
        gamepad2LeftTrigger = applyDeadzone(gamepad2.left_trigger, Settings.DEADZONE_THRESHOLD);
        gamepad2RightTrigger = applyDeadzone(gamepad2.right_trigger, Settings.DEADZONE_THRESHOLD);
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
        if (controllerToggles1.isDpadUpToggled()) {
            robot.increaseLinearSlidePosition(Settings.LINEAR_SLIDE_POWER);
        } else if (controllerToggles1.isDpadDownToggled()) {
            robot.decreaseLinearSlidePosition(Settings.LINEAR_SLIDE_POWER);
        }
    }

    private void armControl() {
        double newArmPosition = robot.getArmPosition();

        if (gamepad1RightTrigger != 0) {
            newArmPosition -= (gamepad1RightTrigger / 300);
        } else if (gamepad1LeftTrigger != 0) {
            newArmPosition += (gamepad1LeftTrigger / 300);
        }

        newArmPosition = clamp(newArmPosition, Settings.ARM_OUT_POSITION, Settings.ARM_IN_POSITION);
        robot.setArmPosition(newArmPosition);
    }

    private void wristControl() {
        if (controllerToggles2.isLeftBumperToggled()) {
            robot.increaseWristPosition();
        } else if (controllerToggles2.isRightBumperToggled()) {
            robot.decreaseWristPosition();
        }
    }

    private void intakeControl() {
        // multiplying by Settings.MAX_INTAKE_POWER normalizes the trigger values to [0, MAX_INTAKE_POWER]
        if (gamepad2RightTrigger != 0) {
            robot.intake.setPower(gamepad2RightTrigger * Settings.MAX_INTAKE_POWER);
        } else if (gamepad2LeftTrigger != 0) {
            robot.intake.setPower(-(gamepad2LeftTrigger * Settings.MAX_INTAKE_POWER));
        } else {
            robot.intake.setPower(0);
        }
    }

    private void bucketControl() {
        if (gamepad2.triangle) {
            robot.bucket.setPosition(Settings.BUCKET_RELEASE_POSITION);
        } else if (gamepad2.cross) {
            robot.bucket.setPosition(Settings.BUCKET_PICKUP_POSITION);
        }
    }

    private void clawControl() {
        if (gamepad2.dpad_left) {
            robot.claw.setPosition(Settings.CLAW_CLOSED_POSITION);
        } else if (gamepad2.dpad_right) {
            robot.claw.setPosition(Settings.CLAW_OPEN_POSITION);
        }
    }

    private void updateTelemetryData() {
        telemetry.addData("Controller Data ", "-----")
                .addData("Left Pad Y: ", straight)
                .addData("Left Pad X: ", strafe)
                .addData("Right Pad X: ", turn)

                .addData("Drive Data ", "-----")
                .addData("Front Left Power: ", powers[0])
                .addData("Back Left Power: ", powers[1])
                .addData("Front Right Power: ", powers[2])
                .addData("Back Right Power: ", powers[3])
                .addData("Heading: ", Math.toDegrees(heading) + 180)

                .addData("Subsystem Data ", "-----")
                .addData("Slide Position Index: ", robot.getLinearSlideIndex())
                .addData("Wrist Position Index: ", robot.getWristIndex())
                .addData("Arm Position: ", robot.getArmPosition());

        telemetry.update();
    }

    private void updateControllers() {
        controllerToggles1.update(gamepad1);
        controllerToggles2.update(gamepad2);
    }
}
