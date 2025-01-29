package org.firstinspires.ftc.teamcode.testing;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.Robot;

@TeleOp(name = "Odometry Push Logger", group = "Testing")
public class OdometryPushLogger extends LinearOpMode {
    Robot robot = new Robot(this);

    @Override
    public void runOpMode() throws InterruptedException {
        robot.init();
        waitForStart();

        robot.driveBase.frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        robot.driveBase.frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        robot.driveBase.backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        robot.driveBase.backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        while (opModeIsActive()) {
            telemetry.addData("Front Left Wheel Position . . . . .", robot.driveBase.frontLeft.getCurrentPosition());
            telemetry.addData("Front Right Wheel Position. . . . .", robot.driveBase.frontRight.getCurrentPosition());
            telemetry.addData("Back Left Wheel Position. . . . . .", robot.driveBase.backLeft.getCurrentPosition());
            telemetry.addData("Back Right Wheel Position . . . . .", robot.driveBase.backRight.getCurrentPosition());

            telemetry.addData("--------------------", "--------------------");

            telemetry.addData("Heading . . . . . . . . . . . . . .", robot.imu.getHeading(AngleUnit.DEGREES));

            telemetry.addData("--------------------", "--------------------");

            telemetry.addData("Horizontal Odometry Wheel Position ", robot.driveBase.horizontalEncoder.getCurrentPosition());
            telemetry.addData("Vertical Odometry Wheel Position. .", robot.driveBase.verticalEncoder.getCurrentPosition());


            telemetry.update();
        }
    }
}
