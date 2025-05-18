// start with the line on the middle

package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.Robot;


@Autonomous(name = "Testing", group = "Auto")
public class MainAutonomous extends LinearOpMode {
    Robot robot = new Robot(this);

    @Override
    public void runOpMode() throws InterruptedException {
        robot.init();

        robot.claw.open();
        sleep(2000);
        robot.claw.close();

        robot.imu.resetYaw();

        robot.claw.start();
        robot.wrist.start();
        robot.bucket.start();
        robot.linearSlide.start();

        Pose2d initialPose = new Pose2d(40, -72 + 15.0 / 2, Math.toRadians(90));
        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);

        waitForStart();

        Actions.runBlocking(
                drive.actionBuilder(initialPose)
                        // hang spec #2
                        .splineToLinearHeading(new Pose2d(0, -45, Math.toRadians(-90)), Math.toRadians(90))
//                        .strafeTo(new Vector2d(0, -35))
                        .stopAndAdd(robot.hangSpecimen())
                        .build());
    }
}