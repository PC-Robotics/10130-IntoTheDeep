package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;


@Autonomous(name = "FourSpec", group = "Auto")
public class FourSpecRoadrunnerAuto extends LinearOpMode {
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

        Pose2d initialPose = new Pose2d(0, -72 + 15.0 / 2, Math.toRadians(270));
        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);

        waitForStart();

        Actions.runBlocking(
                drive.actionBuilder(initialPose)
                        // go to bar to deposit first spec
                        .afterTime(0.1, robot.linearSlide.raiseLinearSlideToHangSpecimen())
                        .strafeTo(new Vector2d(0, -35))

                        // deposit first spec
                        .stopAndAdd(robot.hangSpecimen())

                        // push leftmost spec to observation zone
                        .setReversed(false)
                        .splineToConstantHeading(new Vector2d(33, -40), Math.toRadians(90))
                        .splineToSplineHeading(new Pose2d(34, -13, Math.toRadians(10)), Math.toRadians(90))
                        .splineToSplineHeading(new Pose2d(42, -10, Math.toRadians(90)), Math.toRadians(0))
                        .splineToConstantHeading(new Vector2d(48, -15), Math.toRadians(270))
                        .strafeTo(new Vector2d(46, -55))

                        // push middle spec to observation zone
                        .strafeTo(new Vector2d(46, -15))
                        .splineToConstantHeading(new Vector2d(57, -15), Math.toRadians(270))
                        .strafeTo(new Vector2d(54, -54))

                        // pickup spec #2
                        .setTangent(Math.toRadians(60))
                        .splineToConstantHeading(new Vector2d(40, -72 + 9), Math.toRadians(270))
                        .stopAndAdd(robot.pickupSpecimen())
                        .afterTime(0.2, robot.linearSlide.raiseLinearSlideToHangSpecimen())

                        // hang spec #2
                        .setReversed(false)
                        .splineToSplineHeading(new Pose2d(-3, -35, Math.toRadians(270)), Math.toRadians(90))
                        .stopAndAdd(robot.hangSpecimen())

                        // pickup spec #3
                        .splineToSplineHeading(new Pose2d(40, -72 + 9, Math.toRadians(90)), Math.toRadians(270))
                        .stopAndAdd(robot.pickupSpecimen())
                        .afterTime(0.2, robot.linearSlide.raiseLinearSlideToHangSpecimen())

                        // hang spec #3
                        .setReversed(false)
                        .splineToSplineHeading(new Pose2d(3, -35, Math.toRadians(270)), Math.toRadians(90))
                        .stopAndAdd(robot.hangSpecimen())

                        // pickup spec #4
                        .splineToSplineHeading(new Pose2d(40, -72 + 9, Math.toRadians(90)), Math.toRadians(270))
                        .stopAndAdd(robot.pickupSpecimen())
                        .afterTime(0.2, robot.linearSlide.raiseLinearSlideToHangSpecimen())

                        // hang spec #4
                        .setReversed(false)
                        .splineToSplineHeading(new Pose2d(-1.5, -35, Math.toRadians(270)), Math.toRadians(90))
                        .stopAndAdd(robot.hangSpecimen())

                        // park
                        .strafeTo(new Vector2d(60, -55))
                        .build());
    }
}