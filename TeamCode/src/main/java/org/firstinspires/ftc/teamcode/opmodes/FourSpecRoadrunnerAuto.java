// start with the line on the middle

package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.roadrunner.AngularVelConstraint;
import com.acmerobotics.roadrunner.CompositeVelConstraint;
import com.acmerobotics.roadrunner.MinVelConstraint;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.VelConstraint;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.Robot;

import java.util.Arrays;


@Autonomous(name = "FourSpec", group = "Auto")
public class FourSpecRoadrunnerAuto extends LinearOpMode {
    Robot robot = new Robot(this);

    MinVelConstraint forcedSlowVelocity = new MinVelConstraint(Arrays.asList(
            new TranslationalVelConstraint(40.0),
            new AngularVelConstraint(Math.PI / 4)
    ));

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
                        .strafeTo(new Vector2d(0, -34))

                        // deposit first spec
                        .stopAndAdd(robot.hangSpecimen())

                        // push leftmost spec to observation zone
                        .setReversed(false)
                        .splineToConstantHeading(new Vector2d(37, -40), Math.toRadians(90))
                        .strafeTo(new Vector2d(37, -10))
                        .splineToSplineHeading(new Pose2d(42.75, -2, Math.toRadians(0)), Math.toRadians(0), new AngularVelConstraint(Math.PI / 1.5))
                        .splineToSplineHeading(new Pose2d(48, -10, Math.toRadians(90)), Math.toRadians(270), new AngularVelConstraint(Math.PI / 1.5))
                        .splineToConstantHeading(new Vector2d(46, -57), Math.toRadians(270))

                        // push middle spec to observation zone
                        .strafeTo(new Vector2d(46, -15))
                        .splineToConstantHeading(new Vector2d(49.5, -8), Math.toRadians(0), new TranslationalVelConstraint(40.0))
                        .splineToConstantHeading(new Vector2d(53, -15), Math.toRadians(270), new TranslationalVelConstraint(40.0))
                        .strafeTo(new Vector2d(53, -54))

                        // pickup spec #2
                        .setTangent(Math.toRadians(125))
                        .splineToConstantHeading(new Vector2d(38, -72 + 3), Math.toRadians(280), new TranslationalVelConstraint(40.0))
                        .stopAndAdd(robot.pickupSpecimen())
                        .afterTime(0.2, robot.linearSlide.raiseLinearSlideToHangSpecimen())

                        // hang spec #2
                        .setTangent(Math.toRadians(135))
                        .splineToSplineHeading(new Pose2d(-5, -45, Math.toRadians(-90)), Math.toRadians(90))
                        .strafeTo(new Vector2d(-5, -34), forcedSlowVelocity)
                        .stopAndAdd(robot.hangSpecimen())

                        // pickup spec #3
                        .setReversed(false)
                        .splineToSplineHeading(new Pose2d(40, -72 + 15, Math.toRadians(90)), Math.toRadians(270))
                        .strafeTo(new Vector2d(40, -72 + 3), forcedSlowVelocity)
                        .stopAndAdd(robot.pickupSpecimen())
                        .afterTime(0.2, robot.linearSlide.raiseLinearSlideToHangSpecimen())

                        // hang spec #3
                        .setTangent(Math.toRadians(135))
                        .splineToSplineHeading(new Pose2d(-2.5, -45, Math.toRadians(-90)), Math.toRadians(90))
                        .strafeTo(new Vector2d(-2.5, -34), forcedSlowVelocity)
                        .stopAndAdd(robot.hangSpecimen())

//                        // pickup spec #4
//                        .splineToSplineHeading(new Pose2d(40, -72 + 9, Math.toRadians(90)), Math.toRadians(270))
//                        .stopAndAdd(robot.pickupSpecimen())
//                        .afterTime(0.2, robot.linearSlide.raiseLinearSlideToHangSpecimen())
//
//                        // hang spec #4
//                        .setReversed(false)
//                        .splineToSplineHeading(new Pose2d(-1.5, -35, Math.toRadians(270)), Math.toRadians(90))
//                        .stopAndAdd(robot.hangSpecimen())
//
//                        // park
//                        .strafeTo(new Vector2d(60, -55))
                        .build());
    }
}