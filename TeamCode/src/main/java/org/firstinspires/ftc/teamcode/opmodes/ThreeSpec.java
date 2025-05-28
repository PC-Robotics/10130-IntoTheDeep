// start with the line on the middle

package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.roadrunner.AngularVelConstraint;
import com.acmerobotics.roadrunner.MinVelConstraint;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.ProfileAccelConstraint;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.Robot;

import java.util.Arrays;


@Autonomous(name = "3 Specimen", group = "Autonomous")
public class ThreeSpec extends LinearOpMode {
    Robot robot = new Robot(this);

    MinVelConstraint forcedSlowVelocity = new MinVelConstraint(Arrays.asList(
            new TranslationalVelConstraint(40.0),
            new AngularVelConstraint(Math.PI / 4)
    ));

    MinVelConstraint normalVelocity = new MinVelConstraint(Arrays.asList(
            new TranslationalVelConstraint(80.0),
            new AngularVelConstraint(Math.PI)
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
                        .splineToConstantHeading(new Vector2d(35, -40), Math.toRadians(90))
                        .strafeTo(new Vector2d(35, -12))
                        // TODO - robot jiggles a bit here. maybe force slower velocity or make it splineToLinearHeading without losing angular velocity continuity
                        .splineToSplineHeading(new Pose2d(41.5, -5, Math.toRadians(0)), Math.toRadians(0), new AngularVelConstraint(Math.PI / 1.2))
                        .splineToSplineHeading(new Pose2d(48, -12, Math.toRadians(90)), Math.toRadians(270), new AngularVelConstraint(Math.PI / 1.2))
                        .splineToConstantHeading(new Vector2d(53, -57), Math.toRadians(270), normalVelocity)

//                        // push middle spec to observation zone
//                        .strafeTo(new Vector2d(46, -15))
//                        .splineToConstantHeading(new Vector2d(50.5, -8), Math.toRadians(0), new TranslationalVelConstraint(50.0))
//                        .splineToConstantHeading(new Vector2d(55, -15), Math.toRadians(270), new TranslationalVelConstraint(50.0))
//                        .strafeTo(new Vector2d(58, -54), normalVelocity)

                        // pickup spec #2
                        // TODO - middle spec stays in front of robot and stops wheels from touching wall
                        .strafeTo(new Vector2d(53, -49))
                        .splineToConstantHeading(new Vector2d(48, -49), Math.toRadians(135), new TranslationalVelConstraint(50.0))
                        .splineToConstantHeading(new Vector2d(42, -72 + 3), Math.toRadians(280), new TranslationalVelConstraint(50.0))
                        .stopAndAdd(robot.pickupSpecimen())
                        .afterTime(0.2, robot.linearSlide.raiseLinearSlideToHangSpecimen())

                        // hang spec #2
                        .setTangent(Math.toRadians(135))
                        .splineToSplineHeading(new Pose2d(-5, -45, Math.toRadians(-90)), Math.toRadians(90), normalVelocity)
                        .strafeTo(new Vector2d(-5, -34), forcedSlowVelocity)
                        .stopAndAdd(robot.hangSpecimen())

                        // pickup spec #3
                        .setReversed(false)
                        .splineToSplineHeading(new Pose2d(38, -50, Math.toRadians(90)), Math.toRadians(0), normalVelocity)
                        .splineToConstantHeading(new Vector2d(42, -72 + 15), Math.toRadians(-90))
                        // TODO - not accurate enough
                        .strafeTo(new Vector2d(42, -72 + 3))
                        .stopAndAdd(robot.pickupSpecimen())
                        .afterTime(0.2, robot.linearSlide.raiseLinearSlideToHangSpecimen())

                        // hang spec #3
                        .setTangent(Math.toRadians(135))
                        // TODO - -2.5 is dangerously close to other specimens. maybe -7.5 or make robot slide a bit before placing specimen
                        .splineToSplineHeading(new Pose2d(-2.5, -45, Math.toRadians(-90)), Math.toRadians(90))
                        .strafeTo(new Vector2d(-2.5, -34), forcedSlowVelocity)
                        .stopAndAdd(robot.hangSpecimen())

                        // park TODO - not tested
                        .splineToConstantHeading(new Vector2d(65, -65), Math.toRadians(0), new TranslationalVelConstraint(100), new ProfileAccelConstraint(-100, 100))


                        .build());
    }
}