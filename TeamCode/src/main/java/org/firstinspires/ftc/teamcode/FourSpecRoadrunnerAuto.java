package org.firstinspires.ftc.teamcode;

import androidx.annotation.NonNull;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;


@Autonomous(name = "FourSpec", group = "Auto")
public class FourSpecRoadrunnerAuto extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Pose2d startingPos = new Pose2d(0, -72 + 15.0 / 2, Math.toRadians(270));
        MecanumDrive drive = new MecanumDrive(hardwareMap, startingPos);

        waitForStart();

        Actions.runBlocking(
                drive.actionBuilder(startingPos)
                        .strafeTo(new Vector2d(0, -35))

                        .setReversed(false)
                        .splineToConstantHeading(new Vector2d(33, -40), Math.toRadians(90))
                        .splineToSplineHeading(new Pose2d(34, -13, Math.toRadians(10)), Math.toRadians(90))
                        .splineToSplineHeading(new Pose2d(42, -10, Math.toRadians(90)), Math.toRadians(0))
                        .splineToConstantHeading(new Vector2d(48, -15), Math.toRadians(270))
                        .strafeTo(new Vector2d(46, -55))

                        .strafeTo(new Vector2d(46, -15))
                        .splineToConstantHeading(new Vector2d(57, -15), Math.toRadians(270))
                        .strafeTo(new Vector2d(54, -54))

                        .setTangent(Math.toRadians(60))
                        .splineToConstantHeading(new Vector2d(40, -72 + 9), Math.toRadians(270))

                        .waitSeconds(1)

                        .setReversed(false)
                        .splineToSplineHeading(new Pose2d(0, -35, Math.toRadians(270)), Math.toRadians(90))
                        .waitSeconds(1)
                        .splineToSplineHeading(new Pose2d(40, -72 + 9, Math.toRadians(90)), Math.toRadians(270))

                        .waitSeconds(1)

                        .setReversed(false)
                        .splineToSplineHeading(new Pose2d(0, -35, Math.toRadians(270)), Math.toRadians(90))
                        .waitSeconds(1)
                        .splineToSplineHeading(new Pose2d(40, -72 + 9, Math.toRadians(90)), Math.toRadians(270))

                        .waitSeconds(1)

                        .setReversed(false)
                        .splineToSplineHeading(new Pose2d(0, -35, Math.toRadians(270)), Math.toRadians(90))
                        .waitSeconds(1)
                        .strafeTo(new Vector2d(60, -55))
                        .build());
    }
}