package com.example.meepmeep;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;

import org.rowlandhall.meepmeep.MeepMeep;
import org.rowlandhall.meepmeep.roadrunner.DefaultBotBuilder;
import org.rowlandhall.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class FourSpecAuto {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(1000);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                .setDimensions(18, 18)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .followTrajectorySequence(drive -> drive.trajectorySequenceBuilder(new Pose2d(0, -72 + 15.0 / 2, Math.toRadians(270)))
                        .strafeTo(new Vector2d(0, -32))

                        .setReversed(false)
                        .splineToConstantHeading(new Vector2d(33, -40), Math.toRadians(90))
                        .splineToSplineHeading(new Pose2d(36, -15, Math.toRadians(10)), Math.toRadians(90))
                        .splineToSplineHeading(new Pose2d(41, -10, Math.toRadians(90)), Math.toRadians(0))
                        .splineToConstantHeading(new Vector2d(46, -15), Math.toRadians(270))
                        .strafeTo(new Vector2d(46, -55))

                        .strafeTo(new Vector2d(46, -15))
                        .splineToConstantHeading(new Vector2d(57, -15), Math.toRadians(270))
                        .strafeTo(new Vector2d(57, -50))

                        .setTangent(Math.toRadians(90))
                        .splineToConstantHeading(new Vector2d(40, -72 + 15.0 / 2), Math.toRadians(270))

                        .waitSeconds(1)

                        .setReversed(false)
                        .splineToSplineHeading(new Pose2d(0, -30, Math.toRadians(270)), Math.toRadians(90))
                        .waitSeconds(1)
                        .lineToSplineHeading(new Pose2d(40, -72 + 15.0 / 2, Math.toRadians(90)))

                        .waitSeconds(1)

                        .setReversed(false)
                        .splineToSplineHeading(new Pose2d(0, -30, Math.toRadians(270)), Math.toRadians(90))
                        .waitSeconds(1)
                        .lineToSplineHeading(new Pose2d(40, -72 + 15.0 / 2, Math.toRadians(90)))

                        .waitSeconds(1)

                        .setReversed(false)
                        .splineToSplineHeading(new Pose2d(0, -30, Math.toRadians(270)), Math.toRadians(90))
                        .waitSeconds(1)
                        .strafeTo(new Vector2d(40, -50))
                        .build());

        meepMeep.setBackground(org.rowlandhall.meepmeep.MeepMeep.Background.FIELD_INTOTHEDEEP_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}
