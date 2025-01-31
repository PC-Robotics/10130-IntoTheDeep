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

                        .setReversed(false)
                        .splineToSplineHeading(new Pose2d(-3, -35, Math.toRadians(270)), Math.toRadians(90))

                        .splineToSplineHeading(new Pose2d(40, -72 + 9, Math.toRadians(90)), Math.toRadians(270))

                        .setReversed(false)
                        .splineToSplineHeading(new Pose2d(3, -35, Math.toRadians(270)), Math.toRadians(90))

                        .splineToSplineHeading(new Pose2d(40, -72 + 9, Math.toRadians(90)), Math.toRadians(270))

                        .setReversed(false)
                        .splineToSplineHeading(new Pose2d(-1.5, -35, Math.toRadians(270)), Math.toRadians(90))

                        .strafeTo(new Vector2d(60, -55))
                        .build());

        meepMeep.setBackground(org.rowlandhall.meepmeep.MeepMeep.Background.FIELD_INTOTHEDEEP_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}
