package com.example.meepmeep;

import com.acmerobotics.roadrunner.geometry.Pose2d;

import org.rowlandhall.meepmeep.MeepMeep;
import org.rowlandhall.meepmeep.roadrunner.DefaultBotBuilder;
import org.rowlandhall.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class SampleAutoBetterIntake {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(1000);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                .setDimensions(18, 18)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .followTrajectorySequence(drive -> drive.trajectorySequenceBuilder(new Pose2d(-24, -72 + 15.0/2, Math.toRadians(0)))
                        .setTangent(Math.toRadians(90))
                        .splineToSplineHeading(new Pose2d(-53, -53, Math.toRadians(45)), Math.toRadians(215))

                        .waitSeconds(2)

                        .splineToLinearHeading(new Pose2d(-48, -40, Math.toRadians(90)), Math.toRadians(90))

                        .waitSeconds(2)

                        .setReversed(true)
                        .splineToLinearHeading(new Pose2d(-53, -53, Math.toRadians(45)), Math.toRadians(215))

                        .waitSeconds(2)

                        .setTangent(Math.toRadians(135))
                        .splineToLinearHeading(new Pose2d(-58, -40, Math.toRadians(90)), Math.toRadians(90))

                        .waitSeconds(2)

                        .setReversed(true)
                        .splineToLinearHeading(new Pose2d(-53, -53, Math.toRadians(45)), Math.toRadians(215))

                        .waitSeconds(2)

                        .setReversed(false)
                        .splineToLinearHeading(new Pose2d(-55, -25, Math.toRadians(180)), Math.toRadians(90))

                        .waitSeconds(2)

                        .setReversed(true)
                        .splineToLinearHeading(new Pose2d(-53, -53, Math.toRadians(45)), Math.toRadians(215))

                        .waitSeconds(2)

                        .setReversed(false)
                        .splineToSplineHeading(new Pose2d(-24, -0, Math.toRadians(0)), Math.toRadians(0))
                        .build());


        meepMeep.setBackground(MeepMeep.Background.FIELD_INTOTHEDEEP_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}