package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;

import org.rowlandhall.meepmeep.MeepMeep;
import org.rowlandhall.meepmeep.roadrunner.DefaultBotBuilder;
import org.rowlandhall.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTesting {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(600);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(30.4224324932042, 30.4224324932042, Math.toRadians(217.37504603407623), Math.toRadians(158.50799999999998), 12.37)
                .setDimensions(16, 18)
                .followTrajectorySequence(drive -> drive.trajectorySequenceBuilder(new Pose2d(-37.5, -61, Math.toRadians(-180)))
                        .lineToLinearHeading(new Pose2d(-56, -55, Math.toRadians(-135)))
                        .lineToLinearHeading(new Pose2d(-48, -40, Math.toRadians(-90)))
                        .lineToLinearHeading(new Pose2d(-56, -55, Math.toRadians(-135)))
                        .lineToLinearHeading(new Pose2d(-58, -40, Math.toRadians(-90)))
                        .lineToLinearHeading(new Pose2d(-56, -55, Math.toRadians(-135)))
                        .lineToLinearHeading(new Pose2d(-55, -40, Math.toRadians(-45)))
                        .lineToLinearHeading(new Pose2d(-56, -55, Math.toRadians(-135)))
                        .build());


        meepMeep.setBackground(MeepMeep.Background.FIELD_INTOTHEDEEP_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}