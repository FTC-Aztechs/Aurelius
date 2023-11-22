package com.example.mymeepmeep;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;


public class MyMeepMeep {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);
        Pose2d StartPos = new Pose2d(12,62,Math.toRadians(-90));

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(0 + StartPos.getX(), 0 + StartPos.getY(), Math.toRadians(0) + StartPos.getHeading()))
                                .splineToLinearHeading(new Pose2d(0 + StartPos.getX(), -28 + StartPos.getY(), Math.toRadians(-90) + StartPos.getHeading()), Math.toRadians(230))
                                .setReversed(true)
                                .splineToLinearHeading(new Pose2d(37 + StartPos.getX(), -33 + StartPos.getY(), Math.toRadians(-90) + StartPos.getHeading()), Math.toRadians(0))
                                .strafeTo(new Vector2d(37 + StartPos.getX(), -5 + StartPos.getY()))
                                .build()
                );

        meepMeep.setBackground(MeepMeep.Background.FIELD_CENTERSTAGE_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}