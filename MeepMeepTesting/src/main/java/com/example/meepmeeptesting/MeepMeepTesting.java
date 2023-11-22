package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTesting {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(49, 50, Math.toRadians(214.78926857142858), Math.toRadians(214.78926857142858), 12.68)
                .setDimensions(16, 18)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(8, -63, Math.toRadians(90)))
                                .lineToLinearHeading(new Pose2d(10, -26, Math.toRadians(0)))
                                .lineTo(new Vector2d(11, -26))
                                .lineTo(new Vector2d(10, -32))
                                .splineToConstantHeading(new Vector2d(10, -48), Math.toRadians(0))
                                .splineToSplineHeading(new Pose2d(48, -32, Math.toRadians(180)), Math.toRadians(0))

                                .build()
                );

        meepMeep.setBackground(MeepMeep.Background.FIELD_CENTERSTAGE_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}