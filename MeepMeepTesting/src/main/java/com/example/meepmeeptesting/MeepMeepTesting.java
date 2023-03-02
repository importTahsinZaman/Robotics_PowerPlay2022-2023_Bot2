package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.core.colorscheme.scheme.ColorSchemeBlueLight;
import com.noahbres.meepmeep.core.colorscheme.scheme.ColorSchemeRedDark;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTesting {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(52.48291908330528, 52.48291908330528, 2.9370625019, Math.toRadians(187.94061000000002), 12)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(36, -60, Math.toRadians(-180)))
                                .strafeRight(60)
                                .waitSeconds(.8)
                                .turn(Math.toRadians(140))
                                .splineTo(new Vector2d(58, -12), Math.toRadians(0))
                                .waitSeconds(.2)
                                .turn(Math.toRadians(160))
                                .splineTo(new Vector2d(36, 0), Math.toRadians(-180))
//                                .splineTo(new Vector2d(36, 0), Math.toRadians(-180))
//                                .waitSeconds(.2)
//                                .splineTo(new Vector2d(58, -12), Math.toRadians(0))
//                                .waitSeconds(.2) 120, 4
                                .build()
                );

        meepMeep.setBackground(MeepMeep.Background.FIELD_POWERPLAY_KAI_DARK)
//                .setTheme(new ColorSchemeBlueLight())
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}