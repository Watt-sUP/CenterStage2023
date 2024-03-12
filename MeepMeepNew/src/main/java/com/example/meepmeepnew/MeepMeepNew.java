package com.example.meepmeepnew;

import com.acmerobotics.roadrunner.Pose2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.core.colorscheme.scheme.ColorSchemeRedDark;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepNew {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(640, 144);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                .setColorScheme(new ColorSchemeRedDark())
                .setDimensions(12.89370078740157, 15.19685039370079)
                .setConstraints(55, 50, Math.toRadians(270), Math.toRadians(225), 10.4)
                .build();

        myBot.runAction(
                myBot.getDrive().actionBuilder(new Pose2d(16.75, -62.75, Math.toRadians(90)))
//                        .splineTo(new Vector2d(9, -41.5), Math.toRadians(135)) left
//                        .splineTo(new Vector2d(15, -38), Math.toRadians(90)) middle
//                        .splineTo(new Vector2d(17, -44.25), Math.toRadians(60)) right
                        .build()
        );

        meepMeep.setBackground(MeepMeep.Background.FIELD_CENTERSTAGE_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                // Add both of our declared bot entities
                .addEntity(myBot)
                .start();
    }
}