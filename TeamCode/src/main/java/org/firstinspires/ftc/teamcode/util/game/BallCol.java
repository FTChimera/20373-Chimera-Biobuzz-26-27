package org.firstinspires.ftc.teamcode.util.game;


import com.qualcomm.robotcore.hardware.NormalizedRGBA;

import org.firstinspires.ftc.teamcode.util.AllianceColor;
import org.firstinspires.ftc.teamcode.util.math.MathUtils;

public enum BallCol {
    NECTAR_RED(Ball.NECTAR),
    NECTAR_BLUE(Ball.NECTAR),
    POLLEN_YELLOW(Ball.POLLEN),
    UNKNOWN(Ball.UNKNOWN);
    public final Ball ball;

    BallCol(Ball ball) {
        this.ball = ball;
    }

    public static BallCol compose(BallCol ballCol, BallCol ballCol1, AllianceColor priority) {
        if (ballCol==UNKNOWN) return ballCol1;
        if (ballCol1==UNKNOWN) return ballCol;
        BallCol opposite = priority.isRed() ? NECTAR_BLUE : NECTAR_RED;
        BallCol alliance = priority.isRed() ? NECTAR_RED : NECTAR_BLUE;
        if (ballCol==opposite) return ballCol;
        if (ballCol1==opposite) return ballCol1;
        if (ballCol==alliance) return ballCol;
        if (ballCol1==alliance) return ballCol1;
        return ballCol;
    }

    public static BallCol detect(NormalizedRGBA colors, double costThreshold) { // 0.06
        float r = colors.red;
        float g = colors.green;
        float b = colors.blue;
        float sum = r + g + b;

        if (sum < MathUtils.EPSILON) return UNKNOWN;
        double rn = r/sum;
        double gn = g / sum;
        double bn = b / sum;

        RGB most = (rn > gn) ? (rn > bn ? RGB.RED : (bn > gn? RGB.BLUE : RGB.GREEN)) : (gn > bn ? RGB.GREEN : RGB.BLUE);
        double maxVal = most==RGB.RED?rn:most==RGB.GREEN?gn:bn;
        double secondBest = most==RGB.RED?Math.max(gn, bn) : most==RGB.GREEN?Math.max(rn, bn) : Math.max(rn, gn);
//        RGB second = secondBest==rn? RGB.RED : secondBest==gn ? RGB.GREEN : RGB.BLUE;
        double cost = maxVal - secondBest;

        if (most == RGB.BLUE && cost > costThreshold) return NECTAR_BLUE;
        if (most == RGB.RED && cost > costThreshold) return NECTAR_RED;
        if (Math.min(rn, gn) - bn > costThreshold) {
            return POLLEN_YELLOW;
        }

        return UNKNOWN;
    }
    private enum RGB {RED, GREEN, BLUE}
}
