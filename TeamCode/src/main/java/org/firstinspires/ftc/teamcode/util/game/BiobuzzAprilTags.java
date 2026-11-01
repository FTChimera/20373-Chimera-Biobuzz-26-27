package org.firstinspires.ftc.teamcode.util.game;

import org.firstinspires.ftc.teamcode.util.AllianceColor;

public class BiobuzzAprilTags {

    public static final Cluster RED_OPPOSITE = new Cluster(0, 1, 2, 3);
    public static final Cluster RED_AUDIENCE = new Cluster(4, 5, 6, 7);

    public static final Cluster BLUE_OPPOSITE = new Cluster(42, 43, 44, 45);
    public static final Cluster BLUE_AUDIENCE = new Cluster(38, 39, 40, 41);

    public final Cluster opposite, alliance;

    public BiobuzzAprilTags(AllianceColor allianceColor) {
        opposite = allianceColor.isRed()? RED_OPPOSITE:BLUE_OPPOSITE;
        alliance = allianceColor.isRed()? RED_AUDIENCE:BLUE_AUDIENCE;
    }


    public static class Cluster {
        final int t0, t1, t2, t3;
        public Cluster(int t0, int t1, int t2, int t3) {
            this.t0 = t0;
            this.t1 = t1;
            this.t2 = t2;
            this.t3 = t3;
        }
        public Cluster(int[] t) {
            this(t[0], t[1], t[2], t[3]);
        }
        public int[] getTags() {
            return new int[]{t0, t1, t2, t3};
        }
        public int getTag(int i) {
            switch (i) {
                case 0: return t0;
                case 1: return t1;
                case 2: return t2;
                case 3: return t3;
                default: throw new IllegalArgumentException("Index must be between 0 and 3");
            }
        }

        public int[] getMidTwoTags() {
            return new int[]{t1, t2};
        }
    }
}
