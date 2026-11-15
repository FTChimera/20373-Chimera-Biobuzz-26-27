package org.firstinspires.ftc.teamcode.planners.modules;

import static com.pedropathing.ivy.groups.Groups.*;
import static com.pedropathing.ivy.commands.Commands.*;

import static org.firstinspires.ftc.teamcode.planners.commands.DrivetrainCommands.follow;

import androidx.annotation.NonNull;

import com.pedropathing.api.Paths;
import com.pedropathing.ivy.Command;
import com.pedropathing.paths.Path;

import org.firstinspires.ftc.teamcode.mechanisms.Robot;
import org.firstinspires.ftc.teamcode.util.game.Field;
import org.firstinspires.ftc.teamcode.planners.CommonKt;
import org.firstinspires.ftc.teamcode.planners.IntakePoses;
import org.firstinspires.ftc.teamcode.planners.PathPlanner;
import org.firstinspires.ftc.teamcode.planners.PathPoses;
import org.firstinspires.ftc.teamcode.planners.ShootPoses;
import org.firstinspires.ftc.teamcode.planners.Timings;
import org.firstinspires.ftc.teamcode.planners.Values;
import org.firstinspires.ftc.teamcode.planners.commands.DrivetrainCommands;
import org.firstinspires.ftc.teamcode.planners.commands.ShooterCommands;
import org.firstinspires.ftc.teamcode.util.poses.Pose2d;

public class FromShootLoadingPose {
    public static class ToIntakeFromNorthFlower extends PathPlanner {
        Path toFlower;
        Path back;
        public ToIntakeFromNorthFlower(Robot robot) {
            super(robot);
        }

        @Override
        public Pose2d getEndPose() {
            return ShootPoses.loading;
        }

        @Override
        public Command run() {
            return sequential(
                    instant(robot.intake::setFullPower),
                    parallel(follow(robot.follower, () -> toFlower), DrivetrainCommands.runAtCompletion(robot, Values.flowerIntakeDeployTValue, () -> robot.intake.setDeployed(true))),
                    waitMs(Timings.flowerIntakeTime * 1000),
                    parallel(follow(robot.follower, () -> back), DrivetrainCommands.runAtCompletion(robot, Values.flowerIntakeStowTValue, () -> robot.intake.setDeployed(false))),
                    ShooterCommands.waitForHiveLineup(robot, Field.HiveState.TIPPED),
                    ShooterCommands.shootAllPollen(robot)
            );
        }

        @Override
        public void buildPaths() {
            // Short hop north; rect clearance to the west hive base: 15.3 in.
            toFlower = Paths
                    .curve(startPose, PathPoses.northFlowerFromLoading, IntakePoses.northFlower)
                    .heading(CommonKt.fastInterpolationDegraded(IntakePoses.northFlower.heading(), Values.gardenIntakeConstantTValue, false));
            back = Paths
                    .curve(IntakePoses.northFlower, PathPoses.northFlowerFromLoading, ShootPoses.loading)
                    .heading(CommonKt.fastInterpolationDegraded(ShootPoses.loading.heading(), Values.gardenIntakeConstantTValue, true));
        }

        @NonNull
        @Override
        public String toString() {
            return "From Shoot Loading Pose to Intake North Flower";
        }
    }

    /**
     * Intake the alliance partner's preloads from the loading zone, then
     * shoot them at the garden. Rule (G304.G, TU09): a present partner's
     * preloads stay in contact with the partner's robot, so there is
     * nothing to collect here; a NO-SHOW partner's 4 POLLEN are staged
     * in the loading zone against the perimeter wall, and the robot may
     * collect them (no rule against picking up a partner's POLLEN; G407
     * caps possession at 4, so the robot must already have shot its own
     * preloads). One branch handles both cases: drive to the wall,
     * intake for 2 s, then garden shoot. If the partner was present the
     * intake runs empty and the garden shoot finds nothing to shoot.
     *
     * Extends ToIntakeWestFlowerToGarden: the run sequence (intake +
     * deploy, 2 s, return + stow, lineup UNTIPPED, shoot), the lineup
     * state and the end pose are identical, so only the two legs are
     * replaced -- the inherited {@code toFlower} field holds the wall
     * approach and {@code back} the return down the west side.
     */
    public static class ToIntakeLoadingToGarden extends ToIntakeWestFlowerToGarden {
        public ToIntakeLoadingToGarden(Robot robot) {
            super(robot);
        }

        @Override
        public void buildPaths() {
            super.buildPaths();
            // Loading intake pose: robot center 9.5 in from the west wall
            // (1 in + half the robot length, same standoff as a flower),
            // centered in the loading zone (y 96-120), facing the wall.
            // Short hop west from the loading shoot pose.
            toFlower = Paths
                    .curve(startPose, PathPoses.loadingIntakeApproach, PathPoses.loadingIntake)
                    .heading(CommonKt.fastInterpolationDegraded(PathPoses.loadingIntake.heading(), Values.gardenIntakeConstantTValue, false));
            // Back to the garden shoot pose: down the west side past the
            // hive, then east along the bottom. Rect clearance to the
            // west hive base stays west of x=45 until below y=50
            // (curve passes (26.4, 48.3) at t=0.5, (41.1, 32.1) at
            // t=0.75). Reverse tangential interpolation, as on the
            // other return legs.
            back = Paths
                    .curve(PathPoses.loadingIntake, PathPoses.loadingToGarden, ShootPoses.garden)
                    .heading(CommonKt.fastInterpolationDegraded(ShootPoses.garden.heading(), Values.gardenIntakeConstantTValue, true));
        }

        @NonNull
        @Override
        public String toString() {
            return "From Shoot Loading Pose to Intake Loading Preloads to Garden";
        }
    }

    /**
     * Intake the alliance partner's preloads from the loading zone, then
     * shoot them at the garden. Partner-PRESENT branch (G304.G, TU09):
     * a present partner's 4 POLLEN stay in contact with his robot, so he
     * parks at the north part of the loading zone (assumed centre (7, 130),
     * facing south) and his POLLEN sit in a row along the zone's north tape
     * (y = 120; 2.8 in balls, centres x = 1.4, 4.2, 7.0, 9.8) touching his
     * south face. No avoidance curve is needed: the approach comes from the
     * south and the sweep runs north up the zone, so the robot never goes
     * around the partner, and the partner's parking spot is one the park
     * pathfinder would probably never reach anyway.
     *
     * Geometry (red; blue mirrors via red()): approach from the loading
     * shoot pose down to the south end of the zone (8, 96), then sweep
     * straight north to (8, 112) with the intake deployed, settle 300 ms,
     * and return to the garden shoot pose. The robot is NOT a mass point:
     * its centre stops 8 in short of the north tape, but the intake front
     * edge (centre + half length 8.5) reaches y = 120.5, a little past the
     * tape and 1 in short of the partner's assumed south face (y = 121.5).
     * The sweep takes what the intake spans -- expect 3 of 4, the wall-side
     * ball (x = 1.4) sitting outside the intake's lateral reach -- it does
     * not aim at individual balls. If the partner parks further south or
     * the row sits elsewhere, move SWEEP_X / SWEEP_END_Y; the 1 in partner
     * gap is the number to re-check on the field.
     *
     * Keeps its own run(): unlike the flower planners it has three legs
     * (approach, sweep, return) and a 300 ms settle instead of a 2 s
     * dwell, so it does not extend the flower run sequence.
     */
    public static class ToIntakePartnerPreloadsToGarden extends PathPlanner {
        Path toSweepStart;
        Path sweep;
        Path toGarden;
        public ToIntakePartnerPreloadsToGarden(Robot robot) {
            super(robot);
        }

        @Override
        public Pose2d getEndPose() {
            return ShootPoses.garden;
        }

        @Override
        public Command run() {
            return sequential(
                    instant(robot.intake::setFullPower),
                    parallel(follow(robot.follower, () -> toSweepStart), DrivetrainCommands.runAtCompletion(robot, Values.flowerIntakeDeployTValue, () -> robot.intake.setDeployed(true))),
                    follow(robot.follower, () -> sweep),
                    waitMs(Timings.gardenIntakeTimeMs),
                    parallel(follow(robot.follower, () -> toGarden), DrivetrainCommands.runAtCompletion(robot, Values.flowerIntakeStowTValue, () -> robot.intake.setDeployed(false))),
                    ShooterCommands.waitForHiveLineup(robot, Field.HiveState.UNTIPPED),
                    ShooterCommands.shootAllPollen(robot)
            );
        }

        @Override
        public void buildPaths() {
            // Down to the south end of the loading zone, ending facing
            // north for the sweep. Disc clearance to the west hive base:
            // 7.1 in; to the partner: 13.6 in.
            toSweepStart = Paths
                    .curve(startPose, PathPoses.partnerApproach, PathPoses.partnerSweepStart)
                    .heading(CommonKt.fastInterpolationDegraded(PathPoses.partnerSweepStart.heading(), Values.gardenIntakeConstantTValue, false));
            // Straight north up the zone, intake deployed. Centre stops at
            // y = 112; front edge reaches y = 120.5 (past the north tape,
            // 1 in short of the partner's assumed south face). Disc
            // clearance to the partner at the end of the sweep: 1.0 in.
            sweep = Paths
                    .curve(PathPoses.partnerSweepStart, PathPoses.partnerSweepMid, PathPoses.partnerSweepEnd)
                    .heading(CommonKt.fastInterpolationDegraded(PathPoses.partnerSweepEnd.heading(), Values.gardenIntakeConstantTValue, false));
            // Back to the garden shoot pose: down the west side past the
            // hive, then east along the bottom. Disc clearance to the west
            // hive base: 7.4 in. Reverse tangential interpolation, as on
            // the other return legs.
            toGarden = Paths
                    .curve(PathPoses.partnerSweepEnd, PathPoses.loadingToGarden, ShootPoses.garden)
                    .heading(CommonKt.fastInterpolationDegraded(ShootPoses.garden.heading(), Values.gardenIntakeConstantTValue, true));
        }

        @NonNull
        @Override
        public String toString() {
            return "From Shoot Loading Pose to Intake Partner Preloads to Garden";
        }
    }

    /**
     * West flower intake from the loading shoot pose. Extends
     * ToIntakeFromWestFlower (garden shoot version): the return leg, the
     * run sequence, the lineup state (UNTIPPED) and the end pose are
     * identical, so only the outbound leg is replaced -- the approach
     * comes down the west side from the loading shoot pose instead of
     * out from the garden shoot pose.
     */
    public static class ToIntakeWestFlowerToGarden extends FromShootGardenPose.ToIntakeFromWestFlower {
        public ToIntakeWestFlowerToGarden(Robot robot) {
            super(robot);
        }

        @Override
        public void buildPaths() {
            super.buildPaths();
            // Long diagonal past the north-east corner of the west hive base;
            // rect clearance: 3.8 in out, 12.1 in back.
            toFlower = Paths
                    .curve(startPose, PathPoses.westFlowerFromLoading, IntakePoses.westFlower)
                    .heading(CommonKt.fastInterpolationDegraded(IntakePoses.westFlower.heading(), Values.gardenIntakeConstantTValue, false));
        }

        @NonNull
        @Override
        public String toString() {
            return "From Shoot Loading Pose to Intake West Flower to Garden";
        }
    }
}
