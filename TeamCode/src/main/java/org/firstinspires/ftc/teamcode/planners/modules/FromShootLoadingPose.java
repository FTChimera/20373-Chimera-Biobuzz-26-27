package org.firstinspires.ftc.teamcode.planners.modules;

import androidx.annotation.NonNull;

import com.pedropathing.api.Paths;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.interpolator.Interpolator;
import com.rmsftc.cmdlib.cmd.Command;
import com.rmsftc.cmdlib.cmd.Commands;
import com.rmsftc.cmdlib.cmd.WaitCommand;


import org.firstinspires.ftc.teamcode.mechanisms.Robot;
import org.firstinspires.ftc.teamcode.planners.CommonKt;
import org.firstinspires.ftc.teamcode.planners.Constants;
import org.firstinspires.ftc.teamcode.planners.PathPlanner;
import org.firstinspires.ftc.teamcode.planners.Position;
import org.firstinspires.ftc.teamcode.planners.commands.DrivetrainCommands;
import org.firstinspires.ftc.teamcode.planners.commands.ShooterCommands;
import org.firstinspires.ftc.teamcode.util.game.Field;
import com.rmsftc.utils.math.Pose;

public class FromShootLoadingPose {
    public static class ToIntakeFromNorthFlower extends PathPlanner {
        Path toFlower;
        Path back;

        public ToIntakeFromNorthFlower(Robot robot) {
            super(robot, robot.follower, robot.shooter, robot.transfer, robot.intake);
        }

        @Override
        public Pose getEndPose() {
            return Constants.ShootPoses.loading;
        }

        @Override
        public Position from() {
            return Position.LOADING_SHOOT;
        }

        @Override
        public Position to() {
            return Position.LOADING_SHOOT;
        }

        @Override
        protected Command buildCommand() {
            return Commands.sequence(
                    Commands.runOnce(robot.intake::setFullPower),
                    // Turn in place from -90 (south) to 90 (north) first,
                    // then drive with a constant north heading -- no
                    // tangent sweep, no heading snap.
                    DrivetrainCommands.turnToHeadingWithTimeout(robot.follower, Constants.IntakePoses.northFlower.heading(), Constants.Timings.flowerTurnTimeoutMs, robot.clock),
                    Commands.parallel(DrivetrainCommands.followForIntake(robot.follower, () -> toFlower, Constants.Timings.flowerFollowTimeoutMs, Constants.Timings.flowerIntakeTime * 1000L, robot.clock), DrivetrainCommands.runAtCompletion(robot, Constants.Values.flowerIntakeDeployTValue, () -> robot.intake.setDeployed(true))),
                    // Drive back still facing north, then turn in place
                    // to -90 (south) for the shoot.
                    Commands.parallel(DrivetrainCommands.followWithTimeout(robot.follower, () -> back, Constants.Timings.flowerFollowTimeoutMs, robot.clock), DrivetrainCommands.runAtCompletion(robot, Constants.Values.flowerIntakeStowTValue, () -> robot.intake.setDeployed(false))),
                    DrivetrainCommands.turnToHeadingWithTimeout(robot.follower, Constants.ShootPoses.loading.heading(), Constants.Timings.flowerTurnTimeoutMs, robot.clock),
                    ShooterCommands.waitForHiveLineup(robot, Field.HiveState.TIPPED),
                    ShooterCommands.shootAllPollen(robot)
            ).finallyDo(() -> robot.intake.setDeployed(false));
        }

        @Override
        protected void buildPaths() {
            // Short hop north; rect clearance to the west hive base: 15.3 in.
            // Both legs hold a CONSTANT north heading (90 deg): the turns
            // to 90 and back to -90 are separate turn-in-place steps in
            // run(), so the heading never sweeps with the path tangent and
            // never snaps at t=0.8 the way fastInterpolationDegraded did.
            toFlower = Paths
                    .curve(startPose, Constants.PathPoses.northFlowerFromLoading, Constants.IntakePoses.northFlower)
                    .heading(Interpolator.constant(Constants.IntakePoses.northFlower.heading()));
            back = Paths
                    .curve(Constants.IntakePoses.northFlower, Constants.PathPoses.northFlowerFromLoading, Constants.ShootPoses.loading)
                    .heading(Interpolator.constant(Constants.IntakePoses.northFlower.heading()));
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
     * in the loading zone against the perimeter wall, around the centre
     * of the zone, and the robot may collect them (no rule against
     * picking up a partner's POLLEN; G407 caps possession at 4, so the
     * robot must already have shot its own preloads). One branch handles
     * both cases: drive to the wall, intake, then garden shoot. If the
     * partner was present the intake runs empty and the garden shoot
     * finds nothing to shoot.
     *
     * This is NOT a flower intake: the pollen are just laid out on the
     * floor, so there is no deployer deploy, no 2 s dwell and no stow --
     * the garden pattern instead (intake at full power, drive in, a
     * 300 ms settle, drive back). It keeps its own run() for that
     * reason and does not extend the flower planners.
     */
    public static class ToIntakeLoadingToGarden extends PathPlanner {
        Path toWall;
        Path toGarden;

        public ToIntakeLoadingToGarden(Robot robot) {
            super(robot, robot.follower, robot.shooter, robot.transfer, robot.intake);
        }

        @Override
        public Pose getEndPose() {
            return Constants.ShootPoses.garden;
        }

        @Override
        public Position from() {
            return Position.LOADING_SHOOT;
        }

        @Override
        public Position to() {
            return Position.GARDEN_SHOOT;
        }

        @Override
        protected Command buildCommand() {
            // The follow legs are bounded at 5 s (Constants.Timings.gardenIntakeTimeoutMs,
            // same as the tree versions): an unbounded follow() would hang the
            // whole auto on a wedged robot until the 5 s park window.
            return Commands.sequence(
                    Commands.runOnce(robot.intake::setFullPower),
                    DrivetrainCommands.followWithTimeout(robot.follower, () -> toWall, Constants.Timings.gardenIntakeTimeoutMs, robot.clock),
                    new WaitCommand(Constants.Timings.gardenIntakeTimeMs, robot.clock),
                    DrivetrainCommands.followWithTimeout(robot.follower, () -> toGarden, Constants.Timings.gardenIntakeTimeoutMs, robot.clock),
                    ShooterCommands.waitForHiveLineup(robot, Field.HiveState.UNTIPPED),
                    ShooterCommands.shootAllPollen(robot)
            );
        }

        @Override
        protected void buildPaths() {
            // Loading intake pose: robot center 9.5 in from the west wall
            // (1 in + half the robot length), centered in the loading
            // zone (y 96-120), facing the wall -- the staged pollen sit
            // around the zone centre against the wall.
            toWall = Paths
                    .curve(startPose, Constants.PathPoses.loadingIntakeApproach, Constants.PathPoses.loadingIntake)
                    .heading(CommonKt.fastInterpolationDegraded(Constants.PathPoses.loadingIntake.heading(), Constants.Values.gardenIntakeConstantTValue, false));
            // Back to the garden shoot pose: down the west side past the
            // hive, then east along the bottom. Rect clearance to the
            // west hive base stays west of x=45 until below y=50
            // (curve passes (26.4, 48.3) at t=0.5, (41.1, 32.1) at
            // t=0.75). Reverse tangential interpolation, as on the
            // other return legs.
            toGarden = Paths
                    .curve(Constants.PathPoses.loadingIntake, Constants.PathPoses.loadingToGarden, Constants.ShootPoses.garden)
                    .heading(CommonKt.fastInterpolationDegraded(Constants.ShootPoses.garden.heading(), Constants.Values.gardenIntakeConstantTValue, true));
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
            super(robot, robot.follower, robot.shooter, robot.transfer, robot.intake);
        }

        @Override
        public Pose getEndPose() {
            return Constants.ShootPoses.garden;
        }

        @Override
        public Position from() {
            return Position.LOADING_SHOOT;
        }

        @Override
        public Position to() {
            return Position.GARDEN_SHOOT;
        }

        @Override
        protected Command buildCommand() {
            // The follow legs are bounded at 5 s (Constants.Timings.gardenIntakeTimeoutMs):
            // an unbounded follow() would hang the whole auto on a wedged
            // robot until the 5 s park window.
            return Commands.sequence(
                    Commands.runOnce(robot.intake::setFullPower),
                    Commands.parallel(DrivetrainCommands.followWithTimeout(robot.follower, () -> toSweepStart, Constants.Timings.gardenIntakeTimeoutMs, robot.clock), DrivetrainCommands.runAtCompletion(robot, Constants.Values.flowerIntakeDeployTValue, () -> robot.intake.setDeployed(true))),
                    DrivetrainCommands.followWithTimeout(robot.follower, () -> sweep, Constants.Timings.gardenIntakeTimeoutMs, robot.clock),
                    new WaitCommand(Constants.Timings.gardenIntakeTimeMs, robot.clock),
                    Commands.parallel(DrivetrainCommands.followWithTimeout(robot.follower, () -> toGarden, Constants.Timings.gardenIntakeTimeoutMs, robot.clock), DrivetrainCommands.runAtCompletion(robot, Constants.Values.flowerIntakeStowTValue, () -> robot.intake.setDeployed(false))),
                    ShooterCommands.waitForHiveLineup(robot, Field.HiveState.UNTIPPED),
                    ShooterCommands.shootAllPollen(robot)
            );
        }

        @Override
        protected void buildPaths() {
            // Down to the south end of the loading zone, ending facing
            // north for the sweep. Disc clearance to the west hive base:
            // 7.1 in; to the partner: 13.6 in.
            toSweepStart = Paths
                    .curve(startPose, Constants.PathPoses.partnerApproach, Constants.PathPoses.partnerSweepStart)
                    .heading(CommonKt.fastInterpolationDegraded(Constants.PathPoses.partnerSweepStart.heading(), Constants.Values.gardenIntakeConstantTValue, false));
            // Straight north up the zone, intake deployed. Centre stops at
            // y = 112; front edge reaches y = 120.5 (past the north tape,
            // 1 in short of the partner's assumed south face). Disc
            // clearance to the partner at the end of the sweep: 1.0 in.
            sweep = Paths
                    .curve(Constants.PathPoses.partnerSweepStart, Constants.PathPoses.partnerSweepMid, Constants.PathPoses.partnerSweepEnd)
                    .heading(CommonKt.fastInterpolationDegraded(Constants.PathPoses.partnerSweepEnd.heading(), Constants.Values.gardenIntakeConstantTValue, false));
            // Back to the garden shoot pose: down the west side past the
            // hive, then east along the bottom. Disc clearance to the
            // west hive base: 7.4 in. Reverse tangential interpolation, as on
            // the other return legs.
            toGarden = Paths
                    .curve(Constants.PathPoses.partnerSweepEnd, Constants.PathPoses.loadingToGarden, Constants.ShootPoses.garden)
                    .heading(CommonKt.fastInterpolationDegraded(Constants.ShootPoses.garden.heading(), Constants.Values.gardenIntakeConstantTValue, true));
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
        public Position from() {
            return Position.LOADING_SHOOT;
        }

        @Override
        protected void buildPaths() {
            super.buildPaths();
            // Long diagonal past the north-east corner of the west hive base;
            // rect clearance: 3.6 in out (verified in sim with the physical
            // 17.008 in robot; the old (30, 108) control point overlapped
            // the hive by ~1.8 in). The back leg is inherited and uses
            // westFlowerReturn, not this control point.
            // Constant west heading -- the inherited run() already turns to
            // 180 (west) before this leg, so no tangent sweep here either.
            toFlower = Paths
                    .curve(startPose, Constants.PathPoses.westFlowerFromLoading, Constants.IntakePoses.westFlower)
                    .heading(Interpolator.constant(Constants.IntakePoses.westFlower.heading()));
        }

        @NonNull
        @Override
        public String toString() {
            return "From Shoot Loading Pose to Intake West Flower to Garden";
        }
    }
}
