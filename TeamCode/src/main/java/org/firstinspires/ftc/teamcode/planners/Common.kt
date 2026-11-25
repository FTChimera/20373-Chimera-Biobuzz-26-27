package org.firstinspires.ftc.teamcode.planners

import com.pedropathing.paths.interpolator.Interpolator
import com.pedropathing.paths.curves.Curve
import org.firstinspires.ftc.teamcode.util.poses.PoseFactory
import com.rmsftc.utils.AllianceColor
import org.firstinspires.ftc.teamcode.util.math.MathUtils
import org.firstinspires.ftc.teamcode.util.poses.AlliancePose
import com.rmsftc.utils.math.Pose
import org.locationtech.jts.geom.Coordinate
import kotlin.math.*

fun init(color:AllianceColor) {
    Constants.StartPoses.init(color)
    Constants.IntakePoses.init(color)
    Constants.ShootPoses.init(color)
    Constants.PathPoses.init(color)
    Constants.UtilityPoses.init(color)
}
fun fastInterpolationDegraded(endHeading: Double, change: Double, reverse: Boolean): Interpolator {
    var tangent = Interpolator.tangent
    if (reverse) tangent = tangent.reverse()
    return Interpolator.piecewise()
        .until(change, tangent)
        .until(1.0, Interpolator.constant(endHeading))
}

fun fastInterpolation(pos1: Pose, pos2: Pose): Interpolator {
    return fastInterpolation(pos1, pos2, Constants.Values.fastInterpolationStartTValue, Constants.Values.fastInterpolationEndTValue)
}
fun fastInterpolation(pos1: Pose, pos2: Pose, startTValue: Double, endTValue: Double): Interpolator {
    var tangentHeading = atan2(pos2.y() - pos1.y(), pos2.x() - pos1.x())
    val startHeading = pos1.heading()
    val endHeading = pos2.heading()
    val headingError = atan2(
        sin(tangentHeading - startHeading),
        cos(tangentHeading - startHeading)
    )

    val reversed = abs(headingError) > Math.PI / 2
    var tangentInterpolator = Interpolator.tangent
    if (reversed) {
        tangentHeading -= Math.PI
        tangentInterpolator = tangentInterpolator.reverse()
    }
    return Interpolator.piecewise()
        .until(startTValue, tangentInterpolator)
        .until(endTValue, Interpolator.linear(tangentHeading, endHeading))
        .until(1.0, Interpolator.constant(endHeading))
}

// FLAGGED CHANGE: heading interpolator that follows the curve tangent,
// compared against the robot's heading at the start of the path: if the
// curve's initial tangent points more than 90 degrees away from
// [startHeading], the tangent segment is reversed so the robot drives
// backward instead of spinning 180 degrees. Completion fractions follow
// the existing Values (tangent until [startTValue], blended to the end
// heading by [endTValue], then constant), measured by arc length via
// Curve.pathCompletion, as in fastInterpolation.
//
// [forwardEndHeading] is the heading to end at when driving forward (the
// cardinal heading set in Field.generatePathToLoadingZoneShortest). When
// the path is reversed, it is ignored: the robot stays reversed and ends
// facing the reversed final tangent, so there is no spin at the end
// either.
//
// The reversal decision is made from the curve's INITIAL tangent
// (t = 0), not from whichever t the interpolator is first evaluated
// at — PathTracker evaluates it at t = 1.0 during construction, which
// used to latch the decision onto the final tangent. It must not be
// shared across paths: construct one interpolator per path.
fun reversingTangent(
    startHeading: Double,
    forwardEndHeading: Double,
    startTValue: Double = Constants.Values.fastInterpolationStartTValue,
    endTValue: Double = Constants.Values.fastInterpolationEndTValue
): Interpolator {
    var reversed: Boolean? = null
    return object : Interpolator {
        override fun isGeometric(): Boolean = true

        override fun interpolate(curve: Curve, t: Double): Double {
            fun tangentAt(tt: Double): Double {
                val d = curve.derivative(tt)
                return if (d.magnitude() < 1e-9) startHeading else d.theta()
            }
            if (reversed == null) {
                // Decide from the INITIAL tangent (t = 0) explicitly. The
                // interpolator is evaluated at t = 1.0 during PathTracker
                // construction, so latching on tangentAt(t) would lock the
                // decision onto the FINAL tangent and flip the reversal
                // from central poses, where the initial and final tangents
                // disagree.
                reversed = abs(MathUtils.units.headingError(startHeading, tangentAt(0.0), MathUtils.AngleUnit.RADIANS, MathUtils.HeadingNormSettings.CENTERED_AT_ZERO)) > Math.PI / 2
            }
            fun facing(heading: Double): Double =
                if (reversed == true) MathUtils.units.normalizeHeading(heading + Math.PI, MathUtils.AngleUnit.RADIANS, MathUtils.HeadingNormSettings.POSITIVE) else MathUtils.units.normalizeHeading(heading, MathUtils.AngleUnit.RADIANS, MathUtils.HeadingNormSettings.POSITIVE)

            val completion = curve.pathCompletion(t)
            if (completion <= startTValue) return facing(tangentAt(t))
            val endTarget = if (reversed == true) facing(tangentAt(1.0))
                else MathUtils.units.normalizeHeading(forwardEndHeading, MathUtils.AngleUnit.RADIANS, MathUtils.HeadingNormSettings.POSITIVE)
            if (completion >= endTValue) return endTarget
            val boundaryFacing = facing(tangentAt(curve.parameter(startTValue)))
            val local = (completion - startTValue) / (endTValue - startTValue)
            return MathUtils.units.normalizeHeading(boundaryFacing + MathUtils.units.headingError(endTarget, boundaryFacing, MathUtils.AngleUnit.RADIANS, MathUtils.HeadingNormSettings.CENTERED_AT_ZERO) * local, MathUtils.AngleUnit.RADIANS, MathUtils.HeadingNormSettings.POSITIVE)
        }
    }
}

/**
 * Single home for TeamCode's numeric constants (draft, Phase 1: definitions
 * only). Every leaf is a `const val` unless flagged: poses stay `@JvmField
 * val`, computed values are plain `val`, shot tables are `val` lists, and
 * AprilTag clusters are `val intArrayOf`. Same numeric value with a different
 * meaning gets a separate leaf — never share one constant across meanings.
 *
 * Java access: `Constants.Intake.Deployer.DEPLOYED_POSITION`.
 *
 * References elsewhere still point at the old names — Phase 2 rewires them.
 * Nothing outside this file was touched by the move.
 */
object Constants {

    object Timings {
        const val gardenIntakeTimeMs = 700
        const val flowerIntakeTime = 2
        // Flower legs get timeouts so a robot wedged at the flower (deployer
        // disabled) cannot hang the whole auto on one follow/turn.
        const val flowerFollowTimeoutMs = 3500
        const val flowerTurnTimeoutMs = 2500
        const val shootingTimeMs = 900
        const val upToSpeedShooterTimeoutTimeMs = 1500
        const val hiveLineupTimeoutTime = 2
        // Cut 10 -> 3 (2026-11-21): if no alliance tag is ever seen (wrong
        // alliance saved, camera pipeline, tag IDs), a 10 s wait burns a third
        // of auto before shooting blind anyway. Tags normally appear in <1 s.
        const val autoAlignTimeoutTime = 2
        const val gardenIntakeTimeoutMs = 5000
        const val hiveLineupExtraTimeSec = 2
    }

    object Values {
        // tvalues
        const val fastInterpolationStartTValue = 0.7
        const val fastInterpolationEndTValue = 0.9

        const val gardenIntakeConstantTValue = 0.8
        const val gardenIntakeMaxPowerSetChangeTValue = 0.7
        // drive powers
        const val gardenIntakeMaxPowerSetChangeNewPower = 0.5
        // flower intake: deploy the ramp midway to the flower, stow shortly
        // after leaving it (path-completion t-values, 0-1 along the leg)
        const val flowerIntakeDeployTValue = 0.1
        const val flowerIntakeStowTValue = 0.5
    }

    object StartPoses {
        @JvmField val gardenArea = red(pose(9.0, 25.0, 0.0))
        @JvmField val loadingArea = red(pose(64.0, 128.0, -90.0))
        @JvmField val visionAnchorGarden = red(pose(66.0, 27.0, 190.0))
        @JvmField val visionAnchorLoading = red(pose(60.0, 105.0, 142.0))

        fun init(color: AllianceColor) {
            gardenArea.setAllianceColor(color)
            loadingArea.setAllianceColor(color)
            visionAnchorLoading.setAllianceColor(color)
            visionAnchorGarden.setAllianceColor(color)
        }

    }

    object IntakePoses {
        @JvmField val gardenEnd = red(pose(6.0, 1.0, -90.0))
        @JvmField val gardenControl1 = red(pose(40.0, 50.0))
        // Flower intake poses: the robot center sits 1 in + half the robot
        // length (17.008 / 2 = 8.504, so 9.504 in) from the flower ring center,
        // facing the flower head-on - the ramp ends up under the pollen. West
        // ring center (2.54, 48), north ring center (48, 141.46). Blue sides
        // are auto-mirrored (east / south flowers).
        @JvmField val westFlower = red(pose(16.044, 40.0, 180.0))
        @JvmField val northFlower = red(pose(44.0, 121.0, 90.0))

        fun init(color: AllianceColor) {
            gardenEnd.setAllianceColor(color)
            gardenControl1.setAllianceColor(color)
            westFlower.setAllianceColor(color)
            northFlower.setAllianceColor(color)
        }
    }

    object ShootPoses {
        @JvmField val garden = red(pose(40.0, 10.0, 65.0))
        @JvmField val loading = red(pose(60.0, 120.0, -90.0))

        fun init(color: AllianceColor) {
            garden.setAllianceColor(color)
            loading.setAllianceColor(color)
        }
    }

    object PathPoses {
        // Path control points and intermediate poses for the planners, all in
        // red coordinates; blue sides are auto-mirrored, like the other pose
        // tables. Control points carry no heading (only x/y steer the curve).
        // Headings here go through pose() (degrees), never raw radians.
        // Flower approach / return controls:
        @JvmField val westFlowerFromGarden = red(pose(36.0, 36.0))
        @JvmField val westFlowerFromLoading = red(pose(22.0, 118.0))
        @JvmField val westFlowerReturn = red(pose(36.0, 30.0))
        @JvmField val northFlowerFromGarden = red(pose(78.0, 72.0))
        @JvmField val northFlowerFromLoading = red(pose(50.0, 100.0))
        // Cross-field return controls (to the shoot poses):
        @JvmField val gardenToLoading = red(pose(35.0, 125.0))
        @JvmField val loadingToGarden = red(pose(50.0, 30.0))
        // Under-hive corridor waypoints for the tree auto's side switch: a
        // straight line between them passes under the west hive base
        // (x 45..51, y 50..94) with ~2 in of body clearance and stays west
        // of the x=72 midpoint. Used with a constant corridor heading
        // (90 deg red / -90 deg blue, derived from the waypoint direction).
        @JvmField val underHiveSouth = red(pose(61.0, 32.0))
        @JvmField val underHiveNorth = red(pose(61.0, 112.0))
        // Loading-zone preload intake (no-show partner): centre 9.5 in from
        // the west wall (1 in + half the robot length), facing the wall.
        @JvmField val loadingIntakeApproach = red(pose(30.0, 115.0))
        @JvmField val loadingIntake = red(pose(14.0, 108.0, 180.0))
        // Partner-present preload sweep: straight north up the loading zone,
        // centre stopping 8 in short of the north tape while the intake
        // front edge reaches just past it.
        @JvmField val partnerApproach = red(pose(28.0, 102.0))
        @JvmField val partnerSweepStart = red(pose(8.0, 96.0, 90.0))
        @JvmField val partnerSweepMid = red(pose(8.0, 104.0))
        @JvmField val partnerSweepEnd = red(pose(8.0, 112.0, 90.0))

        fun init(color: AllianceColor) {
            westFlowerFromGarden.setAllianceColor(color)
            westFlowerFromLoading.setAllianceColor(color)
            westFlowerReturn.setAllianceColor(color)
            northFlowerFromGarden.setAllianceColor(color)
            northFlowerFromLoading.setAllianceColor(color)
            gardenToLoading.setAllianceColor(color)
            loadingToGarden.setAllianceColor(color)
            underHiveSouth.setAllianceColor(color)
            underHiveNorth.setAllianceColor(color)
            loadingIntakeApproach.setAllianceColor(color)
            loadingIntake.setAllianceColor(color)
            partnerApproach.setAllianceColor(color)
            partnerSweepStart.setAllianceColor(color)
            partnerSweepMid.setAllianceColor(color)
            partnerSweepEnd.setAllianceColor(color)
        }
    }

    object UtilityPoses {
        @JvmField val cornerLoadingArea = red(pose(9.0, 130.0, 0.0)) //face away from driver
        @JvmField val cornerGardenArea = red(pose(9.0, 2.0, 0.0)) //face away from driver

        // Kept here (inside Constants.UtilityPoses) rather than at Constants
        // level: it is only used by isConfigured() below.
        private const val PLACEHOLDER_EPSILON = 1e-6

        @JvmStatic
        fun isCornerLoadingAreaConfigured(): Boolean = isConfigured(cornerLoadingArea)

        @JvmStatic
        fun isCornerGardenAreaConfigured(): Boolean = isConfigured(cornerGardenArea)

        @JvmStatic
        fun isConfigured(pose: Pose): Boolean {
            return !(abs(pose.x()) < PLACEHOLDER_EPSILON
                    && abs(pose.y()) < PLACEHOLDER_EPSILON
                    && abs(pose.heading()) < PLACEHOLDER_EPSILON)
        }

        fun init(color: AllianceColor) {
            cornerGardenArea.setAllianceColor(color)
            cornerLoadingArea.setAllianceColor(color)
        }
    }

    object ParkAutoStartPoses {
        private val redUntippedStart = pose(60.0, 10.0, 0.0)
        private val redTippedStart = pose(32.0, 134.0, 0.0)
        private val blueUntippedStart = pose(84.0, 130.0, 180.0)
        private val blueTippedStart = pose(112.0, 10.0, 180.0)
        private val redDestination = pose(9.0, 104.0, 0.0)
        private val blueDestination = pose(135.0, 38.0, 180.0)

        @JvmStatic fun redUntippedStart(): Pose = Pose.fromPedro(redUntippedStart)
        @JvmStatic fun redTippedStart(): Pose = Pose.fromPedro(redTippedStart)
        @JvmStatic fun blueUntippedStart(): Pose = Pose.fromPedro(blueUntippedStart)
        @JvmStatic fun blueTippedStart(): Pose = Pose.fromPedro(blueTippedStart)
        @JvmStatic fun destination(color: AllianceColor): Pose =
            if (color.isRed()) Pose.fromPedro(redDestination) else Pose.fromPedro(blueDestination)
    }

    object FieldPoses {
        private val redTippedHiveGoal = pose(60.0, 84.0)
        private val redUntippedHiveGoal = pose(60.0, 60.0)
        private val blueTippedHiveGoal = pose(84.0, 60.0)
        private val blueUntippedHiveGoal = pose(84.0, 84.0)

        @JvmStatic fun redTippedHiveGoal(): Pose = Pose.fromPedro(redTippedHiveGoal)
        @JvmStatic fun redUntippedHiveGoal(): Pose = Pose.fromPedro(redUntippedHiveGoal)
        @JvmStatic fun blueTippedHiveGoal(): Pose = Pose.fromPedro(blueTippedHiveGoal)
        @JvmStatic fun blueUntippedHiveGoal(): Pose = Pose.fromPedro(blueUntippedHiveGoal)
    }

    object FieldCoordinates {
        @JvmStatic fun redLoadingZone(): Array<Coordinate> = arrayOf(
            Coordinate(0.0, 120.0), Coordinate(12.0, 120.0), Coordinate(12.0, 96.0),
            Coordinate(0.0, 96.0), Coordinate(0.0, 120.0)
        )

        // Point-mirror of red (144-x, 144-y), matching the official field:
        // red is x 0-12, y 96-120 (12x24), so blue is x 132-144, y 24-48.
        @JvmStatic fun blueLoadingZone(): Array<Coordinate> = arrayOf(
            Coordinate(Field.SIZE_IN, 24.0), Coordinate(132.0, 24.0), Coordinate(132.0, 48.0),
            Coordinate(Field.SIZE_IN, 48.0), Coordinate(Field.SIZE_IN, 24.0)
        )

        @JvmStatic fun westHiveBase(): Array<Coordinate> = arrayOf(
            Coordinate(45.0, 50.0), Coordinate(51.0, 50.0), Coordinate(51.0, 94.0),
            Coordinate(45.0, 94.0), Coordinate(45.0, 50.0)
        )

        @JvmStatic fun eastHiveBase(): Array<Coordinate> = arrayOf(
            Coordinate(99.0, 94.0), Coordinate(93.0, 94.0), Coordinate(93.0, 50.0),
            Coordinate(99.0, 50.0), Coordinate(99.0, 94.0)
        )

        @JvmStatic fun westFlower(): Array<Coordinate> = arrayOf(
            Coordinate(0.0, 45.0), Coordinate(4.9, 45.0), Coordinate(4.9, 51.0),
            Coordinate(0.0, 51.0), Coordinate(0.0, 45.0)
        )

        @JvmStatic fun northFlower(): Array<Coordinate> = arrayOf(
            Coordinate(45.0, 139.1), Coordinate(51.0, 139.1), Coordinate(51.0, 144.0),
            Coordinate(45.0, 144.0), Coordinate(45.0, 139.1)
        )

        @JvmStatic fun eastFlower(): Array<Coordinate> = arrayOf(
            Coordinate(139.1, 93.0), Coordinate(144.0, 93.0), Coordinate(144.0, 99.0),
            Coordinate(139.1, 99.0), Coordinate(139.1, 93.0)
        )

        @JvmStatic fun southFlower(): Array<Coordinate> = arrayOf(
            Coordinate(93.0, 0.0), Coordinate(99.0, 0.0), Coordinate(99.0, 4.9),
            Coordinate(93.0, 4.9), Coordinate(93.0, 0.0)
        )
    }

    object Field {
        // Field size, inches. Unifies AStarPathfinder.FIELD_SIZE_IN (144.0)
        // and the bare 144 / 144.0 literals in Field.java.
        const val SIZE_IN = 144.0
        // From Field.TRANSIT_CLEARANCE_INCHES.
        const val TRANSIT_CLEARANCE_INCHES = 1.0

        object AStar {
            // From AStarPathfinder.
            const val CELL_SIZE_IN = 6.0
            const val EPSILON = 1e-9
            // Initial capacity of the open-set PriorityQueue.
            const val OPEN_SET_INITIAL_CAPACITY = 64
            // Computed, so plain val (not const val). @JvmField so Java
            // call sites can use Constants.Field.AStar.CELLS_PER_SIDE etc.
            // directly instead of going through INSTANCE getters.
            @JvmField val CELLS_PER_SIDE = (Field.SIZE_IN / CELL_SIZE_IN).roundToInt()
            @JvmField val DIAGONAL_COST = CELL_SIZE_IN * sqrt(2.0)
            @JvmField val SQRT2_MINUS_1 = sqrt(2.0) - 1.0
        }

        object Balls {
            // From Ball.POLLEN / Ball.NECTAR diameters (getDiameter()).
            const val POLLEN = 2.8
            const val NECTAR = 3.6
        }

        object AprilTags {
            // From BiobuzzAprilTags clusters (Cluster objects there); int
            // arrays here — Phase 2 reconciles the Cluster shape.
            // Official BIOBUZZ red 30-37, blue 38-45.
            // @JvmField so Java call sites can use them directly.
            @JvmField val RED_OPPOSITE = intArrayOf(30, 31, 32, 33)
            @JvmField val RED_AUDIENCE = intArrayOf(34, 35, 36, 37)
            @JvmField val BLUE_AUDIENCE = intArrayOf(38, 39, 40, 41)
            @JvmField val BLUE_OPPOSITE = intArrayOf(42, 43, 44, 45)
        }
    }

    object Intake {
        // From Intake.update(): FULL_POWER 1, LOW_POWER 0.7, REVERSE -0.5.
        // EJECTING is conditional in the source: -0.5 if the previous mode
        // was LOW_POWER, else -0.7. EJECT_POWER is the -0.7 branch.
        const val FULL_POWER = 1.0
        const val LOW_POWER = 0.7
        const val REVERSE_POWER = -0.5
        const val EJECT_POWER = -0.7
        // CachedMotor cache interval in the Intake constructor.
        const val CACHE_INTERVAL_S = 0.1

        object Deployer {
            // From Intake.DEPLOYER_* (public static double there — freezing
            // them here removes that mutability; nothing writes them today).
            const val GROUND_POSITION = 0.20313725590705872
            const val TRANSLATION_FACTOR = -0.043725488781929
            const val DEPLOYED_POSITION = 0.25
            const val STOWED_POSITION = 0.6666666680574418
        }
    }

    object Transfer {
        // From Transfer: POLLEN_KICK_TIME_MS, CachedMotor interval, and the
        // per-mode powers in update().
        const val POLLEN_KICK_TIME_MS = 300.0
        const val CACHE_INTERVAL_S = 0.01
        const val KICK_ONE_POWER = 0.5
        const val SHOOT_ALL_POWER = 0.7
        const val PANIC_SHOOT_POWER = 1.0
        const val SLOW_POWER = 0.5
        const val REVERSE_EJECT_POWER = -0.5
        const val BLOCK_POWER = -0.05
    }

    object Shooter {
        object SmartFlywheel {
            // Single leaf for the 28 ticks/rev used by both motors
            // (Shooter MAIN_/COUNTER_ROLLER_PARAMS = Params(28, 1)) and by
            // ProjectileShotCalc.TICKS_PER_REV — ShotCalc references this
            // leaf, it does not get its own copy.
            const val TICKS_PER_REV = 28

            object Main {
                // From Shooter.mainConstants (DoubleFlywheelFullVelocityConstants).
                const val KP = 0.00545791
                const val KI = 0.00810202
                const val KD = 0.00245114
                const val KS = 0.144501
                const val KV = 0.00512021
                const val KA = 0.00584196
                const val TOLERANCE_TICKS_PER_SEC = 30.0
                const val INTEGRAL_LIMIT_TICKS = 30.8565
                const val NOMINAL_VOLTAGE = 12.0
            }

            object Counter {
                // From Shooter.counterConstants.
                const val KP = 0.00897372
                const val KI = 0.0308260
                const val KD = 0.00174155
                const val KS = 3.15347
                const val KV = 0.00487241
                const val KA = 0.00650123
                const val TOLERANCE_TICKS_PER_SEC = 30.0
                const val INTEGRAL_LIMIT_TICKS = 8.11004
                const val NOMINAL_VOLTAGE = 12.0
            }

            // From Shooter.upToSpeed(): error within 40 ticks/s on both
            // motors. FLAG: this 40 differs from the 30.0 tolerances above
            // (inventory discrepancy #2) — kept as-is, NOT "fixed".
            const val UP_TO_SPEED_TOLERANCE_TICKS_PER_SEC = 40
            // From DoubleFlywheelFullVelocityController's plausibility window
            // [0.75 * nominal, 1.20 * nominal].
            const val MIN_VALID_COMPENSATION_RATIO = 0.75
            const val MAX_VALID_COMPENSATION_RATIO = 1.20
            // CachedMotor cache interval in the SmartFlywheel constructor.
            const val MOTOR_CACHE_INTERVAL_S = 0.005
        }

        object ShotCalc {
            // From ProjectileShotCalc. LAUNCH_ANGLE_DEG / LAUNCH_HEIGHT_IN /
            // WHEEL_RADIUS_IN are MEASURE placeholders (table-fit values).
            // TICKS_PER_REV lives in SmartFlywheel above, not here.
            const val GRAVITY_IN_PER_S2 = 386.09
            const val LAUNCH_ANGLE_DEG = 62.7
            const val LAUNCH_HEIGHT_IN = 19.0
            const val GOAL_HEIGHT_IN = 59.55
            const val WHEEL_RADIUS_IN = 1.1
            const val COUNTER_ROLLER_RATIO = 1.0
        }

        object Table {
            // From TableShotCalc.shooterMap / counterRollerMap, same entries
            // in the same order. Source lists are static mutable ArrayLists;
            // these are read-only listOf — no mutation sites exist outside
            // TableShotCalc (only read by interpolate()), so nothing is lost.
            // Counter table is non-monotonic (940 at 40 in, 880 at 69.5 in)
            // and needs re-measuring — kept as-is.
            // Fully-qualified TableEntry: no new import was added to this file.
            // @JvmField so Java call sites can use them directly.
            @JvmField val shooterMap = listOf(
                org.firstinspires.ftc.teamcode.util.shotcalc.TableShotCalc.TableEntry(36.9, 800.0),
                org.firstinspires.ftc.teamcode.util.shotcalc.TableShotCalc.TableEntry(49.5, 820.0),
                org.firstinspires.ftc.teamcode.util.shotcalc.TableShotCalc.TableEntry(69.5, 880.0)
            )
            @JvmField val counterRollerMap = listOf(
                org.firstinspires.ftc.teamcode.util.shotcalc.TableShotCalc.TableEntry(30.0, 600.0),
                org.firstinspires.ftc.teamcode.util.shotcalc.TableShotCalc.TableEntry(40.0, 940.0),
                org.firstinspires.ftc.teamcode.util.shotcalc.TableShotCalc.TableEntry(69.5, 880.0)
            )
        }

        object Aimer {
            // From AutoAimerCalculator.calculateTurningPower(): gain 0.5 on
            // both branches, clamped to ±1.0.
            const val TURNING_POWER_GAIN = 0.5
            const val TURNING_POWER_MAX = 1.0
        }
    }

    object Drivetrain {
        // From DrivetrainCommands (source names POSITION_TOLERANCE,
        // HEADING_TOLERANCE, LINEAR_VELOCITY_TOLERANCE,
        // ANGULAR_VELOCITY_TOLERANCE — renamed here with units). These are
        // the TeamCode tolerances, NOT the vendored Pedro values, which stay
        // in pedro/Constants.java.
        const val POSITION_TOLERANCE_IN = 0.1
        const val HEADING_TOLERANCE_RAD = 0.007
        const val LINEAR_VELOCITY_TOLERANCE_IN_PER_SEC = 0.1
        const val ANGULAR_VELOCITY_TOLERANCE_RAD_PER_SEC = 0.1
    }

    object Vision {
        // Vision.java's numeric statics (RED/BLUE_PIPELINE_INDEX,
        // MIN_TAGS_FOR_LOCALIZATION, MAX_RESULT_STALENESS_MS,
        // MAX_LOCALIZATION_AMBIGUITY_M, FIELD_X/Y_OFFSET_IN,
        // FIELD_HEADING_OFFSET_DEG, MIN_HIVE_TAG_AREA,
        // HIVE_CLUSTER_AREA_MARGIN) are NOT moved here: Vision.java is
        // @Config, so those are live Dashboard tunables, and freezing them
        // as const val would remove that tunability. They stay in Vision.java.

        object Cam {
            // CamTransform.HypParams values (hivePivotHeightIn 43.95,
            // hiveCellDownAngleDeg 23.0, hiveCellUpAngleDeg 35.0,
            // hivePitchCheckTolDeg 10.0) are likewise NOT moved: HypParams is
            // a @Config holder of mutable tunables. They stay in CamTransform.
            // From the inline `tagCount >= 2` in CamTransform.isReliable().
            const val MIN_TAGS_FOR_RELIABLE = 2
        }
    }

    object Tree {
        object Shoot {
            // From TreeShooterCommands. Heading/angular tolerances are kept
            // as degree primitives (const val); the source converts with
            // Math.toRadians at use — Phase 2 converts at the call site.
            // SHOOT_APPROACH_TIMEOUT_MS is a SEPARATE 5000 ms leaf from
            // Timings.gardenIntakeTimeoutMs (also 5000 ms): different meaning.
            const val SHOOT_SETTLE_MS = 1500.0
            const val SETTLE_DWELL_MS = 150.0
            const val POSITION_TOLERANCE_IN = 2.0
            const val HEADING_TOLERANCE_DEG = 3.0
            const val LINEAR_SPEED_TOLERANCE_IN_PER_SEC = 2.0
            const val ANGULAR_SPEED_TOLERANCE_DEG_PER_SEC = 5.0
            const val SHOOT_APPROACH_TIMEOUT_MS = 5000.0
        }

        object SideSwitch {
            // From TreeShooterCommands.
            const val SWITCH_POSITION_TOLERANCE_IN = 1.0
            const val SWITCH_TRAVERSE_TIMEOUT_MS = 6000.0
        }
    }

    object Lights {
        // 24 = 12 + 6 + 6 (one 12-LED strip, two 6-LED strips).
        const val TOTAL_LEDS = 24
        const val STRIP_12_LEDS = 12
        const val STRIP_6_LEDS = 6

        object Events {
            // From Lights event methods: blink/pulse period, primary-color
            // on-time (blink only), and the playEventAnimation duration.
            // Same numbers in different events are SEPARATE leaves.
            const val REZERO_PERIOD_MS = 300
            const val REZERO_ON_TIME_MS = 150
            const val REZERO_DURATION_MS = 600
            const val INTAKE_ON_PERIOD_MS = 600
            const val INTAKE_ON_DURATION_MS = 600
            const val INTAKE_OFF_PERIOD_MS = 400
            const val INTAKE_OFF_ON_TIME_MS = 200
            const val INTAKE_OFF_DURATION_MS = 400
            const val DEPLOYER_TOGGLE_PERIOD_MS = 300
            const val DEPLOYER_TOGGLE_ON_TIME_MS = 150
            const val DEPLOYER_TOGGLE_DURATION_MS = 600
            const val EJECT_START_PERIOD_MS = 200
            const val EJECT_START_ON_TIME_MS = 100
            const val EJECT_START_DURATION_MS = 1000
            const val SHOOT_START_PERIOD_MS = 100
            const val SHOOT_START_DURATION_MS = 800
            const val REVERSE_OUTTAKE_PERIOD_MS = 200
            const val REVERSE_OUTTAKE_ON_TIME_MS = 100
            const val REVERSE_OUTTAKE_DURATION_MS = 1000
            const val PANIC_START_PERIOD_MS = 400
            const val PANIC_START_DURATION_MS = 1500
            const val PARK_START_DIM_FACTOR = 4.0f
            const val PARK_START_SPEED = 0.4f
            const val PARK_START_PERIOD_MS = 1000
            const val PARK_START_DURATION_MS = 1000
        }

        object TransferDisplay {
            // From Lights.ballColorAnimation / shootReadyAnimation (period
            // 500 each) and the 1.4 dim factor in displayTransferColor().
            const val BALL_COLOR_PERIOD_MS = 500
            const val SHOOT_READY_PERIOD_MS = 500
            const val DIM_FACTOR = 1.4
        }

        object TeamSine {
            // Shared by the team-color / alliance-color / park sine waves
            // (Lights.teamColorAnimation, displayAllianceColor, onParkStart
            // all use dim 4.0, speed 0.4, period 1000).
            const val DIM_FACTOR = 4.0f
            const val SPEED = 0.4f
            const val PERIOD_MS = 1000
        }

        object Blinkin {
            // Shared servo-pulse values from both Blinkin drivers
            // (FtcBlinkinLedDriver SERVO_PULSE_MIN/MAX_US + NORMAL_PULSE_BASE_US,
            // RevBlinkinLedDriver @ServoType bounds + PATTERN_OFFSET 10).
            const val SERVO_PULSE_MIN_US = 500
            const val SERVO_PULSE_MAX_US = 2500
            const val NORMAL_PULSE_BASE_US = 1005
            const val PATTERN_STEP_US = 10
            // RevBlinkinLedDriver.PULSE_WIDTH_INCREMENTOR, as declared
            // (servo-position units; 0.0005 per step). Its computed
            // BASE_SERVO_POSITION (505 * incrementor) stays driver-local.
            const val PULSE_WIDTH_INCREMENTOR = 0.0005
            // FtcBlinkinLedDriver.DEFAULT_FRAME_DURATION_MS (that driver only).
            const val DEFAULT_FRAME_DURATION_MS = 25
        }
    }

    object Robot {
        object Battery {
            // From Robot: low-power threshold/tolerance, and the 300 ms
            // cached-voltage refresh (cur - lastBatteryTime < 300).
            const val LOW_POWER_VOLTAGE_THRESHOLD = 11.5
            const val LOW_POWER_VOLTAGE_TOLERANCE = 0.25
            const val CACHE_MS = 300
        }

        object AutoPark {
            // From Robot.AUTOMATIC_PARK_START_TIME_REMAINING_SEC (renamed).
            const val START_TIME_REMAINING_SEC = 5.0
        }

        object Park {
            // One shared leaf replacing the 9 PARK_RETRY_INTERVAL_SEC
            // declarations (8 autos + MainTeleOp), all 0.5 s.
            const val RETRY_INTERVAL_SEC = 0.5
        }

        object Drive {
            // From Robot.update(): drivePowers.scale(0.7) and the 0.35
            // auto-align dampening. NOTE: the source scales by 0.7 in BOTH
            // branches (!isBatteryLowPower ? 0.7 : 0.7) — the low-power
            // branch does nothing (inventory discrepancy #6); kept as-is.
            // INPUT_DEADBAND is the 0.1 in hasInput(DriveVector, 0.1).
            const val SCALE = 0.7
            const val INPUT_DEADBAND = 0.1
            const val AUTO_ALIGN_DAMPENING = 0.35
        }

        object Lights {
            // From Robot.update(): flash red at <= 7 s remaining; the
            // one-minute window is 58-61 s remaining.
            const val FLASH_RED_SECONDS_REMAINING = 7.0
            const val ONE_MINUTE_WINDOW_START_SEC = 58
            const val ONE_MINUTE_WINDOW_END_SEC = 61
        }
    }

    object TeleOp {
        object Controls {
            // From Controls.kt: right/left trigger >= 0.5 (both sites).
            const val TRIGGER_THRESHOLD = 0.5
        }

        object AutoFire {
            // Default/min/max of MainTeleOp's Dashboard Var<Double>
            // AUTO_FIRE_HEADING_TOLERANCE_DEGREES (doubleVar(..., 3.0, 0.0,
            // 180.0)). The live Var itself stays in MainTeleOp — only its
            // numeric args are recorded here; Phase 2 may wire the Var's
            // constructor to these leaves.
            const val HEADING_TOLERANCE_DEG_DEFAULT = 3.0
            const val MIN = 0.0
            const val MAX = 180.0
        }
    }

    /**
     * Overflow for constants that cannot be easily classified into a
     * subsystem above. Timing-like values go here; other values go in
     * AutoValues. Prefer a subsystem home when one fits — this object is a
     * last resort, not a dumping ground. Currently empty: nothing found
     * during the move needed it.
     */
    object AutoTimings

    /**
     * Overflow for constants that cannot be easily classified into a
     * subsystem above. See AutoTimings. Currently empty: nothing found
     * during the move needed it.
     */
    object AutoValues
}

private fun alliance(red: Pose, blu: Pose): AlliancePose {
    return AlliancePose(red, blu, AllianceColor.RED)
}

private fun pose(x: Double, y: Double, heading: Double): Pose {
    return PoseFactory.deg(x, y, heading)
}
private fun pose(x: Double, y: Double): Pose {
    return PoseFactory.of(x, y)
}
private fun pose(): Pose{return pose(0.0,0.0,0.0)}


private val mirrorFactory = PoseFactory({pose -> PoseFactory.of(Constants.Field.SIZE_IN - pose.x(), Constants.Field.SIZE_IN - pose.y(),
    MathUtils.units.normalizeHeading(pose.heading() + PI, MathUtils.AngleUnit.RADIANS, MathUtils.HeadingNormSettings.POSITIVE))})
private fun mirrorPose(red: Pose) = mirrorFactory.create(red)


private fun red(red: Pose): AlliancePose {
    return AlliancePose(red, mirrorPose(red), AllianceColor.RED)
}
