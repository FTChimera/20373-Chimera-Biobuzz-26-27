package org.firstinspires.ftc.teamcode.planners

import com.pedropathing.paths.interpolator.Interpolator
import com.pedropathing.paths.curves.Curve
import org.firstinspires.ftc.teamcode.util.poses.PoseFactory
import org.firstinspires.ftc.teamcode.util.AllianceColor
import org.firstinspires.ftc.teamcode.util.math.MathUtils
import org.firstinspires.ftc.teamcode.util.poses.AlliancePose2d
import org.firstinspires.ftc.teamcode.util.poses.Pose2d
import kotlin.math.*

fun init(color:AllianceColor) {
    StartPoses.init(color)
    IntakePoses.init(color)
    ShootPoses.init(color)
}
fun fastInterpolationDegraded(endHeading: Double, change: Double, reverse: Boolean): Interpolator {
    var tangent = Interpolator.tangent
    if (reverse) tangent = tangent.reverse()
    return Interpolator.piecewise()
        .until(change, tangent)
        .until(1.0, Interpolator.constant(endHeading))
}

fun fastInterpolation(pos1: Pose2d, pos2: Pose2d): Interpolator {
    return fastInterpolation(pos1, pos2, Values.fastInterpolationStartTValue, Values.fastInterpolationEndTValue)
}
fun fastInterpolation(pos1: Pose2d, pos2: Pose2d, startTValue: Double, endTValue: Double): Interpolator {
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
// The reversal decision is latched on the first evaluation and must not be
// shared across paths: construct one interpolator per path.
fun reversingTangent(
    startHeading: Double,
    forwardEndHeading: Double,
    startTValue: Double = Values.fastInterpolationStartTValue,
    endTValue: Double = Values.fastInterpolationEndTValue
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
                reversed = abs(startHeading - tangentAt(t)) > Math.PI / 2
            }
            fun facing(heading: Double): Double =
                if (reversed == true) MathUtils.units.normalizeHeading(heading + Math.PI, MathUtils.AngleUnit.RADIANS, MathUtils.HeadingNormSettings.POSITIVE) else MathUtils.units.normalizeHeading(heading, MathUtils.AngleUnit.RADIANS, MathUtils.HeadingNormSettings.POSITIVE)

            val completion = curve.pathCompletion(t)
            if (completion <= startTValue) return facing(tangentAt(t))
            val endTarget = if (reversed == true) facing(tangentAt(1.0))
                else Angle.normalize(forwardEndHeading)
            if (completion >= endTValue) return endTarget
            val boundaryFacing = facing(tangentAt(curve.parameter(startTValue)))
            val local = (completion - startTValue) / (endTValue - startTValue)
            return MathUtils.units.normalizeHeading(boundaryFacing + (boundaryFacing - endTarget) * local, MathUtils.AngleUnit.RADIANS, MathUtils.HeadingNormSettings.POSITIVE)
        }
    }
}

object Timings {
    const val gardenIntakeTimeMs = 300
    const val flowerIntakeTime = 2
}

object Values {
    // tvalues
    const val fastInterpolationStartTValue = 0.7
    const val fastInterpolationEndTValue = 0.9

    const val gardenIntakeConstantTValue = 0.8
    const val gardenIntakeMaxPowerSetChangeTValue = 0.7
    // drive powers
    const val gardenIntakeMaxPowerSetChangeNewPower = 0.5
}

object StartPoses {
    @JvmField val gardenArea = alliance(pose(9.0, 25.0, 0.0), pose(135.0, 119.0, 180.0))
    @JvmField val loadingArea = alliance(pose(60.0, 135.0, -90.0), pose(84.0, 9.0, 90.0))
    @JvmField val visionAnchorGarden = alliance(pose(66.0, 27.0, 190.0), pose(78.0, 117.0, 10.0))
    @JvmField val visionAnchorLoading = alliance(pose(60.0, 105.0, 142.0), pose(84.0, 39.0, 322.0))

    fun init(color: AllianceColor) {
        gardenArea.setAllianceColor(color)
        loadingArea.setAllianceColor(color)
        visionAnchorLoading.setAllianceColor(color)
        visionAnchorGarden.setAllianceColor(color)
    }

}

object IntakePoses {
    @JvmField val gardenEnd = red(pose(7.0, 5.0, -90.0))
    @JvmField val gardenControl1 = alliance(pose(18.0, 25.0), pose(126.0, 119.0, 180.0))

    fun init(color: AllianceColor) {
        gardenEnd.setAllianceColor(color)
        gardenControl1.setAllianceColor(color)
    }
}

object ShootPoses {
    @JvmField val garden = alliance(pose(60.0, 25.0, 90.0), pose(84.0, 119.0, -90.0))
    @JvmField val loading = alliance(pose(60.0, 119.0, 270.0), pose(84.0, 25.0, 90.0))

    fun init(color: AllianceColor) {
        garden.setAllianceColor(color)
        loading.setAllianceColor(color)
    }
}

object UtilityPoses {
    @JvmField val cornerLoadingArea = alliance(pose(9.0, 135.0, 180.0), pose(135.0, 9.0, 0.0)) //face driver
    @JvmField val cornerGardenArea = alliance(pose(9.0, 9.0, 180.0), pose(135.0, 130.0, 0.0)) //face driver

    private const val PLACEHOLDER_EPSILON = 1e-6

    @JvmStatic
    fun isCornerLoadingAreaConfigured(): Boolean = isConfigured(cornerLoadingArea)

    @JvmStatic
    fun isCornerGardenAreaConfigured(): Boolean = isConfigured(cornerGardenArea)

    @JvmStatic
    fun isConfigured(pose: Pose2d): Boolean {
        return !(abs(pose.x()) < PLACEHOLDER_EPSILON
                && abs(pose.y()) < PLACEHOLDER_EPSILON
                && abs(pose.heading()) < PLACEHOLDER_EPSILON)
    }
}

private fun alliance(red: Pose2d, blu: Pose2d): AlliancePose2d {
    return AlliancePose2d(red, blu, AllianceColor.RED)
}

private fun pose(x: Double, y: Double, heading: Double): Pose2d {
    return PoseFactory.deg(x, y, heading)
}
private fun pose(x: Double, y: Double): Pose2d {
    return PoseFactory.of(x, y)
}
private fun pose(): Pose2d{return pose(0.0,0.0,0.0)}


private val mirrorFactory = PoseFactory({pose2d -> PoseFactory.of(144- pose2d.x(), 144-pose2d.y(),
    MathUtils.units.normalizeHeading(pose2d.heading() + PI, MathUtils.AngleUnit.RADIANS, MathUtils.HeadingNormSettings.POSITIVE))})
private fun mirrorPose(red: Pose2d) = mirrorFactory.create(red)


private fun red(red: Pose2d): AlliancePose2d {
    return AlliancePose2d(red, mirrorPose(red), AllianceColor.RED)
}
