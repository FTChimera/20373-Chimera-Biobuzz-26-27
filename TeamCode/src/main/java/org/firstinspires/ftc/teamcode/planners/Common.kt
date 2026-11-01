package org.firstinspires.ftc.teamcode.planners

import com.pedropathing.paths.interpolator.Interpolator
import org.firstinspires.ftc.teamcode.util.poses.PoseFactory
import org.firstinspires.ftc.teamcode.util.AllianceColor
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
    @JvmField val gardenArea = alliance(pose(9.0, 25.0, 0.0), pose())
    @JvmField val loadingArea = alliance(pose(60.0, 135.0, -90.0), pose())
    @JvmField val visionAnchorGarden = alliance(pose(), pose())
    @JvmField val visionAnchorLoading = alliance(pose(), pose())

    fun init(color: AllianceColor) {
        gardenArea.setAllianceColor(color)
        loadingArea.setAllianceColor(color)
        visionAnchorLoading.setAllianceColor(color)
        visionAnchorGarden.setAllianceColor(color)
    }

}

object IntakePoses {
    @JvmField val gardenEnd = alliance(pose(10.0, 10.0, -90.0), pose())
    @JvmField val gardenControl1 = alliance(pose(18.0, 25.0), pose())

    fun init(color: AllianceColor) {
        gardenEnd.setAllianceColor(color)
        gardenControl1.setAllianceColor(color)
    }
}

object ShootPoses {
    @JvmField val garden = alliance(pose(60.0, 25.0, 90.0), pose())
    @JvmField val loading = alliance(pose(), pose())

    fun init(color: AllianceColor) {
        garden.setAllianceColor(color)
        loading.setAllianceColor(color)
    }
}

object UtilityPoses {
    @JvmField val cornerLoadingArea = alliance(pose(), pose()) //face driver
    @JvmField val cornerGardenArea = alliance(pose(), pose()) //face driver
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