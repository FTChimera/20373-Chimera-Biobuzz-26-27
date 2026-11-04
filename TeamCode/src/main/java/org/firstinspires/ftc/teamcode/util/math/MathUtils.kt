package org.firstinspires.ftc.teamcode.util.math

import com.pedropathing.math.Pose
import com.pedropathing.utils.Angle
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D
import org.firstinspires.ftc.teamcode.util.interpolation.Interpolate
import kotlin.math.acos
import kotlin.math.atan2
import kotlin.math.cos
import kotlin.math.hypot
import kotlin.math.sin
import kotlin.math.sqrt

class MathUtils private constructor() {

    enum class HeadingNormSettings {
        CENTERED_AT_ZERO, //
        POSITIVE,
        NEGATIVE
    }

    enum class DistanceUnit {
        MILLIMETERS,
        CENTIMETERS,
        METERS,
        INCHES,
        FEET
    }

    enum class AngleUnit {
        DEGREES,
        RADIANS
    }

    companion object {
        private val instance = MathUtils()
        @JvmField
        val units = instance.MathUnits()
        @JvmField
        val func = instance.MathFunc()
        // Constants
        const val PI = Math.PI
        const val TAU = 2 * PI
        const val E = Math.E
        const val DEG_TO_RAD = PI / 180.0
        const val EPSILON = 1e-6

        private const val MILLIMETERS_PER_INCH = 25.4
        private const val CENTIMETERS_PER_INCH = 2.54
        private const val INCHES_PER_FOOT = 12.0
        private const val INCHES_PER_METER = 39.37007874015748

        val SMOOTH_STEP_FUNCTION = Polynomial(
            doubleArrayOf(
                -2.0,
                3.0,
                0.0,
                0.0
            )
        )
        val SMOOTHER_STEP_FUNCTION = Polynomial(
            doubleArrayOf(
                6.0,
                -15.0,
                10.0,
                0.0,
                0.0,
                0.0
            )
        )

        val IDENTITY_LINEAR_FUNCTION = Equation.IDENTITY



        @JvmStatic
        fun distance(x1: Double, y1: Double, x2: Double, y2: Double): Double =
            hypot(x2 - x1, y2 - y1)

        @JvmStatic
        fun normalizeVector(x: Double, y: Double): Pose {
            val magnitude = hypot(x, y)
            if (magnitude < EPSILON) return Pose(1.0, 0.0)
            return Pose(x / magnitude, y / magnitude)
        }

        @JvmStatic
        fun normalizedDirection(start: Pose, end: Pose): Pose = normalizeVector(end.x() - start.x(), end.y() - start.y())

        @JvmStatic
        fun addScaled(pose: Pose, direction: Pose, scale: Double): Pose {
            return Pose(
                pose.x() + direction.x() * scale,
                pose.y() + direction.y() * scale
            )
        }

        @JvmStatic
        fun smoothTangents(waypoints: List<Pose>): Array<Pose> {
            require(waypoints.size >= 2) { "Need at least two waypoints" }

            return Array(waypoints.size) { index ->
                when (index) {
                    0 -> normalizedDirection(waypoints[0], waypoints[1])
                    waypoints.lastIndex -> normalizedDirection(waypoints[waypoints.lastIndex - 1], waypoints[waypoints.lastIndex])
                    else -> {
                        val incoming = normalizedDirection(waypoints[index - 1], waypoints[index])
                        val outgoing = normalizedDirection(waypoints[index], waypoints[index + 1])
                        normalizeVector(incoming.x() + outgoing.x(), incoming.y() + outgoing.y())
                    }
                }
            }
        }

        @JvmStatic
        fun adaptiveHandleLengths(waypoints: List<Pose>, maxHandleFraction: Double): DoubleArray {
            require(waypoints.size >= 2) { "Need at least two waypoints" }

            return DoubleArray(waypoints.size) { index ->
                when (index) {
                    0 -> waypoints[0].distance(waypoints[1]) * maxHandleFraction
                    waypoints.lastIndex -> waypoints[waypoints.lastIndex - 1].distance(waypoints[waypoints.lastIndex]) * maxHandleFraction
                    else -> {
                        val previous = waypoints[index - 1]
                        val current = waypoints[index]
                        val next = waypoints[index + 1]

                        val previousDistance = previous.distance(current)
                        val nextDistance = current.distance(next)
                        val previousDirection = normalizedDirection(previous, current)
                        val nextDirection = normalizedDirection(current, next)
                        val dotProduct = clamp(previousDirection.x() * nextDirection.x() + previousDirection.y() * nextDirection.y(), -1.0, 1.0)
                        val turnScale = sqrt(0.0.coerceAtLeast((1.0 + dotProduct) / 2.0))

                        previousDistance.coerceAtMost(nextDistance) * maxHandleFraction * turnScale
                    }
                }
            }
        }

        @JvmStatic
        fun maxTurnAngle(waypoints: List<Pose>): Double {
            if (waypoints.size < 3) return 0.0

            var maxTurn = 0.0
            for (index in 1 until waypoints.lastIndex) {
                val previousDirection = normalizedDirection(waypoints[index - 1], waypoints[index])
                val nextDirection = normalizedDirection(waypoints[index], waypoints[index + 1])
                val dotProduct = clamp(previousDirection.x() * nextDirection.x() + previousDirection.y() * nextDirection.y(), -1.0, 1.0)
                maxTurn = maxTurn.coerceAtLeast(acos(dotProduct))
            }

            return maxTurn
        }

        @JvmStatic
        fun maxSpacingRatio(waypoints: List<Pose>): Double {
            if (waypoints.size < 3) return 1.0

            var minDistance = Double.POSITIVE_INFINITY
            var maxDistance = 0.0
            for (index in 0 until waypoints.lastIndex) {
                val distance = waypoints[index].distance(waypoints[index + 1])
                if (distance < EPSILON) continue
                minDistance = minDistance.coerceAtMost(distance)
                maxDistance = maxDistance.coerceAtLeast(distance)
            }

            if (!minDistance.isFinite() || minDistance < EPSILON) return Double.POSITIVE_INFINITY
            return maxDistance / minDistance
        }

        @JvmStatic
        fun offsetPoseInRobotFrame(
            robotPose: Pose,
            forwardDistance: Double,
            lateralOffset: Double,
        ): Pose {
            val heading = robotPose.heading()
            val fieldX = robotPose.x() + forwardDistance * cos(heading) - lateralOffset * sin(heading)
            val fieldY = robotPose.y() + forwardDistance * sin(heading) + lateralOffset * cos(heading)
            return Pose(fieldX, fieldY, heading)
        }

        @JvmStatic
        fun wrapAngle(angle: Double): Double {
            return units.normalizeHeading(angle, AngleUnit.RADIANS, HeadingNormSettings.CENTERED_AT_ZERO)
        }

        @JvmStatic
        fun clamp(value: Double, minimum: Double, maximum: Double): Double {
            return minimum.coerceAtLeast(maximum.coerceAtMost(value))
        }

        @JvmStatic
        fun headingDelta(toHeading: Double, fromHeading: Double): Double {
            return wrapAngle(toHeading - fromHeading)
        }

        @JvmStatic
        fun headingBetween(x1: Double, y1: Double, x2: Double, y2: Double): Double =
            atan2(y2 - y1, x2 - x1)

        @JvmStatic
        fun maximum(vararg value: Double): Double {
            return value.maxOrNull() ?: Double.NEGATIVE_INFINITY
        }

        @JvmStatic
        fun minimum(vararg value: Double): Double {
            return value.minOrNull() ?: Double.POSITIVE_INFINITY
        }
    }

    inner class MathUnits {
        fun toMillimeters(num: Double, unit: DistanceUnit): Double = when (unit) {
            DistanceUnit.MILLIMETERS -> num
            DistanceUnit.CENTIMETERS -> num * 10.0
            DistanceUnit.METERS -> num * 1000.0
            DistanceUnit.INCHES -> num * MILLIMETERS_PER_INCH
            DistanceUnit.FEET -> num * INCHES_PER_FOOT * MILLIMETERS_PER_INCH
        }

        fun toCentimeters(num: Double, unit: DistanceUnit): Double = toMillimeters(num, unit) / 10.0

        fun toMeters(num: Double, unit: DistanceUnit): Double = toMillimeters(num, unit) / 1000.0

        fun toInches(num: Double, unit: DistanceUnit): Double = when (unit) {
            DistanceUnit.MILLIMETERS -> num / MILLIMETERS_PER_INCH
            DistanceUnit.CENTIMETERS -> num / CENTIMETERS_PER_INCH
            DistanceUnit.METERS -> num * INCHES_PER_METER
            DistanceUnit.INCHES -> num
            DistanceUnit.FEET -> num * INCHES_PER_FOOT
        }

        fun toFeet(num: Double, unit: DistanceUnit): Double = toInches(num, unit) / INCHES_PER_FOOT

        fun toDegrees(num: Double, unit: AngleUnit): Double = when (unit) {
            AngleUnit.DEGREES -> num
            AngleUnit.RADIANS -> num / DEG_TO_RAD
        }

        fun toRadians(num: Double, unit: AngleUnit): Double = when (unit) {
            AngleUnit.DEGREES -> num * DEG_TO_RAD
            AngleUnit.RADIANS -> num
        }

        fun toDegrees(num: Double): Double {
            return toDegrees(num, AngleUnit.RADIANS)
        }

        fun toRadians(num: Double): Double {
            return toRadians(num, AngleUnit.DEGREES)
        }

//        fun toPedroPathing(ftcPose: Pose2D): Pose {
//        }

        //TODO :IMPLEMENT THIS
//        fun toPedroPathing(ftcPose: Pose3D): Pose {
//            return toPedroPathing(Pose2D(
//                ftcPose.position.unit,
//                ftcPose.position.x,
//                ftcPose.position.y,
//                org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.RADIANS,
//                ftcPose.orientation.getYaw(org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.RADIANS)))
//        }

//        fun toFTCPose2D(pedroPose: Pose): Pose2D {
//            val pedropose1 = pedroPose.getAsCoordinateSystem(FTCCoordinates.INSTANCE)
//            return Pose2D(
//                org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit.INCH,
//                pedropose1.x,
//                pedropose1.y,
//                org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.RADIANS,
//                pedropose1.heading,
//            )
//        }

        fun normalizeHeading(heading: Double, unit: AngleUnit, settings:HeadingNormSettings): Double {
            val fullTurn = when (unit) {
                AngleUnit.DEGREES -> 360.0
                AngleUnit.RADIANS -> TAU
            }
            val halfTurn = fullTurn / 2.0
            val positiveWrapped = ((heading % fullTurn) + fullTurn) % fullTurn

            return when (settings) {
                HeadingNormSettings.CENTERED_AT_ZERO ->
                    if (positiveWrapped > halfTurn) positiveWrapped - fullTurn else positiveWrapped
                HeadingNormSettings.POSITIVE -> positiveWrapped
                HeadingNormSettings.NEGATIVE ->
                    if (positiveWrapped == 0.0) 0.0 else positiveWrapped - fullTurn
            }
        }

        fun headingError(headingA: Double, headingB: Double, unit: AngleUnit, settings: HeadingNormSettings): Double  {
            val normalizedA = normalizeHeading(headingA, unit, settings)
            val normalizedB = normalizeHeading(headingB, unit, settings)
            return normalizeHeading(normalizedA - normalizedB, unit, settings)
        }
    }

    inner class MathFunc {

        fun smoothFunc(): Polynomial {
            return SMOOTH_STEP_FUNCTION
        }
        // Now add smooth interp
        fun evalSmoothF(t: Double): Double {
            return smoothFunc().getY(t)
        }

        fun evalSmoothFInterp(t: Double, h0: Double, h1: Double): Double {
            return Interpolate.func(t, h0, h1, smoothFunc())
        }

    }
}