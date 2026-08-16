package com.rmsftc.library.math

import com.pedropathing.ftc.FTCCoordinates
import com.pedropathing.geometry.CoordinateSystem
import com.pedropathing.geometry.PedroCoordinates
import com.pedropathing.geometry.Pose
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D

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

        val SMOOTH_STEP_FUNCTION = Polynomial(doubleArrayOf(0.0,0.0,3.0,-2.0))
        val SMOOTHER_STEP_FUNCTION = Polynomial(doubleArrayOf(0.0,0.0,0.0,10.0,-15.0,6.0))
        @JvmStatic
        fun distance(x1: Double, y1: Double, x2: Double, y2: Double): Double = Math.hypot(x2 - x1, y2 - y1)

        @JvmStatic
        fun normalizeVector(x: Double, y: Double): Pose {
            val magnitude = Math.hypot(x, y)
            if (magnitude < EPSILON) return Pose(1.0, 0.0)
            return Pose(x / magnitude, y / magnitude)
        }

        @JvmStatic
        fun normalizedDirection(start: Pose, end: Pose): Pose = normalizeVector(end.x - start.x, end.y - start.y)

        @JvmStatic
        fun addScaled(pose: Pose, direction: Pose, scale: Double): Pose {
            return Pose(
                pose.x + direction.x * scale,
                pose.y + direction.y * scale
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
                        normalizeVector(incoming.x + outgoing.x, incoming.y + outgoing.y)
                    }
                }
            }
        }

        @JvmStatic
        fun adaptiveHandleLengths(waypoints: List<Pose>, maxHandleFraction: Double): DoubleArray {
            require(waypoints.size >= 2) { "Need at least two waypoints" }

            return DoubleArray(waypoints.size) { index ->
                when (index) {
                    0 -> waypoints[0].distanceFrom(waypoints[1]) * maxHandleFraction
                    waypoints.lastIndex -> waypoints[waypoints.lastIndex - 1].distanceFrom(waypoints[waypoints.lastIndex]) * maxHandleFraction
                    else -> {
                        val previous = waypoints[index - 1]
                        val current = waypoints[index]
                        val next = waypoints[index + 1]

                        val previousDistance = previous.distanceFrom(current)
                        val nextDistance = current.distanceFrom(next)
                        val previousDirection = normalizedDirection(previous, current)
                        val nextDirection = normalizedDirection(current, next)
                        val dotProduct = clamp(previousDirection.x * nextDirection.x + previousDirection.y * nextDirection.y, -1.0, 1.0)
                        val turnScale = Math.sqrt(Math.max(0.0, (1.0 + dotProduct) / 2.0))

                        Math.min(previousDistance, nextDistance) * maxHandleFraction * turnScale
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
                val dotProduct = clamp(previousDirection.x * nextDirection.x + previousDirection.y * nextDirection.y, -1.0, 1.0)
                maxTurn = Math.max(maxTurn, Math.acos(dotProduct))
            }

            return maxTurn
        }

        @JvmStatic
        fun maxSpacingRatio(waypoints: List<Pose>): Double {
            if (waypoints.size < 3) return 1.0

            var minDistance = Double.POSITIVE_INFINITY
            var maxDistance = 0.0
            for (index in 0 until waypoints.lastIndex) {
                val distance = waypoints[index].distanceFrom(waypoints[index + 1])
                if (distance < EPSILON) continue
                minDistance = Math.min(minDistance, distance)
                maxDistance = Math.max(maxDistance, distance)
            }

            if (!minDistance.isFinite() || minDistance < EPSILON) return Double.POSITIVE_INFINITY
            return maxDistance / minDistance
        }

        @JvmStatic
        fun offsetPoseInRobotFrame(
            robotPose: Pose,
            forwardDistance: Double,
            lateralOffset: Double,
            coordinateSystem: CoordinateSystem
        ): Pose {
            val convertedRobotPose = coordinateSystem.convertFromPedro(robotPose)
            val heading = convertedRobotPose.heading
            val fieldX = convertedRobotPose.x + forwardDistance * Math.cos(heading) - lateralOffset * Math.sin(heading)
            val fieldY = convertedRobotPose.y + forwardDistance * Math.sin(heading) + lateralOffset * Math.cos(heading)
            return coordinateSystem.convertToPedro(Pose(fieldX, fieldY, heading, coordinateSystem))
        }

        @JvmStatic
        fun wrapAngle(angle: Double): Double {
            return units.normalizeHeading(angle, AngleUnit.RADIANS, HeadingNormSettings.CENTERED_AT_ZERO)
        }

        private fun clamp(value: Double, minimum: Double, maximum: Double): Double {
            return Math.max(minimum, Math.min(maximum, value))
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

        fun toPedroPathing(ftcPose: Pose2D): Pose {
            val pedroPose = Pose(
                ftcPose.getX(org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit.INCH) + 72,
                ftcPose.getY(org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit.INCH) + 72,
                ftcPose.getHeading(org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.RADIANS),
                FTCCoordinates.INSTANCE
            ).getAsCoordinateSystem(PedroCoordinates.INSTANCE)
            return pedroPose
        }

        fun toPedroPathing(ftcPose: Pose3D): Pose {
            return toPedroPathing(Pose2D(
                ftcPose.position.unit,
                ftcPose.position.x,
                ftcPose.position.y,
                org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.RADIANS,
                ftcPose.orientation.getYaw(org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.RADIANS)))
        }

        fun toFTCPose2D(pedroPose: Pose): Pose2D {
            val pedropose1 = pedroPose.getAsCoordinateSystem(FTCCoordinates.INSTANCE)
            return Pose2D(
                org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit.INCH,
                pedropose1.x,
                pedropose1.y,
                org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.RADIANS,
                pedropose1.heading,
            )
        }

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
    }

    inner class MathFunc {
        fun smoothFunc(): Polynomial {
            return SMOOTHER_STEP_FUNCTION
        }
        // Now add smooth interp
        fun evalSmoothF(t: Double): Double {
            return smoothFunc().getY(t)
        }

        fun evalSmoothFInterp(t: Double, h0: Double, h1: Double): Double {
            return h0 + (h1-h0)*evalSmoothF(t)
        }

    }
}