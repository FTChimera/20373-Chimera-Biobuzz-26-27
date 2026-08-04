package org.firstinspires.ftc.teamcode.library.math

import com.pedropathing.ftc.FTCCoordinates
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

        private const val MILLIMETERS_PER_INCH = 25.4
        private const val CENTIMETERS_PER_INCH = 2.54
        private const val INCHES_PER_FOOT = 12.0
        private const val INCHES_PER_METER = 39.37007874015748
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
}