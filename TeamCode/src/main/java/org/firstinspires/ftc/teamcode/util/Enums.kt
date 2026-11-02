package org.firstinspires.ftc.teamcode.util

enum class AllianceColor {
    RED,
    BLUE,
    UNKNOWN;

    fun isRed(): Boolean {
        return this==RED
    }

    fun isBlue(): Boolean {
        return this==BLUE
    }

    fun isUnknown(): Boolean {
        return this==UNKNOWN
    }

    fun fieldCentricOffset(): Double {
        if (isBlue()) return 0.0;
        return 180.0;
    }

    companion object {
         fun from (isRed: Boolean): AllianceColor {
            if (isRed) {
                return RED
            }
            else {
                return BLUE
            }
        }

    }
}

enum class RobotMode {
    AUTONOMOUS(30),
    TELEOP(120),
    TEST(Integer.MAX_VALUE);

    val timeLimit: Int
    constructor(i: Int) {
        timeLimit = i
    }
}

enum class MatchMode {
    PLAYOFFS, // maximize points
    QUALIFICATIONS // maximize ranking points
}