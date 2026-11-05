package org.firstinspires.ftc.teamcode.util.hardware.gamepad

import com.pedropathing.drivetrain.DrivePowers
import com.sun.tools.javac.api.Formattable
import org.firstinspires.ftc.teamcode.util.AllianceColor
import org.firstinspires.ftc.teamcode.util.Timer
import org.firstinspires.ftc.teamcode.util.math.MathUtils.*
import org.firstinspires.ftc.teamcode.util.math.MathUtils.Companion.DEG_TO_RAD
import org.firstinspires.ftc.teamcode.util.math.MathUtils.Companion.units
import java.util.Locale
import java.util.concurrent.TimeUnit
import java.util.function.BooleanSupplier
import java.util.function.Consumer
import java.util.function.DoubleUnaryOperator
import kotlin.math.cos
import kotlin.math.sin


open class FancyButton constructor(typePressType: PressType) {
    enum class PressType {
        Toggle,
        RisingEdge,
        LongPress,
        FallingEdge,
        Normal
    }
    val pressType: PressType = typePressType

    var timer: Timer = Timer(TimeUnit.SECONDS)

    var action: Consumer<FancyButton> = Consumer {false}
    private var startPress : Boolean = false
    private var endPress : Boolean = false
    private var lastPress: Boolean = false
    // Previous cycle's OUTPUT (pressed), distinct from lastPress (raw input).
    private var lastPressed: Boolean = false

    private var pressed: Boolean = false

    private var longPressAlreadyHappened = false

    inline fun declareActionOnPress(newAction: Consumer<FancyButton>) {
        action = newAction
    }

    fun update(isPressed: Boolean) {
        if (isPressed != lastPress) {timer.reset(); longPressAlreadyHappened = false}
        startPress = isPressed && !lastPress
        endPress = !isPressed && lastPress
        when (pressType) {
            PressType.LongPress -> {pressed = timer.get()>=0.8 && isPressed && !longPressAlreadyHappened; longPressAlreadyHappened = longPressAlreadyHappened||pressed} // use rising edge TODO HERE IS THE LONGPRESS TIMER
            PressType.Toggle -> {if (startPress) {pressed = !pressed}} // also use rising edge
            PressType.RisingEdge -> {pressed = startPress}
            PressType.FallingEdge -> {pressed = endPress}
            PressType.Normal -> {pressed = isPressed}
        }
        action.accept(this)
        lastPress = isPressed
        lastPressed = pressed
    }

    fun setPressed(pressed: Boolean) {
        this.pressed = pressed
        // Establishing a baseline, not a transition: do not report an edge for it.
        this.lastPressed = pressed
    }

    fun getTimeSec(): Double {
        return timer.get()
    }

    fun isPressed(): Boolean {
        return pressed
    }

    fun startPress(): Boolean {
        return startPress
    }

    fun endPress(): Boolean {
        return endPress
    }

    /**
     * Edges on the OUTPUT (pressed) rather than the raw input: risingEdge()
     * is true for the single cycle in which pressed turns true, fallingEdge()
     * for the single cycle in which it turns false. For Normal buttons these
     * match startPress()/endPress(); for Toggle buttons they fire when the
     * toggle flips on/off; for LongPress, risingEdge() fires when the hold
     * completes (0.8 s), not on the initial press.
     */
    fun risingEdge(): Boolean {
        return pressed && !lastPressed
    }

    fun fallingEdge(): Boolean {
        return !pressed && lastPressed
    }
}

open class GpFancyButton(typePressType: PressType, gamepadButtonSupplier: BooleanSupplier) : FancyButton(typePressType) {
    val buttonSupplier = gamepadButtonSupplier
    fun update() {
        update(buttonSupplier.asBoolean)
    }
}

class NormalButton(gamepadButtonSupplier: BooleanSupplier) : GpFancyButton(PressType.Normal, gamepadButtonSupplier)


class DriveVector(var forward: Double, var strafe:Double, var turn:Double) : DrivePowers(forward, strafe, turn) {
    companion object {
        @JvmField
        val ZERO = DriveVector()
    }

    constructor(drivePowers: DrivePowers) : this(
        drivePowers.forward(),
        drivePowers.strafe(),
        drivePowers.turn()
    )

    constructor() : this(0.0, 0.0, 0.0)
    constructor(
        forward: Double,
        strafe: Double,
        turn: Double,
        headingRadians: Double
    ) : this(
        forward * cos(headingRadians) + strafe * sin(headingRadians),
        -forward * sin(headingRadians) + strafe * cos(headingRadians),
        turn
    )

    constructor(turn: Double) : this(0.0, 0.0, turn)

    override fun forward(): Double {
        return forward
    }

    override fun strafe(): Double {
        return strafe
    }

    override fun turn(): Double {
        return turn
    }

    fun forward(double: Double) {
        forward = double
    }

    fun strafe(double: Double) {
        strafe = double
    }

    fun turn(double: Double) {
        turn = double
    }

    override fun toString(): String {
        return String.format(Locale.US, "(%.2f, %.2f, %.2f)", forward, strafe, turn)
    }

    inline fun doubleUnaryOperator(op: (Double) -> Double): DriveVector {
        forward = op(forward)
        strafe = op(strafe)
        turn = op(turn)
        return this
    }

    fun fieldCentric(headingRadians: Double): DriveVector {
        val cosA = cos(headingRadians)
        val sinA = sin(headingRadians)
        val newForward = forward * cosA + strafe * sinA
        val newStrafe = -forward * sinA + strafe * cosA
        forward = newForward
        strafe = newStrafe
        return this
    }

    fun fieldCentric(headingRadians: Double, color: AllianceColor): DriveVector {
        return fieldCentric(units.normalizeHeading(
            headingRadians - (color.fieldCentricOffset() * DEG_TO_RAD),
            unit = AngleUnit.RADIANS,
            settings = HeadingNormSettings.POSITIVE
        ))
    }

    fun copy(): DriveVector {
        return DriveVector(forward, strafe, turn)
    }

    fun scale(v: Double) {
        forward *= v
        strafe *= v
        turn *= v
    }
}

