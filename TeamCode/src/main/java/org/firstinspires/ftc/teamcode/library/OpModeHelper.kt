package org.firstinspires.ftc.teamcode.library

import com.qualcomm.hardware.lynx.LynxModule
import com.qualcomm.robotcore.hardware.HardwareMap
import java.util.concurrent.TimeUnit

/**
 * Util Class for usage in an opmode.
 * This class is meant to be initialized in init, start, and updated every loop
 * to provide fresh data.
 * <br>
 * This class provides: Dt (as a long in millis), Bulk reads to optimize loop time
 */
class OpModeHelper {
    enum class State {
        UN_INITIALIZED,
        INITIALIZED,
        STARTED,
        LOOP,
        STOP,
        STOPPED;

        fun next(): State {
            return when (this)
            {
                UN_INITIALIZED -> INITIALIZED
                INITIALIZED -> STARTED
                STARTED -> LOOP
                LOOP -> STOP // Will only be called on transition
                STOP -> STOPPED
                STOPPED -> STOPPED
            }
        }
    }

    constructor(hardwareMap: HardwareMap) {
        state = State.UN_INITIALIZED
        init(hardwareMap)
    }
    var state: State

    lateinit var hardwareMap: HardwareMap
    lateinit var dtTimer: Timer
    var dt: Long = 0L
    lateinit var allHubs: List<LynxModule>

    fun init(hardwareMap: HardwareMap) {
        if (state != State.UN_INITIALIZED) {
            throw IllegalStateException("OpModeHelper is already initialized")
        }
        this.hardwareMap = hardwareMap
        dtTimer = Timer(TimeUnit.MILLISECONDS)
        allHubs = hardwareMap.getAll(LynxModule::class.java)
        for (hub in allHubs) {
            hub.bulkCachingMode = LynxModule.BulkCachingMode.MANUAL

        }
        state = state.next()
    }
    fun start() {
        if (state != State.INITIALIZED) {
            throw IllegalStateException("OpModeHelper is not initialized or is already started")
        }
        dtTimer.reset()
        state = state.next()
    }

    fun update() {
        if (state != State.STARTED && state != State.LOOP) {
            throw IllegalStateException("OpModeHelper is not started or already closed")
        }
        if (state == State.STARTED) {
            state = state.next()
        }
        for (hub in allHubs) {
            hub.clearBulkCache()
        }
        dt = dtTimer.get()
        dtTimer.reset()
    }
    fun close() {
        if (state != State.LOOP) {
            throw IllegalStateException("OpModeHelper is not in loop state, or is already closed")
        }
        allHubs = emptyList()
        dtTimer.reset()
        state = state.next()
    }
}