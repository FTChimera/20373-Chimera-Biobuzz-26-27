package org.firstinspires.ftc.teamcode.planners;

/**
 * A named field position a planner leg starts or ends at. {@link Plan}
 * validates that consecutive legs chain (one leg's {@code to()} is the
 * next leg's {@code from()}), the same safety the old PlanBuilder state
 * machine gave, without the fluent API.
 *
 * <p>{@link #ANY} marks a position-neutral leg (waits, custom paths): it
 * neither requires nor changes the current position, and {@link Plan}
 * skips it during validation.
 */
public enum Position {
    GARDEN_START,
    LOADING_START,
    GARDEN_SHOOT,
    LOADING_SHOOT,
    GARDEN_VISION_SEARCH,
    LOADING_VISION_SEARCH,
    ANY
}
