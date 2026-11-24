package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.teamcode.mechanisms.RobotParkingTest;
import org.firstinspires.ftc.teamcode.mechanisms.SimFakesTest;
import org.firstinspires.ftc.teamcode.mechanisms.VisionHiveInferenceTest;
import org.firstinspires.ftc.teamcode.pedro.procedures.EKFPinpointTuningMathTest;
import org.firstinspires.ftc.teamcode.planners.UtilityPosesTest;
import org.firstinspires.ftc.teamcode.planners.modules.NullPlannerTest;
import org.firstinspires.ftc.teamcode.planners.tree.TreeStateTest;
import org.firstinspires.ftc.teamcode.util.autotune.SvaFitTest;
import org.firstinspires.ftc.teamcode.util.control.DoubleFlywheelFullVelocityControllerTest;
import org.firstinspires.ftc.teamcode.util.control.filters.ExtendedKalmanFilterConvergenceTest;
import org.firstinspires.ftc.teamcode.util.control.filters.ExtendedKalmanFilterTest;
import org.firstinspires.ftc.teamcode.util.game.AStarPathfinderTest;
import org.firstinspires.ftc.teamcode.util.game.AuditCorrectionsTest;
import org.firstinspires.ftc.teamcode.util.hardware.actuator.SmartFlywheelParamsTest;
import org.firstinspires.ftc.teamcode.util.hardware.gamepad.DriveVectorTest;
import org.firstinspires.ftc.teamcode.util.localization.VisionLocalizationMathTest;
import org.firstinspires.ftc.teamcode.util.shotcalc.AutoAimerCalculatorTest;
import org.firstinspires.ftc.teamcode.util.shotcalc.ShotCalculatorTest;
import org.junit.runner.RunWith;
import org.junit.runners.Suite;

@RunWith(Suite.class)
@Suite.SuiteClasses({
        RobotParkingTest.class,
        SimFakesTest.class,
        VisionHiveInferenceTest.class,
        EKFPinpointTuningMathTest.class,
        UtilityPosesTest.class,
        NullPlannerTest.class,
        TreeStateTest.class,
        SvaFitTest.class,
        DoubleFlywheelFullVelocityControllerTest.class,
        ExtendedKalmanFilterConvergenceTest.class,
        ExtendedKalmanFilterTest.class,
        AuditCorrectionsTest.class,
        AStarPathfinderTest.class,
        SmartFlywheelParamsTest.class,
        DriveVectorTest.class,
        VisionLocalizationMathTest.class,
        AutoAimerCalculatorTest.class,
        ShotCalculatorTest.class
})
public class UnitTestAll {
}
