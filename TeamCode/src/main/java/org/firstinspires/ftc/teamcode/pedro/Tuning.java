package org.firstinspires.ftc.teamcode.pedro;

import com.pedropathing.algorithm.Foresight;
import com.pedropathing.revhub.drivetrains.Mecanum;
import com.pedropathing.revhub.localizers.PinpointLocalizer;
import com.pedropathing.tuning.autotune.Procedure;
import com.pedropathing.tuning.autotune.Tuner;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.mechanisms.Shooter;
import org.firstinspires.ftc.teamcode.pedro.procedures.ColorSensorTuner;
import org.firstinspires.ftc.teamcode.pedro.procedures.EKFPinpointTuner;
import org.firstinspires.ftc.teamcode.pedro.procedures.ForesightTuner;
import org.firstinspires.ftc.teamcode.pedro.procedures.MecanumTuner;
import org.firstinspires.ftc.teamcode.pedro.procedures.PinpointTuner;
import org.firstinspires.ftc.teamcode.pedro.procedures.RobotDevelopmentTester;
import org.firstinspires.ftc.teamcode.pedro.procedures.ShooterPIDSVATuner;
import org.firstinspires.ftc.teamcode.pedro.procedures.Tests;
import org.firstinspires.ftc.teamcode.util.AllianceColor;

public class Tuning {
    // Tuners go here
    @Tuner(name = "Mecanum Tuner")
    public static Procedure mecanumTuner() { return new MecanumTuner(); }

    @Tuner(name = "Pinpoint Tuner")
    public static Procedure pinpointTuner() { return new PinpointTuner(); }

    @Tuner(name = "EKF Pinpoint Tuner")
    public static Procedure ekfPinpointTuner() { return new EKFPinpointTuner(); }

    @Tuner(name = "Foresight Tuner")
    public static Procedure foresightTuner() {
        return new ForesightTuner(
                hardwareMap -> new EKFPinpoint(hardwareMap, Constants.PINPOINT_CONFIG),
                hardwareMap -> new Mecanum(hardwareMap, Constants.MECANUM_CONFIG));
    }

    @Tuner(name = "Color Sensor Tuner")
    public static Procedure colorSensorTuner() { return new ColorSensorTuner(); }

    @Tuner(name = "Tests")
    public static Procedure tests() {
        return new Tests(
                hardwareMap -> new Mecanum(hardwareMap, Constants.MECANUM_CONFIG),
                hardwareMap -> new EKFPinpoint(hardwareMap, Constants.PINPOINT_CONFIG),
                () -> new Foresight(Constants.FORESIGHT_CONFIG),
                hardwareMap -> colorSensor(hardwareMap, "color_sensor", 65.3F));
    }

    @Tuner(name = "Robot Development Tester 1")
    public static Procedure robotDevTester1() { return new RobotDevelopmentTester(
            hardwareMap -> {
                DcMotorEx dcMotorEx = hardwareMap.get(DcMotorEx.class, "intake");
                dcMotorEx.setDirection(DcMotorSimple.Direction.FORWARD);
                return dcMotorEx;
            },
            hardwareMap -> {
                return new Shooter(hardwareMap, AllianceColor.BLUE);
            },
            hardwareMap -> {
                DcMotorEx dcMotorEx = hardwareMap.get(DcMotorEx.class, "transfer");
                dcMotorEx.setDirection(DcMotorSimple.Direction.FORWARD);
                return dcMotorEx;
            }
    ); }

    @Tuner(name = "Robot Development Tester 2")
    public static Procedure robotDevTester2() { return new RobotDevelopmentTester(
            hardwareMap -> {
                DcMotorEx dcMotorEx = hardwareMap.get(DcMotorEx.class, "intake");
                dcMotorEx.setDirection(DcMotorSimple.Direction.REVERSE);
                return dcMotorEx;
            },
            hardwareMap -> {
                return new Shooter(hardwareMap, AllianceColor.BLUE);
            },
            hardwareMap -> {
                DcMotorEx dcMotorEx = hardwareMap.get(DcMotorEx.class, "transfer");
                dcMotorEx.setDirection(DcMotorSimple.Direction.FORWARD);
                return dcMotorEx;
            }
    ); }

    @Tuner(name = "Shooter PIDSVA Tuner")
    public static Procedure shooterPIDSVATuner() {
        return new ShooterPIDSVATuner(
                hardwareMap -> {
                    DcMotorEx dcMotorEx = hardwareMap.get(DcMotorEx.class, "mainShooter");
                    dcMotorEx.setDirection(DcMotorSimple.Direction.FORWARD);
                    return dcMotorEx;
                },
                // TODO-USER-CONSTANTS: ticks per rev AT THE OUTPUT SHAFT
                // = encoderCpm * gearRatio (see SmartFlywheel.Params).
                // Placeholder 28.0 is from robotDev/AutoTuner/TuningConfig.java.
                28.0);
    }

    @Tuner(name = "CounterRoller PIDSVA Tuner")
    public static Procedure counterRollerPIDSVATuner() {
        return new ShooterPIDSVATuner(
                hardwareMap -> {
                    DcMotorEx dcMotorEx = hardwareMap.get(DcMotorEx.class, "counterShooter");
                    // REVERSE, matching production Shooter.java (flipped in
                    // the Nov 15 tuning). The tuner learns the direction
                    // signs from the setup OpModes, so it spins the wheel
                    // in its shoot direction either way.
                    dcMotorEx.setDirection(DcMotorSimple.Direction.REVERSE);
                    return dcMotorEx;
                },
                // TODO-USER-CONSTANTS: ticks per rev AT THE OUTPUT SHAFT
                // = encoderCpm * gearRatio (see SmartFlywheel.Params).
                // Placeholder 28.0 is from robotDev/AutoTuner/TuningConfig.java.
                // Same motor type as mainMotor (28 ticks/rev).
                // Note: the tuner has no "Reversed" input -- it reads
                // getVelocity(), which is direction-adjusted, so a
                // positive power still gives a positive velocity reading.
                28.0);
    }

    @Tuner(name = "Dual Shooter PIDSVA Tuner")
    public static Procedure dualShooterPIDSVATuner() {
        // Tunes both flywheels at the same time, same algorithm: both
        // wheels spin together in every phase, each with its own fit
        // and validation. The manual-spin setup tracks both wheels in
        // one OpMode (spin the main by hand, then the counter roller
        // by hand, then stop it). Results are reported per motor
        // ("main.*" / "counter.*").
        return new ShooterPIDSVATuner(
                java.util.List.of(
                        new ShooterPIDSVATuner.MotorSpec("main", hardwareMap -> {
                            DcMotorEx dcMotorEx = hardwareMap.get(DcMotorEx.class, "mainShooter");
                            dcMotorEx.setDirection(DcMotorSimple.Direction.FORWARD);
                            return dcMotorEx;
                        }),
                        new ShooterPIDSVATuner.MotorSpec("counter", hardwareMap -> {
                            DcMotorEx dcMotorEx = hardwareMap.get(DcMotorEx.class, "counterShooter");
                            // REVERSE, matching production Shooter.java.
                            dcMotorEx.setDirection(DcMotorSimple.Direction.REVERSE);
                            return dcMotorEx;
                        })),
                // TODO-USER-CONSTANTS: ticks per rev AT THE OUTPUT SHAFT
                // = encoderCpm * gearRatio (see SmartFlywheel.Params).
                // Placeholder 28.0 is from robotDev/AutoTuner/TuningConfig.java.
                // Same motor type for both flywheels (28 ticks/rev).
                28.0);
    }

    private static RevColorSensorV3 colorSensor(HardwareMap hardwareMap, String name, float gain) {
        RevColorSensorV3 colorSensorV3 = hardwareMap.get(RevColorSensorV3.class, name);
        colorSensorV3.setGain(gain);
        return colorSensorV3;
    }
}
