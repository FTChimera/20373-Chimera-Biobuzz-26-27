package org.firstinspires.ftc.teamcode.pedro;

import com.pedropathing.algorithm.Foresight;
import com.pedropathing.revhub.drivetrains.Mecanum;
import com.pedropathing.revhub.localizers.PinpointLocalizer;
import com.pedropathing.tuning.autotune.Procedure;
import com.pedropathing.tuning.autotune.Tuner;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import org.firstinspires.ftc.teamcode.pedro.procedures.ColorSensorTuner;
import org.firstinspires.ftc.teamcode.pedro.procedures.ForesightTuner;
import org.firstinspires.ftc.teamcode.pedro.procedures.MecanumTuner;
import org.firstinspires.ftc.teamcode.pedro.procedures.PinpointTuner;
import org.firstinspires.ftc.teamcode.pedro.procedures.RobotDevelopmentTester;
import org.firstinspires.ftc.teamcode.pedro.procedures.ShooterPIDSVATuner;
import org.firstinspires.ftc.teamcode.pedro.procedures.Tests;

public class Tuning {
    // Tuners go here
    @Tuner(name = "Mecanum Tuner")
    public static Procedure mecanumTuner() { return new MecanumTuner(); }

    @Tuner(name = "Pinpoint Tuner")
    public static Procedure pinpointTuner() { return new PinpointTuner(); }

    @Tuner(name = "Foresight Tuner")
    public static Procedure foresightTuner() {
        return new ForesightTuner(
                hardwareMap -> new PinpointLocalizer(hardwareMap, Constants.PINPOINT_CONFIG),
                hardwareMap -> new Mecanum(hardwareMap, Constants.MECANUM_CONFIG));
    }

    @Tuner(name = "Color Sensor Tuner")
    public static Procedure colorSensorTuner() { return new ColorSensorTuner(); }

    @Tuner(name = "Tests")
    public static Procedure tests() {
        return new Tests(
                hardwareMap -> new Mecanum(hardwareMap, Constants.MECANUM_CONFIG),
                hardwareMap -> new PinpointLocalizer(hardwareMap, Constants.PINPOINT_CONFIG),
                () -> new Foresight(Constants.FORESIGHT_CONFIG),
                hardwareMap -> colorSensor(hardwareMap, "color_sensor", 65.3F));
    }

    @Tuner(name = "Robot Development Tester")
    public static Procedure robotDevTester() { return new RobotDevelopmentTester(
            hardwareMap -> {
                DcMotorEx dcMotorEx = hardwareMap.get(DcMotorEx.class, "intake");
                dcMotorEx.setDirection(DcMotorSimple.Direction.FORWARD);
                return dcMotorEx;
            },
            hardwareMap -> {
                DcMotorEx dcMotorEx = hardwareMap.get(DcMotorEx.class, "shooter");
                dcMotorEx.setDirection(DcMotorSimple.Direction.FORWARD); // TODO
                dcMotorEx.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                dcMotorEx.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(100, 0, 0, 30));
                return dcMotorEx;
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
                    DcMotorEx dcMotorEx = hardwareMap.get(DcMotorEx.class, "shooter");
                    dcMotorEx.setDirection(DcMotorSimple.Direction.FORWARD);
                    return dcMotorEx;
                },
                // TODO-USER-CONSTANTS: ticks per rev AT THE OUTPUT SHAFT
                // = encoderCpm * gearRatio (see SmartFlywheel.Params).
                // Placeholder 28.0 is from robotDev/AutoTuner/TuningConfig.java.
                28.0);
    }

    private static RevColorSensorV3 colorSensor(HardwareMap hardwareMap, String name, float gain) {
        RevColorSensorV3 colorSensorV3 = hardwareMap.get(RevColorSensorV3.class, name);
        colorSensorV3.setGain(gain);
        return colorSensorV3;
    }
}

