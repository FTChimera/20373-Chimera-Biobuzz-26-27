package org.firstinspires.ftc.teamcode.pedro;

import com.pedropathing.revhub.drivetrains.Mecanum;
import com.pedropathing.revhub.localizers.PinpointLocalizer;
import com.pedropathing.tuning.autotune.Procedure;
import com.pedropathing.tuning.autotune.Tuner;
import com.pedropathing.tuning.autotune.TunerRegistrar;
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
import org.firstinspires.ftc.teamcode.pedro.procedures.Tests;

public class Tuning {
    // Tuners go here
    private static RevColorSensorV3 colorSensor(HardwareMap hardwareMap, String name, float gain) {
        RevColorSensorV3 colorSensorV3 = hardwareMap.get(RevColorSensorV3.class, name);
        colorSensorV3.setGain(gain);
        return colorSensorV3;
    }

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



    static {
        // REGISTER ALL TUNERS HERE
        // add TunerRegistrar.register(name, procedure) here
        // todo: also if tuning does not work then add back @Tuner
        // and all the functions

        // ok

        TunerRegistrar.deregisterAll();
        TunerRegistrar.register("Mecanum Tuner", new MecanumTuner());
        TunerRegistrar.register("Pinpoint Tuner", new PinpointTuner());
        TunerRegistrar.register("Foresight Tuner", new ForesightTuner(
                hardwareMap -> new PinpointLocalizer(hardwareMap, Constants.PINPOINT_CONFIG),
                hardwareMap -> new Mecanum(hardwareMap, Constants.MECANUM_CONFIG)
        ));
        TunerRegistrar.register("Color Sensor Tuner", new ColorSensorTuner());
        TunerRegistrar.register("Tests", new Tests(
                hardwareMap -> new Mecanum(hardwareMap, Constants.MECANUM_CONFIG),
                null,
                null,
                hardwareMap -> colorSensor(hardwareMap, "color_sensor", 65.3F)));
        TunerRegistrar.register("Robot Dev Tester", robotDevTester());
    }
}
