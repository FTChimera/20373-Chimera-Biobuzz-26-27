package org.firstinspires.ftc.teamcode.robotDev.customSW;

import static org.firstinspires.ftc.teamcode.mechanisms.Robot.LOW_POWER_VOLTAGE_THRESHOLD;
import static org.firstinspires.ftc.teamcode.mechanisms.Robot.LOW_POWER_VOLTAGE_TOLERANCE;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.Utility;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.teamcode.util.AllianceColor;
import org.firstinspires.ftc.teamcode.util.RobotMode;
import org.firstinspires.ftc.teamcode.util.Timer;
import org.firstinspires.ftc.teamcode.util.game.Field;

import java.util.concurrent.TimeUnit;


@Utility
public class TestRobotUtilMathGeomAndFeatureRegistrar extends OpMode {
    Field field;
    Timer opModeTimer;
    RobotMode robotMode = RobotMode.AUTONOMOUS;
    double lastVolt = Double.NaN;
    boolean isBatteryLowPower = false;
    double lastBatteryTime = System.currentTimeMillis();
    HardwareMap.DeviceMapping<VoltageSensor> voltageSensors;
    public double getBatteryVoltage() {
        double cur=System.currentTimeMillis();
        if (lastBatteryTime-cur < 300 && !Double.isNaN(lastVolt)) {
            return lastVolt;
        }
        lastBatteryTime=cur;
        double result = Double.POSITIVE_INFINITY;
        for (VoltageSensor sensor : voltageSensors) {
            double voltage = sensor.getVoltage();
            if (voltage > 0) {
                result = Math.min(result, voltage);
            }
        }
        if (result <= LOW_POWER_VOLTAGE_THRESHOLD-LOW_POWER_VOLTAGE_TOLERANCE) isBatteryLowPower=true;
        else if (result >= LOW_POWER_VOLTAGE_THRESHOLD+LOW_POWER_VOLTAGE_TOLERANCE) isBatteryLowPower=false;
        lastVolt = result;
        return result;
    }

    @Override
    public void init() {
        voltageSensors = hardwareMap.voltageSensor;
        field = new Field(AllianceColor.RED, 18, 18);
    }

    @Override
    public void start() {
        opModeTimer = new Timer(TimeUnit.SECONDS);
    }

    @Override
    public void loop() {
        telemetry.addData("Op Mode timer", opModeTimer.get());
        telemetry.addData("Delta Time", opModeTimer.dt(TimeUnit.MILLISECONDS));
        telemetry.addData("Battery Voltage", getBatteryVoltage());
        telemetry.addData("Is Battery Low Power", isBatteryLowPower);
    }

}
