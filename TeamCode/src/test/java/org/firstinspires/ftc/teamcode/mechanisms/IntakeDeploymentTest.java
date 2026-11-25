package org.firstinspires.ftc.teamcode.mechanisms;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertTrue;

import com.pedropathing.revhub.drivetrains.CachedMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.rmsftc.utils.AllianceColor;

import org.firstinspires.ftc.teamcode.planners.Constants;
import org.junit.Test;

import java.lang.reflect.Proxy;

public class IntakeDeploymentTest {
    @Test
    public void deploymentUsesExistingPositionsAndUpdatesState() {
        double[] servoPosition = {0};
        Intake intake = intake(servoPosition, new double[]{0});
        assertFalse(intake.isDeployed());
        intake.setDeployed(true);
        assertTrue(intake.isDeployed());
        assertEquals(Constants.Intake.Deployer.DEPLOYED_POSITION + Constants.Intake.Deployer.TRANSLATION_FACTOR,
                servoPosition[0], 1e-9);
        intake.setDeployed(false);
        assertFalse(intake.isDeployed());
        assertEquals(Constants.Intake.Deployer.STOWED_POSITION + Constants.Intake.Deployer.TRANSLATION_FACTOR,
                servoPosition[0], 1e-9);
    }

    @Test
    public void emergencyStopClearsEjectAndStowsWithoutAnotherLoop() {
        double[] motorPower = {0};
        Intake intake = intake(new double[]{0}, motorPower);
        intake.setDeployed(true);
        intake.forceEject();
        intake.update();
        intake.stopImmediately();
        assertEquals(0, motorPower[0], 0);
        assertEquals(Intake.Mode.IDLE, intake.mode());
        assertFalse(intake.isDeployed());
        intake.update();
        assertEquals(0, motorPower[0], 0);
    }

    private static Intake intake(double[] servoPosition, double[] motorPower) {
        Servo servo = (Servo) Proxy.newProxyInstance(Servo.class.getClassLoader(),
                new Class<?>[]{Servo.class}, (proxy, method, args) -> {
                    if (method.getName().equals("setPosition")) servoPosition[0] = (double) args[0];
                    if (method.getReturnType() == double.class) return servoPosition[0];
                    return null;
                });
        DcMotorEx motor = (DcMotorEx) Proxy.newProxyInstance(DcMotorEx.class.getClassLoader(),
                new Class<?>[]{DcMotorEx.class}, (proxy, method, args) -> {
                    if (method.getName().equals("setPower")) motorPower[0] = (double) args[0];
                    if (method.getReturnType() == double.class) return motorPower[0];
                    if (method.getReturnType() == int.class) return 0;
                    if (method.getReturnType() == boolean.class) return false;
                    return null;
                });
        return new Intake(new CachedMotor(motor, 0.1), servo, AllianceColor.RED);
    }
}
