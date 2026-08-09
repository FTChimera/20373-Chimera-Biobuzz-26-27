package org.firstinspires.ftc.teamcode.library.robot;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.library.hardware.actuators.Direction;
import org.firstinspires.ftc.teamcode.library.hardware.gamepad.Button;
import org.firstinspires.ftc.teamcode.library.hardware.gamepad.GpBindings;
import org.firstinspires.ftc.teamcode.library.hardware.leds.RGBIndicator;
import org.firstinspires.ftc.teamcode.library.math.MathUtils;
import org.firstinspires.ftc.teamcode.library.hardware.webcam.Limelight;
import org.firstinspires.ftc.teamcode.mechanisms.Intake;
import org.firstinspires.ftc.teamcode.mechanisms.Mechanism;
import org.firstinspires.ftc.teamcode.mechanisms.RobotDrive;
import org.junit.Test;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertNotSame;
import static org.junit.Assert.assertTrue;
import static org.junit.Assert.assertThrows;

public class RobotConfigurationTest {
    @Test
    public void configureDriveMechanismsAndCreateMechanismUseRegisteredCopies() {
        RobotConstants bot = new RobotConstants().configureDriveMechanisms();
        assertTrue(bot.Mechanisms.contains(RobotDrive.class));

        RuntimeException exception = assertThrows(RuntimeException.class,
                () -> new RobotConstants().createMechanism(null, FakeMechanism.class));
        assertTrue(exception.getCause() instanceof IllegalArgumentException);

        FakeParams source = new FakeParams().value(7).label("configured");
        RobotConstants configured = new RobotConstants().addMechanism(FakeMechanism.class, source);
        source.value(99).label("mutated");

        FakeParams retrieved = configured.getMechanismParams(FakeMechanism.class, new FakeParams().value(1));
        assertNotSame(source, retrieved);
        assertEquals(7, retrieved.VALUE);
        assertEquals("configured", retrieved.LABEL);

        FakeMechanism mechanism = configured.createMechanism(null, FakeMechanism.class);
        assertEquals(8, mechanism.params.VALUE);
        assertEquals("configured", mechanism.params.LABEL);
        assertTrue(mechanism.params.UPDATED_FROM_BOT);

        FakeParams defaults = new FakeParams().value(4).label("default");
        FakeParams copiedDefault = new RobotConstants().getMechanismParams(FakeMechanism.class, defaults);
        defaults.value(12);
        assertEquals(4, copiedDefault.VALUE);
    }

    @Test
    public void robotProfilesExposeExpectedDriveAndOptionalMechanismConfiguration() {
        RobotConstants first = RobotConfigurations.FirstRobot();
        assertEquals("frontLeft", first.FRONT_LEFT_MOTOR_NAME);
        assertEquals(0.4, first.MAX_POWER, MathUtils.EPSILON);
        assertTrue(first.Mechanisms.contains(RobotDrive.class));
        assertFalse(first.Mechanisms.contains(Intake.class));
        assertFalse(first.USE_FOLLOWER_DRIVE);

        RobotConstants oldDecoder = RobotConfigurations.OldDECODERobotTesting();
        assertEquals("frontLeftMotor", oldDecoder.FRONT_LEFT_MOTOR_NAME);
        assertTrue(oldDecoder.Mechanisms.contains(RobotDrive.class));
        assertTrue(oldDecoder.Mechanisms.contains(Intake.class));
        assertTrue(oldDecoder.Mechanisms.contains(Limelight.class));
        assertTrue(oldDecoder.Mechanisms.contains(RGBIndicator.class));
        assertTrue(oldDecoder.USE_FOLLOWER_DRIVE);

        Intake.INTAKE_PARAMS intakeParams = oldDecoder.getMechanismParams(Intake.class, new Intake.INTAKE_PARAMS());
        assertEquals("intake", intakeParams.HARDWAREMAP_NAME);
        assertEquals(Direction.REVERSE, intakeParams.INTAKE_MOTOR_DIRECTION);

        RobotDrive.ROBOT_DRIVE_PARAMS robotDriveParams = oldDecoder.getMechanismParams(RobotDrive.class, new RobotDrive.ROBOT_DRIVE_PARAMS());
        assertEquals(Button.BACK, robotDriveParams.ZERO_HEADING_BUTTON);
    }

    public static class FakeMechanism extends Mechanism {
        public final FakeParams params;

        public FakeMechanism(HardwareMap map, RobotConstants bot) {
            super(map, bot);
            this.params = resolveParams(FakeMechanism.class, new FakeParams());
        }

        @Override
        public void periodic(GpBindings GpBindings) {}
    }

    public static class FakeParams implements Mechanism.Params<FakeParams> {
        public int VALUE = 0;
        public String LABEL = "unset";
        public boolean UPDATED_FROM_BOT = false;

        public FakeParams value(int value) {
            this.VALUE = value;
            return this;
        }

        public FakeParams label(String label) {
            this.LABEL = label;
            return this;
        }

        @Override
        public FakeParams copy() {
            return new FakeParams().value(VALUE).label(LABEL).updated(UPDATED_FROM_BOT);
        }

        @Override
        public FakeParams updateFromBot(RobotConstants bot) {
            UPDATED_FROM_BOT = true;
            VALUE += bot.Mechanisms.size();
            return this;
        }

        private FakeParams updated(boolean updatedFromBot) {
            this.UPDATED_FROM_BOT = updatedFromBot;
            return this;
        }
    }
}
