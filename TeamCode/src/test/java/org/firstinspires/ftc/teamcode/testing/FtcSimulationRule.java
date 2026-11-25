package org.firstinspires.ftc.teamcode.testing;

import com.rmsftc.utils.AllianceColor;
import com.rmsftc.utils.MatchMode;
import com.rmsftc.utils.math.Pose;

import org.firstinspires.ftc.teamcode.config.OpModeStorage;
import org.firstinspires.ftc.teamcode.mechanisms.Shooter;
import org.firstinspires.ftc.teamcode.planners.CommonKt;
import org.firstinspires.ftc.teamcode.pedro.Constants;
import com.pedropathing.revhub.localizers.PinpointLocalizer;
import org.firstinspires.ftc.teamcode.util.SimEnvironment;
import org.firstinspires.ftc.teamcode.util.FakeClock;
import org.junit.rules.TestRule;
import org.junit.runner.Description;
import org.junit.runners.model.Statement;

/** Scopes desktop services and shared robot state to one JUnit test, even on failure. */
public final class FtcSimulationRule implements TestRule {
    public final FakeClock clock = new FakeClock();

    @Override
    public Statement apply(Statement base, Description description) {
        return new Statement() {
            @Override
            public void evaluate() throws Throwable {
                Boolean previousOverride = SimEnvironment.override;
                String previousProperty = System.getProperty("ftcsim.jvm");
                SimEnvironment.setOverride(true);
                System.setProperty("ftcsim.jvm", "true");
                try {
                    // Set simulation BEFORE first touching the storage class's initializer.
                    AllianceColor alliance = OpModeStorage.allianceColor.get();
                    MatchMode matchMode = OpModeStorage.matchMode.get();
                    Pose pose = OpModeStorage.pose.get();
                    boolean initialized = OpModeStorage.initialized.get();
                    boolean sotm = Shooter.useSotm.get();
                    boolean manualBrake = Constants.FOLLOWER_MANUAL_BRAKE_MODE.get();
                    boolean brakeAtEnd = Constants.FOLLOWER_BRAKE_AT_END.get();
                    double maxPathSpeed = Constants.FOLLOWER_MAX_PATH_SPEED.get();
                    PinpointLocalizer.ResetMode resetMode = Constants.PINPOINT_CONFIG.resetMode.get();
                    clock.set(0);
                    try (DaemonRegistryScope ignored = new DaemonRegistryScope()) {
                        base.evaluate();
                    } finally {
                        SimEnvironment.setOverride(true);
                        OpModeStorage.allianceColor.set(alliance);
                        OpModeStorage.matchMode.set(matchMode);
                        OpModeStorage.pose.set(pose);
                        OpModeStorage.initialized.set(initialized);
                        Shooter.useSotm.set(sotm);
                        CommonKt.init(alliance);
                        Constants.FOLLOWER_MANUAL_BRAKE_MODE.set(manualBrake);
                        Constants.FOLLOWER_BRAKE_AT_END.set(brakeAtEnd);
                        Constants.FOLLOWER_MAX_PATH_SPEED.set(maxPathSpeed);
                        Constants.PINPOINT_CONFIG.resetMode.set(resetMode);
                    }
                } finally {
                    SimEnvironment.setOverride(previousOverride);
                    if (previousProperty == null) {
                        System.clearProperty("ftcsim.jvm");
                    } else {
                        System.setProperty("ftcsim.jvm", previousProperty);
                    }
                }
            }
        };
    }
}
