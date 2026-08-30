package org.firstinspires.ftc.teamcode.mechanisms;

import com.pedropathing.ivy.Scheduler;
import com.pedropathing.ivy.commands.Commands;
import com.pedropathing.geometry.Pose;
import com.pedropathing.math.Vector;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.configs.Blackboard;
import org.firstinspires.ftc.teamcode.library.DriveVector;
import org.firstinspires.ftc.teamcode.library.OpModeHelper;
import org.firstinspires.ftc.teamcode.library.game.RobotMode;
import org.firstinspires.ftc.teamcode.library.game.State;
import org.firstinspires.ftc.teamcode.library.robot.RobotConstants;
import org.firstinspires.ftc.teamcode.library.robot.RobotIvyBuilder;

import java.util.Collection;
import java.util.Collections;
import java.util.Comparator;
import java.util.LinkedHashMap;
import java.util.Map;
import java.util.Objects;
import java.util.function.DoubleUnaryOperator;
import java.util.Set;
import java.util.function.Supplier;

/**
 * Aggregate access to every profile-managed mechanism registered on a robot profile.
 */
public class Robot extends Mechanism {
    private final Map<Class<? extends Mechanism>, Mechanism> mechanisms = new LinkedHashMap<>();
    private final RobotMode robotMode;
    private State lastState;
    OpModeHelper helper;

    public Robot(HardwareMap map, RobotConstants bot) {
        this(map, bot, RobotMode.TELEOP);
    }

    public Robot(HardwareMap map, RobotConstants bot, RobotMode robotMode) {
        super(map, bot);
        this.robotMode = Objects.requireNonNull(robotMode, "robotMode");
        helper = new OpModeHelper(hardwareMap);
        bot.Mechanisms.stream()
                .filter(mechanismClass -> mechanismClass != Robot.class)
                .sorted(Comparator.comparing(Class::getName))
                .forEach(mechanismClass -> mechanisms.put(mechanismClass, bot.createMechanism(map, mechanismClass)));
    }

    public void start() {
        helper.start();
        startVision(Vision.Pipeline.APRIL_TAG);
    }
    public Set<Class<? extends Mechanism>> getConfiguredMechanismClasses() {
        return Collections.unmodifiableSet(mechanisms.keySet());
    }

    public RobotMode getRobotMode() {
        return robotMode;
    }

    /** Creates Ivy commands bound to this robot's configured mechanisms. */
    public RobotIvyBuilder build() {
        return new RobotIvyBuilder(this);
    }

    public Collection<Mechanism> all() {
        return Collections.unmodifiableCollection(mechanisms.values());
    }

    public boolean has(Class<? extends Mechanism> mechanismClass) {
        return mechanisms.containsKey(mechanismClass);
    }

    public <M extends Mechanism> M get(Class<M> mechanismClass) {
        Mechanism mechanism = mechanisms.get(mechanismClass);
        return mechanism == null ? null : mechanismClass.cast(mechanism);
    }

    public <M extends Mechanism> M require(Class<M> mechanismClass) {
        M mechanism = get(mechanismClass);
        if (mechanism == null) {
            throw new IllegalStateException("Mechanism not aggregated: " + mechanismClass.getSimpleName());
        }
        return mechanism;
    }

    /** Returns the drive mechanism, or {@code null} when it is not configured. */
    public RobotDrive getDrive() {
        return get(RobotDrive.class);
    }

    /** Returns the profile-configured drive mechanism. */
    public RobotDrive requireDrive() {
        return require(RobotDrive.class);
    }

    /** Returns the intake mechanism, or {@code null} when it is not configured. */
    public Intake getIntake() {
        return get(Intake.class);
    }

    /** Returns the profile-configured intake mechanism. */
    public Intake requireIntake() {
        return require(Intake.class);
    }

    /** Returns the vision mechanism, or {@code null} when it is not configured. */
    public Vision getVision() {
        return get(Vision.class);
    }

    /** Returns the profile-configured vision mechanism. */
    public Vision requireVision() {
        return require(Vision.class);
    }

    public void startIntake() {
        if (all().contains(Intake.class)) requireIntake().FullPower();
    }

    public void startIntakeHalfPower() {
        if (all().contains(Intake.class)) requireIntake().HalfPower();
    }

    public void reverseIntake() {
        if (all().contains(Intake.class)) requireIntake().ReverseFullPower();
    }

    public void stopIntake() {
        if (all().contains(Intake.class)) requireIntake().Stop();
    }

    public double getIntakePower() {
        return all().contains(Intake.class)? getIntake().getTargetPower(): 0;
    }

    public void setIntakePower(double power) {
        if (all().contains(Intake.class)) requireIntake().setPower(power);
    }

    public void setStartingPose(Pose pose) {
        requireDrive().setStartingPose(pose);
    }

    public void setPose(Pose pose) {
        requireDrive().setPose(pose);
    }

    public Pose getPose() {
        return requireDrive().getPose();
    }

    public void startTeleOpDrive() {
        requireDrive().startTeleOpDrive();
    }

    public void setTeleOpDrive(DriveVector vector) {
        requireDrive().setTeleOpDrive(vector);
    }

    public void setTeleOpDrive(DriveVector vector, boolean fieldCentric) {
        requireDrive().setTeleOpDrive(vector, fieldCentric);
    }

    public void setDriveStickCurve(DoubleUnaryOperator curve) {
        requireDrive().setStickCurve(curve);
    }

    public void zeroDriveHeading() {
        requireDrive().zeroHeadingOffset();
    }

    public void startVision(Vision.Pipeline pipeline) {
        if (all().contains(Vision.class)) requireVision().start(pipeline);
    }

    public void stopVision() {
        if (all().contains(Vision.class)) requireVision().stop();
    }

    public void setVisionPipeline(Vision.Pipeline pipeline) {
        if (all().contains(Vision.class)) requireVision().setPipeline(pipeline);
    }

    /**
     * Applies the latest valid Limelight MT2 pose to the drive localizer.
     *
     * @return whether a valid vision pose was available and applied
     */
    public boolean forceRelocalize() {
        return requireVision().forceRelocalize(requireDrive());
    }

    public State getLastState() {
        return lastState;
    }

    @Override
    public void periodic(State state) {
        periodic(state, robotMode);
    }

    @Override
    public void periodic(State state, RobotMode periodicMode) {
        Objects.requireNonNull(state, "state");
        if (periodicMode != robotMode) {
            throw new IllegalArgumentException("Robot mode does not match the configured robot mode.");
        }
        helper.update();
        state.setRobotMode(robotMode);
        state.setOpModeHelper(helper);
        state.setRobotDrive(getDrive());
        lastState = state;
        for (Mechanism mechanism : mechanisms.values()) {
            mechanism.periodic(state, robotMode);
        }
    }

    public void schedule(Supplier<State> stateSupplier) {
        Objects.requireNonNull(stateSupplier, "State Supplier");
        Scheduler.schedule(Commands.infinite(() -> periodic(stateSupplier.get())));
    }

    @Override
    public void stop() {
        stop(true);
    }
    public void stop(boolean saveEndVals) {
        for (Mechanism m: all()) {
            m.stop();
        }
        helper.close();
        if (saveEndVals) saveEndVals();
    }

    public void saveEndVals() {
        if (robotMode == RobotMode.AUTONOMOUS) {
            if (lastState == null) {
                // No periodic update ran, so there is no reliable autonomous state to hand off.
                return;
            }
            Pose estimatedEndPose = requireDrive().getPose().copy();
            Vector robotV = requireDrive().getVelocity().times(helper.getDt() / 1_000.0);
            estimatedEndPose = estimatedEndPose.plus(
                    new Pose(
                            robotV.getXComponent(),
                            robotV.getYComponent()
                    )
            );
            Blackboard.put(Blackboard.Caption.POSE, estimatedEndPose);
            if (lastState.getAllianceColor() != null) {
                Blackboard.put(Blackboard.Caption.ALLIANCE, lastState.getAllianceColor());
            }
        } else {
            for (Blackboard.Item i: Blackboard.getAll()) {
                if (i.getCaption()!= Blackboard.Caption.DRIVER) {
                    i.reset();
                }
            }
        }
    }

    public double getOpModeTimeSeconds() {
        return helper.getOpModeTime();
    }
}
