package org.firstinspires.ftc.teamcode.config;

import com.rmsftc.utils.AllianceColor;
import com.rmsftc.utils.MatchMode;
import com.rmsftc.utils.config.vars.Var;
import com.rmsftc.utils.config.vars.VarProvider;
import com.rmsftc.utils.ftc.utils.Storage;
import com.rmsftc.utils.ftc.utils.Value;

import org.firstinspires.ftc.teamcode.planners.CommonKt;
import org.firstinspires.ftc.teamcode.planners.StartPoses;
import com.rmsftc.utils.math.Pose;

/**
 * Cross-OpMode state (alliance, match mode, robot pose), exposed as
 * FTCUtils {@link Var}s backed by FTCUtils {@link Storage}
 * (SharedPreferences) instead of plain static fields, so values survive
 * OpMode switches — and app restarts.
 *
 * <p>Defaults are unchanged (BLUE, QUALIFICATIONS, garden start pose) and
 * apply when a key has never been stored. {@link #initialized} is the one
 * exception: it stays in memory so it is false after every app restart,
 * preserving the "auto has not run yet, reset to defaults" behavior in
 * teleop.
 */
public class OpModeStorage {

    private static final Value<AllianceColor> ALLIANCE_VALUE = new Value<AllianceColor>() {
        @Override public String format(AllianceColor value) { return value.name(); }
        @Override public AllianceColor parse(String stored) { return AllianceColor.valueOf(stored); }
    };

    private static final Value<MatchMode> MATCH_MODE_VALUE = new Value<MatchMode>() {
        @Override public String format(MatchMode value) { return value.name(); }
        @Override public MatchMode parse(String stored) { return MatchMode.valueOf(stored); }
    };

    private static final Value<Pose> POSE_VALUE = new Value<Pose>() {
        @Override public String format(Pose value) {
            return value.x() + "," + value.y() + "," + value.heading();
        }
        @Override public Pose parse(String stored) {
            String[] parts = stored.split(",");
            return new Pose(
                    Double.parseDouble(parts[0]),
                    Double.parseDouble(parts[1]),
                    Double.parseDouble(parts[2]));
        }
    };

    // Created lazily so loading this class never touches AppUtil
    // (unit tests load classes on a desktop JVM).
    private static Storage storage;

    private static Storage storage() {
        if (storage == null) storage = new Storage();
        return storage;
    }

    private static <T> Var<T> storedVar(String key, Value<T> type, T defaultValue) {
        VarProvider<T> provider = new VarProvider<T>() {
            @Override public T get() { return storage().get(key, type, defaultValue); }
            @Override public void set(T value) { storage().set(key, type, value); }
        };
        return new Var<T>(key, provider);
    }

    public static final Var<AllianceColor> allianceColor =
            storedVar("opmodeStorage.allianceColor", ALLIANCE_VALUE, AllianceColor.BLUE);
    public static final Var<MatchMode> matchMode =
            storedVar("opmodeStorage.matchMode", MATCH_MODE_VALUE, MatchMode.QUALIFICATIONS);
    public static final Var<Pose> pose =
            storedVar("opmodeStorage.pose", POSE_VALUE, StartPoses.gardenArea);
    /** In memory only (see class javadoc): false after every app restart. */
    public static final Var<Boolean> initialized = new Var<>(false);

    static {
        // Mirror the field poses for the stored alliance (was
        // resetToDefault() when these were plain statics; now the stored
        // alliance survives restarts, so only the mirroring remains).
        CommonKt.init(allianceColor.get());
    }

    public static void setEndVals(Pose newPose) {
        pose.set(newPose);
        initialized.set(true);
    }

    public static void endMatch() {
        initialized.set(false);
        resetToDefault();
    }

    public static void resetToDefault() {
        allianceColor.set(AllianceColor.BLUE);
        CommonKt.init(AllianceColor.BLUE);
        pose.set(StartPoses.gardenArea); // Chimera's testing start pose in robotics room
        matchMode.set(MatchMode.QUALIFICATIONS);
    }
}
