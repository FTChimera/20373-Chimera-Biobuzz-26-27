package org.firstinspires.ftc.teamcode.config;

import com.rmsftc.utils.AllianceColor;
import com.rmsftc.utils.MatchMode;
import com.rmsftc.utils.config.vars.Var;
import com.rmsftc.utils.config.vars.VarProvider;
import com.rmsftc.utils.ftc.utils.Storage;
import com.rmsftc.utils.ftc.utils.Value;

import org.firstinspires.ftc.teamcode.planners.CommonKt;
import org.firstinspires.ftc.teamcode.planners.Constants;
import org.firstinspires.ftc.teamcode.util.SimEnvironment;
import com.rmsftc.utils.math.Pose;

/**
 * Cross-OpMode state (alliance, match mode, robot pose), exposed as
 * FTCUtils {@link Var}s backed by FTCUtils {@link Storage}
 * (SharedPreferences) instead of plain static fields, so values survive
 * OpMode switches — and app restarts.
 *
 * <p>Defaults are unchanged (BLUE, QUALIFICATIONS, garden start pose) and
 * apply when a key has never been stored. {@link #initialized} is the one
 * exception: it stays in memory so it is false after every app restart, so
 * callers can tell no auto has run yet. Stored values are driver config and
 * are never reset automatically — the explicit reset is a ConfigOpMode
 * question ({@link #resetToDefault}).
 *
 * <p>Under the ftcsim simulator the stored Vars use in-memory values
 * (initialized to the defaults) instead of Storage: SharedPreferences
 * needs Android's AppUtil, which does not exist on the desktop JVM, and
 * a throw while this class initializes poisons it for the whole JVM.
 * On the robot the behaviour is exactly Storage-backed as before.
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

    // Created lazily so loading this class never touches AppUtil on a
    // desktop JVM (unit tests load classes there, and so does the ftcsim
    // simulator). Under the simulator the stored Vars below never reach
    // this field at all — their providers use in-memory values instead
    // (see storedVar). On the robot AppUtil exists, so this behaves
    // exactly as before.
    private static Storage storage;

    private static Storage storage() {
        if (storage == null) storage = new Storage();
        return storage;
    }

    private static <T> Var<T> storedVar(String key, Value<T> type, T defaultValue) {
        VarProvider<T> provider = new VarProvider<T>() {
            // In-memory fallback used under the simulator, where Storage
            // (SharedPreferences via AppUtil) does not exist: constructing
            // a Storage on a desktop JVM throws, and a throw from this
            // class's static initializer poisons the class for the whole
            // JVM (NoClassDefFoundError on every later use). One holder per
            // stored Var, initialized to the default; volatile so a set
            // from one thread is visible to a get from another. Checked on
            // every call so the sim/real decision always follows the
            // current SimEnvironment answer.
            private volatile T simValue = defaultValue;

            @Override public T get() {
                return SimEnvironment.isSimulation()
                        ? simValue
                        : storage().get(key, type, defaultValue);
            }

            @Override public void set(T value) {
                if (SimEnvironment.isSimulation()) {
                    simValue = value;
                } else {
                    storage().set(key, type, value);
                }
            }
        };
        return new Var<T>(key, provider);
    }

    public static final Var<AllianceColor> allianceColor =
            storedVar("opmodeStorage.allianceColor", ALLIANCE_VALUE, AllianceColor.BLUE);
    public static final Var<MatchMode> matchMode =
            storedVar("opmodeStorage.matchMode", MATCH_MODE_VALUE, MatchMode.QUALIFICATIONS);
    public static final Var<Pose> pose =
            storedVar("opmodeStorage.pose", POSE_VALUE, Constants.StartPoses.gardenArea);
    /** In memory only (see class javadoc): false after every app restart. */
    public static final Var<Boolean> initialized = new Var<>(false);

    static {
        // Mirror the field poses for the stored alliance (was
        // resetToDefault() when these were plain statics; now the stored
        // alliance survives restarts, so only the mirroring remains).
        // Kept (rather than relying only on the Robot ctor / ConfigOpMode
        // mirroring) so any first touch of this class mirrors the poses:
        // removing it would change robot behaviour for paths that read a
        // stored value without constructing a Robot. Under the simulator
        // this is safe now — allianceColor.get() reads the in-memory
        // default instead of constructing Storage.
        CommonKt.init(allianceColor.get());
    }

    public static void setEndVals(Pose newPose) {
        pose.set(newPose);
        initialized.set(true);
    }

    public static void endMatch() {
        // Only the in-memory flag clears. Stored alliance/mode/pose are
        // driver config and must survive: the old resetToDefault() call here
        // persisted BLUE/QUALIFICATIONS/garden after every TeleOp, destroying
        // the saved config the FTCUtils migration was meant to keep. The
        // explicit reset lives in ConfigOpMode.
        initialized.set(false);
    }

    public static void resetToDefault() {
        allianceColor.set(AllianceColor.RED);
        CommonKt.init(allianceColor.get());
        pose.set(Constants.StartPoses.gardenArea); // Chimera's testing start pose in robotics room
        matchMode.set(MatchMode.QUALIFICATIONS);
    }
}
