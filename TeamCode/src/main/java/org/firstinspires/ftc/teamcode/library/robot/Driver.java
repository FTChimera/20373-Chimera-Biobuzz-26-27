package org.firstinspires.ftc.teamcode.library.robot;

/**
 * A handy Driver Preferences class for each RMS Chimera driver.
 * @author Team
 */
@SuppressWarnings("SpellCheckingInspection")
public class Driver {
    public static Driver ATHARVA = new Driver(true);
    public static Driver SHLOK = new Driver(true);
    public static Driver EASON = new Driver(true);
    public static Driver DEFAULT = new Driver(true);
    public static Driver SIMPLE = new Driver(false);
    // ADD OTHER CHIMERA DRIVERS

    public static Driver create(boolean fieldCentric) {
        return new Driver(fieldCentric);
    }
    public boolean USE_FIELD_CENTRIC;
    Driver(boolean fieldCentric) {
        USE_FIELD_CENTRIC = fieldCentric;
    }
}
