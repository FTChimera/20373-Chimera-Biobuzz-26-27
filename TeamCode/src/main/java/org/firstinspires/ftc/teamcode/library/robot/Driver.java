package org.firstinspires.ftc.teamcode.library.robot;

/**
 * A handy Driver Preferences class for each RMS Chimera driver.
 * @author Team
 */
@SuppressWarnings("SpellCheckingInspection")
public enum Driver {
    ATHARVA(true),
    SHLOK(false),
    EASON(true),
    // ADD OTHER CHIMERA DRIVERS
    DEFAULT(false)
    ;
    public boolean USE_FIELD_CENTRIC;
    Driver(boolean fieldCentric) {
        USE_FIELD_CENTRIC = fieldCentric;
    }
}
