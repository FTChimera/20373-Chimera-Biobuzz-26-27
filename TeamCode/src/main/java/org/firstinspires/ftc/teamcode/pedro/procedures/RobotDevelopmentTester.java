package org.firstinspires.ftc.teamcode.pedro.procedures;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.ValueProvider;
import com.acmerobotics.dashboard.config.VariableProvider;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.pedropathing.tuning.autotune.DisplayName;
import com.pedropathing.tuning.autotune.Inputs;
import com.pedropathing.tuning.autotune.Procedure;
import com.pedropathing.tuning.autotune.TuningOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.mechanisms.Intake;
import org.firstinspires.ftc.teamcode.mechanisms.Shooter;
import org.firstinspires.ftc.teamcode.mechanisms.Transfer;

import java.util.function.Function;


public class RobotDevelopmentTester extends Procedure {
    Function<HardwareMap, Shooter> shooter;
    Function<HardwareMap, DcMotorEx> intake, transfer;

    public RobotDevelopmentTester(
            Function<HardwareMap, DcMotorEx> intake,
            Function<HardwareMap, Shooter> shooter,
            Function<HardwareMap, DcMotorEx> transfer
    ) {
        super("Robot Development Tester", "Test the intake, transfer, and both shooter flywheels.");
        this.shooter = shooter;
        this.intake = intake;
        this.transfer = transfer;
    }

    @Override
    public void run() throws InterruptedException {
        boolean
                isShooter = shooter!=null,
                isIntake = intake!=null,
                isTransfer = transfer!=null;
        confirmation("Configured Subsystems", "Shooter is"+ (isShooter? " ": " not ")+ "configured. \n Intake is"+(isIntake?" ":" not ") + "configured. \n Transfer is"+ (isTransfer?" ": " not ") + "configured. ");
        Inputs testerType = inputs("Tester Type", "Enter which subsytems to include in tester");
        Inputs.Field<Boolean> useShooter = testerType.b("Use Shooter").withDefault(false);
        Inputs.Field<Boolean> useIntake = testerType.b("Use Intake").withDefault(false);
        Inputs.Field<Boolean> useTransfer = testerType.b("Use Transfer").withDefault(false);
        Inputs.Field<DevelopmentInputType> developmentTestingType = testerType.e("Development Testing Type", DevelopmentInputType.class).withDefault(DevelopmentInputType.FTC_DASH);
        awaitInputs(testerType);

        boolean completed = runOpMode(new ThreeSubsystemTester(
                shooter, transfer, intake,
                new MechanismUsage(isShooter, useShooter.get(), isIntake, useIntake.get(), isTransfer, useTransfer.get()),
                developmentTestingType.get()
        ));

        result("Completed", completed);
    }
}

class ThreeSubsystemTester extends TuningOpMode<Boolean> {
    private static final double DEFAULT_POWER = 0.8;
    private static final int DEFAULT_RPM = 1000;

    Function<HardwareMap, Shooter> shooter;
    Function<HardwareMap, DcMotorEx> transfer, intake;
    boolean useShooter,useTransfer,useIntake;
    DevelopmentInputType devType;
    private final ValueProvider<Double> powerProvider = new VariableProvider<>(DEFAULT_POWER);
    private final ValueProvider<Integer> rpmProvider = new VariableProvider<>(DEFAULT_RPM);
    private boolean dashboardInputsRegistered = false;

    public ThreeSubsystemTester(
            Function<HardwareMap, Shooter> shooter,
            Function<HardwareMap, DcMotorEx> transfer,
            Function<HardwareMap, DcMotorEx> intake,
            MechanismUsage mechUsage,
            DevelopmentInputType developmentInputType
    ) {
        super("Three Subsystem Tester", "Tests some of the 3 subsystems", true);
        this.shooter = shooter;
        this.transfer = transfer;
        this.intake = intake;
        this.useShooter = mechUsage.shooter();
        this.useTransfer = mechUsage.transfer();
        this.useIntake = mechUsage.intake();
        this.devType = developmentInputType;
    }

    @Override
    protected Boolean runTuningOpMode() throws InterruptedException {
        DcMotorEx intake = null, transfer = null;
        Shooter shooter = null;
        if (useIntake) intake = this.intake.apply(hardwareMap);
        if (useTransfer) transfer = this.transfer.apply(hardwareMap);
        if (useShooter) shooter = this.shooter.apply(hardwareMap);

        configureDashboardInputs();
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        telemetry.clearAll();
        telemetry.addData("Development Input", devType);
        telemetry.addData("Use Intake", useIntake);
        telemetry.addData("Use Transfer", useTransfer);
        telemetry.addData("Use Shooter", useShooter);
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            double power = getPower();
            int rpm = getRPM();

            if (useIntake) {
                intake.setPower(power);
            }
            if (useTransfer) {
                transfer.setPower(power);
            }
            if (useShooter) {
                shooter.setVelocity(rpm, rpm);
            }

            telemetry.addData("Development Input", devType);
            telemetry.addData("Target Power", power);
            telemetry.addData("Target RPM", rpm);
            if (useIntake) {
                telemetry.addData("Intake Power", intake.getPower());
            }
            if (useTransfer) {
                telemetry.addData("Transfer Power", transfer.getPower());
            }
            if (useShooter) {
                telemetry.addData("Shooter", shooter.getDebugString());
            }
            telemetry.update();
        }

        if (useIntake) {
            intake.setPower(0);
        }
        if (useTransfer) {
            transfer.setPower(0);
        }
        if (useShooter) {
            shooter.stopImmediately();
        }

        return true;
    }

    double getPower() {
        switch (devType) {
            case FTC_DASH:
                return powerProvider.get();
            case GAMEPADS:
                return gamepad1.right_trigger - gamepad1.left_trigger;
            case PRESET_POWER:
            default:
                return DEFAULT_POWER;
        }
    }
    int getRPM() {
        switch (devType) {
            case FTC_DASH:
                return rpmProvider.get();
            case GAMEPADS:
                return (int) Math.round(DEFAULT_RPM * (gamepad1.right_trigger + gamepad1.left_trigger));
            case PRESET_POWER:
            default:
                return DEFAULT_RPM;
        }
    }

    private void configureDashboardInputs() {
        if (devType != DevelopmentInputType.FTC_DASH || dashboardInputsRegistered) {
            return;
        }
        FtcDashboard.getInstance().addConfigVariable("ThreeSubsystemTester", "Power", powerProvider);
        FtcDashboard.getInstance().addConfigVariable("ThreeSubsystemTester", "RPM", rpmProvider);
        dashboardInputsRegistered = true;
    }
}


class MechanismClassTester extends TuningOpMode<Boolean> {
    private static final double DEFAULT_POWER = 0.8;
    private static final int DEFAULT_RPM = 1000;

    DevelopmentInputType devType;
    MechanismUsage m;
    Function<HardwareMap, Intake> intakeFunction;
    Function<HardwareMap, Transfer> transferFunction;
    Function<HardwareMap, Shooter> shooterFunction;
    private final ValueProvider<Double> powerProvider = new VariableProvider<>(DEFAULT_POWER);
    private final ValueProvider<Integer> rpmProvider = new VariableProvider<>(DEFAULT_RPM);
    private boolean dashboardInputsRegistered = false;

    public MechanismClassTester(Function<HardwareMap, Intake> intakeFunction, Function<HardwareMap, Transfer> transferFunction, Function<HardwareMap, Shooter> shooterFunction, DevelopmentInputType devType, MechanismUsage m) {
        super("Mechanism Class Tester", "Tests a single mechanism class", true);
        this.devType = devType;
        this.m = m;
        this.intakeFunction = intakeFunction;
        this.transferFunction = transferFunction;
        this.shooterFunction = shooterFunction;
    }

    @Override
    protected Boolean runTuningOpMode() throws InterruptedException {
        Intake intake = null;
        Transfer transfer = null;
        Shooter shooter = null;
        if (m.intake()) intake = this.intakeFunction.apply(hardwareMap);
        if (m.transfer()) transfer = this.transferFunction.apply(hardwareMap);
        if (m.shooter()) shooter = this.shooterFunction.apply(hardwareMap);

        configureDashboardInputs();
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        telemetry.clearAll();
        telemetry.addData("Development Input", devType);
        telemetry.addData("Use Intake", m.intake());
        telemetry.addData("Use Transfer", m.transfer());
        telemetry.addData("Use Shooter", m.shooter());
        telemetry.update();

        waitForStart();
        while (opModeIsActive()) {
            double power = getPower();
            int rpm = getRPM();

            if (m.intake()) {
                intake.setCustomPower(power);
                intake.update();
            }
            if (m.transfer()) {
                transfer.customPower(power);
                transfer.update();
            }
            if (m.shooter()) {
                shooter.setVelocity(rpm, rpm);
                shooter.update();
            }

            telemetry.addData("Development Input", devType);
            telemetry.addData("Target Power", power);
            telemetry.addData("Target RPM", rpm);
            if (m.intake()) {
                telemetry.addData("Intake Mode", intake.mode());
                telemetry.addData("Intake Debug", intake.getDebugString());
            }
            if (m.transfer()) {
                telemetry.addData("Transfer Mode", transfer.mode());
                telemetry.addData("Transfer Debug", transfer.getDebugString());
            }
            if (m.shooter()) {
                telemetry.addData("Shooter Debug", shooter.getDebugString());
            }
            telemetry.update();
        }

        if (m.intake()) {
            intake.stopImmediately();
            intake.update();
        }
        if (m.transfer()) {
            transfer.stopImmediately();
            transfer.update();
        }
        if (m.shooter()) {
            shooter.stopImmediately();
            shooter.update();
        }

        return true;
    }

    private double getPower() {
        switch (devType) {
            case FTC_DASH:
                return powerProvider.get();
            case GAMEPADS:
                return gamepad1.right_trigger - gamepad1.left_trigger;
            case PRESET_POWER:
            default:
                return DEFAULT_POWER;
        }
    }

    private int getRPM() {
        switch (devType) {
            case FTC_DASH:
                return rpmProvider.get();
            case GAMEPADS:
                return (int) Math.round(DEFAULT_RPM * (gamepad1.right_trigger + gamepad1.left_trigger));
            case PRESET_POWER:
            default:
                return DEFAULT_RPM;
        }
    }

    private void configureDashboardInputs() {
        if (devType != DevelopmentInputType.FTC_DASH || dashboardInputsRegistered) {
            return;
        }
        FtcDashboard.getInstance().addConfigVariable("MechanismClassTester", "Power", powerProvider);
        FtcDashboard.getInstance().addConfigVariable("MechanismClassTester", "RPM", rpmProvider);
        dashboardInputsRegistered = true;
    }

}

enum DevelopmentInputType {
    @DisplayName("Use FTC Dashboard to control the RPM and speed")
    FTC_DASH,
    @DisplayName("Use gamepads/driver station to control RPM and speed")
    GAMEPADS,
    @DisplayName("Run at full/preset power the entire time")
    PRESET_POWER
}

class MechanismUsage {
    private boolean useShooter;
    private boolean useIntake;
    private boolean useTransfer;

    public MechanismUsage(
            boolean isShooter, boolean shouldShooter,
            boolean isIntake, boolean shouldIntake,
            boolean isTransfer, boolean shouldTransfer
    ) {
        this.useShooter = isShooter && shouldShooter;
        this.useIntake = isIntake && shouldIntake;
        this.useTransfer = isTransfer && shouldTransfer;
    }

    public boolean shooter() {
        return useShooter;
    }

    public boolean intake() {
        return useIntake;
    }

    public boolean transfer() {
        return useTransfer;
    }
}

enum TestingType {
    @DisplayName("Test the selected mechanism wrapper classes")
    WRAPPER,
    @DisplayName("Test the raw FTC SDK versions of the selected mechanisms (LIMITED")
    RAW
}