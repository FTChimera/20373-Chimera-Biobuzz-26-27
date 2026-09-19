package org.firstinspires.ftc.teamcode.mechanisms;

import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.library.ShotSolver;
import org.firstinspires.ftc.teamcode.library.control.BangBangController;
import org.firstinspires.ftc.teamcode.library.control.DoubleBangBangConstants;
import org.firstinspires.ftc.teamcode.library.control.DoubleBangBangController;
import org.firstinspires.ftc.teamcode.library.control.DoubleCompositeConstants;
import org.firstinspires.ftc.teamcode.library.control.DoubleCompositeController;
import org.firstinspires.ftc.teamcode.library.control.DoublePIDFCoeffs;
import org.firstinspires.ftc.teamcode.library.control.DoublePIDFController;
import org.firstinspires.ftc.teamcode.library.game.State;
import org.firstinspires.ftc.teamcode.library.hardware.actuators.Direction;
import org.firstinspires.ftc.teamcode.library.hardware.actuators.Motor;
import org.firstinspires.ftc.teamcode.library.robot.RobotConstants;

import java.util.Objects;
import java.util.function.DoubleUnaryOperator;

public class Shooter extends Mechanism {
    public static class SHOOTER_PARAMS implements Params<SHOOTER_PARAMS> {
        public String HARDWAREMAP_NAME = "shooter";
        public Direction MOTOR_DIRECTION = Direction.REVERSE;
        public DoubleCompositeConstants CONTROLLER_CONSTANTS = null;
        public Boolean usingVelocityController = Boolean.FALSE;
        public DoubleUnaryOperator inpsToRPM = DoubleUnaryOperator.identity();
        public SHOOTER_PARAMS() {}
        private SHOOTER_PARAMS(SHOOTER_PARAMS other) {
            this.HARDWAREMAP_NAME = other.HARDWAREMAP_NAME;
            this.MOTOR_DIRECTION = other.MOTOR_DIRECTION;
            this.CONTROLLER_CONSTANTS = other.CONTROLLER_CONSTANTS;
            this.inpsToRPM = other.inpsToRPM;
            this.usingVelocityController = other.usingVelocityController;
        }

        public SHOOTER_PARAMS hardwareMapName(String name) {
            HARDWAREMAP_NAME = name;
            return this;
        }

        public SHOOTER_PARAMS motorDirection(Direction dir) {
            MOTOR_DIRECTION = dir;
            return this;
        }

        public SHOOTER_PARAMS controllerConsts(DoubleBangBangConstants bangBangConstants, DoublePIDFCoeffs pidfCoeffs, double switchs) {
            CONTROLLER_CONSTANTS = new DoubleCompositeConstants(
                    new DoubleBangBangController(bangBangConstants),
                    new DoublePIDFController(pidfCoeffs),
                    switchs
            );
            usingVelocityController = Boolean.TRUE;
            return this;
        }
        public SHOOTER_PARAMS inpsToRPM(DoubleUnaryOperator u) {
            inpsToRPM = u;
            return this;
        }

        @Override
        public SHOOTER_PARAMS copy() {
            return new SHOOTER_PARAMS(this);
        }

        @Override
        public SHOOTER_PARAMS updateFromBot(RobotConstants bot) {
            return null;
        }
    }
    public enum ShooterMode {
        CRUISING,
        SPINNING_UP,
        IDLE
    }
    Motor flywheel;
    ShotSolver solver;
    ShooterMode mode = ShooterMode.IDLE;
    SHOOTER_PARAMS shooterParams;
    public Shooter(HardwareMap map, RobotConstants bot) {
        super(map, bot);
        solver = new ShotSolver();
        shooterParams = bot.getMechanismParams(Shooter.class, new SHOOTER_PARAMS());
        flywheel = new Motor(hardwareMap, shooterParams.HARDWAREMAP_NAME).Float().setDirection(shooterParams.MOTOR_DIRECTION);
        flywheel.setMode(true);
        if (shooterParams.usingVelocityController) flywheel.setVelocityController(new DoubleCompositeController(shooterParams.CONTROLLER_CONSTANTS));
    }

    @Override
    public void stop() {
        setMode(ShooterMode.IDLE);
    }

    @Override
    public void periodic(State robotState) {
        updateFlywheel(robotState);
    }

    public void updateFlywheel(State robotState) {
        ShotSolver.ShotContext.Builder context = ShotSolver.ShotContext.Builder.create()
                .pose(robotState.getRobot().getPoseCompensatedForVelocity())
                .state(robotState.getRobot().get(FieldUpdates.class).hiveState);
        switch (mode) {
            case CRUISING:
                flywheel.setVelocity(computeVelocity(context.c));
            case SPINNING_UP:
                /*
                double computedVelocity = computeVelocity(context.c);
                // this is a linear function: 0 maps to 6000, target velocity maps to itself
                // todo replace with a quadratic prob
                double m = -6000/computedVelocity + 1;
                flywheel.setVelocity(m*flywheel.getVelocity() + 6000);*/
                flywheel.setVelocity(computeVelocity(context.c));
                if (Math.abs(flywheel.getVelocity()-flywheel.getTargetVelocity()) <= 20) { // TODO ADD THIS TOLERANCE CONSTANT TO PARAMS
                    setMode(ShooterMode.CRUISING);
                }
            case IDLE:
                //flywheel.setVelocity(0);
                // do nothing hooray!
        }
    }

    public void setMode(ShooterMode mode) {
        this.mode = mode;
        if (mode == ShooterMode.IDLE) {
            flywheel.setVelocity(0); // just for when called stop()
        }
    }

    public double computeVelocity(ShotSolver.ShotContext shotContext) {
        return shooterParams.inpsToRPM.applyAsDouble(solver.calculateShot(shotContext).getB());
    }
}
