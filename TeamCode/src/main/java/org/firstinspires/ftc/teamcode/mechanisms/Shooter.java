package org.firstinspires.ftc.teamcode.mechanisms;

import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.library.ShotSolver;
import org.firstinspires.ftc.teamcode.library.game.State;
import org.firstinspires.ftc.teamcode.library.hardware.actuators.Direction;
import org.firstinspires.ftc.teamcode.library.hardware.actuators.Motor;
import org.firstinspires.ftc.teamcode.library.robot.RobotConstants;

import java.util.Objects;

public class Shooter extends Mechanism {
    public static class SHOOTER_PARAMS implements Params<SHOOTER_PARAMS> {
        public String HARDWAREMAP_NAME = "shooter";
        public Direction MOTOR_DIRECTION = Direction.REVERSE;
        public SHOOTER_PARAMS() {}
        private SHOOTER_PARAMS(SHOOTER_PARAMS other) {
            this.HARDWAREMAP_NAME = other.HARDWAREMAP_NAME;
        }

        public SHOOTER_PARAMS hardwareMapName(String name) {
            HARDWAREMAP_NAME = name;
            return this;
        }

        public SHOOTER_PARAMS motorDirection(Direction dir) {
            MOTOR_DIRECTION = dir;
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
    }

    @Override
    public void stop() {
        setMode(ShooterMode.IDLE);
    }

    @Override
    public void periodic(State robotState) {
        switch (mode) {
            case CRUISING:
                flywheel.setVelocity(computeVelocity());
            case SPINNING_UP:
                double computedVelocity = computeVelocity();
                // this is a linear function: 0 maps to 6000, target velocity maps to itself
                // replace with a quadratic prob
                double m = -6000/computedVelocity + 1;
                flywheel.setVelocity(m*flywheel.getVelocity() + 6000);
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

    public double computeVelocity() {return 0;} // replace with direct call to shotsolver
}
