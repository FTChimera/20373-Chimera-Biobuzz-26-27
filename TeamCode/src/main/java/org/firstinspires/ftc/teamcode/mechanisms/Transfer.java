package org.firstinspires.ftc.teamcode.mechanisms;


import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.library.game.Ball;
import org.firstinspires.ftc.teamcode.library.game.State;
import org.firstinspires.ftc.teamcode.library.hardware.actuators.Motor;
import org.firstinspires.ftc.teamcode.library.robot.RobotConstants;

// handles both transfer motor and color sensor. ACTUALLY CALLS THE SHOOTER HERE
public class Transfer extends Mechanism {
    public static class TRANSFER_PARAMS implements Params<TRANSFER_PARAMS> {
        @Override
        public TRANSFER_PARAMS copy() {
            return this;
        }

        @Override
        public TRANSFER_PARAMS updateFromBot(RobotConstants bot) {
            return this;
        }
    }

    public enum TransferInput {SMART_SHOOT, OVERRIDE_POLLEN}
    public enum TransferCmd {
            KICK_POLLEN_ONE,
            KICK_NECTAR_ONE,
            SLOW_TO_SHOOTER_CONSTANT,
            SLOW_TO_SHOOTER_SMART, // this looks at difference between needed and actual shooter current values, then calculates the transfer power real time based on that
            FAST, // like all balls same color but still want to make it in the goal
            RAPID, // just want the balls out, end of auto smth like that
            STOP;
            public static TransferCmd kickFrom(Ball ball) {
                return ball==Ball.POLLEN? KICK_POLLEN_ONE : KICK_NECTAR_ONE;
            }
    }
    public TRANSFER_PARAMS params;

    public Motor transferMotor;
    // color sensor
    public Transfer(HardwareMap map, RobotConstants bot) {
        super(map, bot);
        params = resolveParams(Transfer.class, new TRANSFER_PARAMS());
        transferMotor = new Motor(hardwareMap, null); // params
    }

    @Override
    public void stop() {

    }

    @Override
    public void periodic(State robotState) {

    }

    // meant to be called in periodic
    // state int represents the internal state from gp controls
    public void updateState(TransferInput state) {
        if (state == TransferInput.SMART_SHOOT) {
            Ball ball = getCurrentDetectedBallInShooter();
            updateShooter(ball);
            if (shooterUpToSpeed(ball)) {
                runTransferCommand(TransferCmd.kickFrom(ball));
            }
        }
    }

    public void updateShooter(Ball ball) {

    }
    public boolean shooterUpToSpeed(Ball ball) {
        return false;
    }

    public void runTransferCommand(TransferCmd command) {

    }

    public Ball getCurrentDetectedBallInShooter() {
        return null;
    }
}
