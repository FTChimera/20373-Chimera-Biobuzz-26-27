package org.firstinspires.ftc.teamcode.opModes.variants;

import androidx.annotation.NonNull;

import com.pedropathing.geometry.Pose;

import org.firstinspires.ftc.teamcode.configs.Blackboard;
import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.library.AllianceColor;
import org.firstinspires.ftc.teamcode.opModes.TeleOp;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "BLUE TELEOP", group = "0: TeleOp")
public class Blue_TeleOp extends TeleOp {
    public Blue_TeleOp() {
        super(AllianceColor.BLUE);
    }

    /**
     * Override this method and place your code here.
     * <p>
     * Please do not catch {@link InterruptedException}s that are thrown in your OpMode
     * unless you are doing it to perform some brief cleanup, in which case you must exit
     * immediately afterward. Once the OpMode has been told to stop, your ability to
     * control hardware will be limited.
     */
    @NonNull
    @Override
    protected Pose getStartingPose() {
        Pose pose = Blackboard.get(Blackboard.Caption.POSE).getValue();
        if (pose != null && (pose.getX() != 0 && pose.getY() != 0 && pose.getHeading() != 0)) {
            return pose; // If it is not null or 0,0,0 (bug: given by follower at end of auto)
        }
        return Constants.BLUE_DEFAULT_START_POSE;
    }
}
