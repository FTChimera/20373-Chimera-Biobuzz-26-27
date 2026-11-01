package org.firstinspires.ftc.teamcode.planners.commands;

import com.pedropathing.ivy.Command;
import com.pedropathing.paths.Path;

import org.firstinspires.ftc.teamcode.mechanisms.Robot;
import org.firstinspires.ftc.teamcode.pedro.Constants;
import org.firstinspires.ftc.teamcode.pedro.Follower;

import java.util.Objects;
import java.util.concurrent.atomic.AtomicReference;
import java.util.function.Supplier;

public class DrivetrainCommands {
    public static Command follow(Follower follower, Path path) {
        return follow(follower, () -> path);
    }

    public static Command follow(Follower follower, Supplier<Path> pathSupplier) {
        return Command.build()
                .requiring(follower)
                .setStart(() -> follower.follow(Objects.requireNonNull(pathSupplier.get(), "Path supplier returned null")))
                .setDone(follower::atParametricEnd);
    }

    public static Command setMaxPowerOnFly(Robot robot, double tValue, double newPower) {
        AtomicReference<Double> previousPower = new AtomicReference<>();
        return Command.build()
                .setStart(() -> previousPower.set(Constants.FOLLOWER_MAX_PATH_SPEED.get()))
                .setExecute(() -> {
                    if (robot.follower.completion() >= tValue) Constants.FOLLOWER_MAX_PATH_SPEED.set(newPower);
                })
                .setDone(robot.follower::atParametricEnd)
                .setEnd((endCondition) -> {
                    Double priorSpeed = previousPower.get();
                    if (priorSpeed != null) {
                        Constants.FOLLOWER_MAX_PATH_SPEED.set(priorSpeed);
                    }
                });
    }
}
