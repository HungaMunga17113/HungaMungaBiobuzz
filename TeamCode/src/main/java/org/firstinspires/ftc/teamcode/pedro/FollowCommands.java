package org.firstinspires.ftc.teamcode.pedro;

import com.pedropathing.follower.Follower;
import com.pedropathing.ivy.CommandBuilder;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;

/**
 * Ivy commands for the Pedro 3.0 {@link Follower}. Ivy 1.0.0's own PedroCommands
 * targets the Pedro 2.x PathChain API, so these replace it.
 */
public final class FollowCommands {
    private FollowCommands() {}

    /** Follows the path and finishes when the follower is no longer busy. */
    public static CommandBuilder follow(Follower follower, Path path) {
        return new CommandBuilder()
                .requiring(follower)
                .setStart(() -> follower.follow(path))
                .setDone(() -> !follower.isBusy());
    }

    /** Holds the given pose until the command is interrupted or cancelled. */
    public static CommandBuilder hold(Follower follower, Pose pose) {
        return new CommandBuilder()
                .requiring(follower)
                .setStart(() -> follower.hold(pose))
                .setDone(() -> false);
    }
}
