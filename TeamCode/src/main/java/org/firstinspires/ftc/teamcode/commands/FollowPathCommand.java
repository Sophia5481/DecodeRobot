package org.firstinspires.ftc.teamcode.commands;

import android.util.Log;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.pedroPathing.follower.Follower;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Path;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.PathChain;

import java.util.function.Supplier;

public class FollowPathCommand extends CommandBase {

    private final Follower m_follower;

    private Supplier<Path> pathSupplier;

    private Supplier<PathChain> pathchainSupplier;

    private Path m_path;

    private PathChain m_pathChain;
    private boolean holdEnd = false;

    private long waitUntil;

    public FollowPathCommand(Follower follower, Path path, boolean holdEnd) {
        this.m_follower = follower;
        this.m_path = path;
        this.holdEnd = holdEnd;
        Log.d("FollowPath", "constructor");
    }

    public FollowPathCommand(Follower follower, Path path) {
        this.m_follower = follower;
        this.m_path = path;
    }

    public FollowPathCommand(Follower follower, PathChain pathChain, boolean holdEnd) {
        this.m_follower = follower;
        this.m_pathChain = pathChain;
        this.holdEnd = holdEnd;
    }

    public FollowPathCommand(Follower follower, PathChain pathChain) {
        this.m_follower = follower;
        this.m_pathChain = pathChain;
    }

    public FollowPathCommand(Follower follower, Supplier<Path> pathSupplier) {
        this.m_follower = follower;
        this.pathSupplier = pathSupplier;
    }

    // boolean flag is added to avoid the two methods having the same erasure
    public FollowPathCommand(Follower follower, Supplier<PathChain> pathchainSupplier, boolean flag) {
        this.m_follower = follower;
        this.pathchainSupplier = pathchainSupplier;
    }

    @Override
    public void initialize() {
        if (m_path != null) {
            Log.d("Follower", "initialized");
            m_follower.followPath(m_path, holdEnd);
        } else if (m_pathChain != null) {
            Log.d("Follower", "initialized");
            m_follower.followPath(m_pathChain, holdEnd);
        } else if (pathSupplier != null) {
            m_path = pathSupplier.get();
            if (m_path == null) {
                throw new IllegalStateException("Path supplier returned null");
            }
            Log.d("Follower", "initialized supplied path");
            m_follower.followPath(m_path, holdEnd);
        } else if (pathchainSupplier != null) {
            m_pathChain = pathchainSupplier.get();
            if (m_pathChain == null) {
                throw new IllegalStateException("Path supplier returned null");
            }
            Log.d("Follower", "initialized supplied path");
            m_follower.followPath(m_pathChain, holdEnd);
        }
    }

    @Override
    public void execute() {
        // THIS IS COMMENTED OUT BECAUSE THE AUTO ALREADY CONSTANTLY RUNS THIS IN THE MAIN LOOP
//        m_follower.drawOnDashBoard();
        //m_follower.update();
    }

    @Override
    public boolean isFinished() {
        if (!m_follower.isBusy()) {
            Log.d("Follower", "finished");
        }
        return !m_follower.isBusy();
    }
}
