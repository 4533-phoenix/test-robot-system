package frc.robot.subsystems;

import java.util.concurrent.locks.ReentrantLock;

public class Subsystem {
    private final ReentrantLock threadLock = new ReentrantLock();

    public final ReentrantLock getThreadLock() {
        return this.threadLock;
    }
}
