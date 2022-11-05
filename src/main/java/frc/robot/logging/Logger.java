package frc.robot.logging;

import java.util.concurrent.locks.ReentrantLock;

public final class Logger {
    private static final ReentrantLock loggingLock = new ReentrantLock(true);

    public static final ReentrantLock getLoggingLock() {
        return loggingLock;
    }
}
