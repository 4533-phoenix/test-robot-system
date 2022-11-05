package frc.robot.subsystems;

import java.util.concurrent.locks.ReentrantLock;

import frc.robot.actions.ActionThread;
import frc.robot.logging.Logger;

public class Subsystem {
    private final ReentrantLock threadLock = new ReentrantLock(true);

    private final ActionThread loggingThread = new ActionThread(
        () -> { this.log(); }, 
        true, 
        false, 
        false, 
        Logger.getLoggingLock()
    );

    private final ActionThread periodicThread = new ActionThread(
        () -> { this.periodic(); }, 
        true, 
        false, 
        false, 
        this.threadLock
    );

    public final ReentrantLock getThreadLock() {
        return this.threadLock;
    }

    protected final ActionThread getLoggingThread() {
        return this.loggingThread;
    }

    protected final ActionThread getPeriodicThread() {
        return this.periodicThread;
    }

    public void log() {}

    public void periodic() {}

    public void queryInitialActions() {}
}
