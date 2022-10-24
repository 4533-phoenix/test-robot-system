package frc.robot.actions;

import java.lang.Thread;
import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReentrantLock;

public class ActionThread extends Thread {
    private boolean isRunning = false;

    private boolean willLoop;

    private boolean willCancel;

    private ReentrantLock lock;

    public ActionThread(Runnable target, boolean willLoop, boolean willCancel, ReentrantLock lock) {
        super(target);

        this.willLoop = willLoop;

        this.willCancel = willCancel;

        this.lock = lock;
    }

    @Override
    public void run() {
        this.isRunning = true;

        this.lock.lock();

        if (this.willLoop) {
            while (this.isRunning) {
                super.run();
            }
        }
        else {
            super.run();
        }

        this.lock.unlock();

        this.isRunning = false;
    }

    public void end() {
        this.isRunning = false;
    }

    public boolean isRunning() {
        return this.isRunning;
    }

    public boolean willLoop() {
        return this.willLoop;
    }

    public boolean willCancel() {
        return this.willCancel;
    }

    public ReentrantLock getLock() {
        return this.lock;
    }
}
