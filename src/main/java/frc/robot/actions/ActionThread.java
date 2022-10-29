package frc.robot.actions;

import java.lang.Thread;
import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReentrantLock;
import frc.robot.actions.Listener;

public class ActionThread extends Thread implements Listener {
    private boolean willLoop;

    private boolean willCancel;

    private boolean isEvent;
    
    private boolean willRun = false;

    protected ReentrantLock lock;

    public ActionThread(Runnable target, boolean willLoop, boolean willCancel, boolean isEvent, ReentrantLock lock) {
        super(target);

        this.willLoop = willLoop;

        this.willCancel = willCancel;

        this.isEvent = isEvent;

        this.lock = lock;
    }

    @Override
    public void start() {
        if (isEvent) {
            if (willRun) {
                super.start();
            }
        }
        else {
            super.start();
        }
    }

    @Override
    public void run() {
        this.lock.lock();

        if (this.willLoop) {
            while (this.willLoop) {
                super.run();
            }

            this.willLoop = true;
        }
        else {
            super.run();
        }

        this.lock.unlock();
    }

    public void end() {
        this.willLoop = false;
    }

    @Override
    public void respond() {
        this.willRun = true;
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
