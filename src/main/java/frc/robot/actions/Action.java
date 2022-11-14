package frc.robot.actions;

import java.util.concurrent.locks.ReentrantLock;

public class Action extends Thread implements Listener {
    private Runnable target;

    private boolean willCancel;

    private boolean willThreadRun = false;

    private boolean isEvent;
    
    private boolean willRun = false;

    private ReentrantLock threadLock;

    public Action(Runnable target, boolean willCancel, boolean isEvent, ReentrantLock threadLock) {
        super(target);

        this.target = target;

        this.willCancel = willCancel;

        this.isEvent = isEvent;

        if (this.willCancel || this.isEvent) {
            this.willThreadRun = true;
        }

        this.threadLock = threadLock;
    }

    public void run() {
        if (this.willThreadRun()) {
            try {
                this.getThreadLock().lock();
            }
            finally {}
        }

        if (this.isEvent) {
            if (this.willRun) {
                target.run();
            }
        }
        else {
            target.run();
        }

        if (this.willThreadRun()) {
            try {
                this.getThreadLock().unlock();
            }
            finally {}
        }
    }

    @Override
    public void respondTrue() {
        this.willRun = true;
    }

    @Override
    public void respondFalse() {
        this.willRun = false;
    }

    public boolean willCancel() {
        return this.willCancel;
    }

    public boolean willThreadRun() {
        return this.willThreadRun;
    }

    public boolean isEvent() {
        return this.isEvent;
    }

    public boolean willRun() {
        return this.willRun;
    }

    public ReentrantLock getThreadLock() {
        return this.threadLock;
    }
}
