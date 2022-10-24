package frc.robot.actions;

import com.fasterxml.jackson.databind.util.LookupCache;

import frc.robot.actions.ActionThread;
import frc.robot.actions.Listener;
import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReentrantLock;

public class EventThread extends ActionThread implements Listener {
    private boolean willRun = false;

    public EventThread(Runnable target, boolean keepRunning, boolean willCancel, ReentrantLock lock) {
        super(target, keepRunning, willCancel, lock);
    }

    @Override
    public void start() {
        if (this.willRun) {
            super.start();
        }
    }

    @Override
    public void respond() {
        this.willRun = true;
    }
}
