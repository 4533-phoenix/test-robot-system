package frc.robot.actions;

import java.util.concurrent.Callable;
import java.util.concurrent.locks.ReentrantLock;

public class FunctionThread extends ActionThread {
    private Runnable startMethod;
    private Callable<Boolean> conditionMethod;
    private Runnable endMethod;

    public FunctionThread(Runnable startMethod, Runnable runMethod, Callable<Boolean> conditionMethod, Runnable endMethod, boolean willCancel, boolean isEvent, ReentrantLock lock) {
        super(runMethod, false, willCancel, isEvent, lock);

        this.startMethod = startMethod;
        this.conditionMethod = conditionMethod;
        this.endMethod = endMethod;
    }

    @Override
    public void run() {
        this.lock.lock();

        this.startMethod.run();

        try {
            while (!conditionMethod.call()) {
                super.run();
            }
        }
        catch (Exception e) {}

        this.endMethod.run();

        this.lock.unlock();
    }
}
