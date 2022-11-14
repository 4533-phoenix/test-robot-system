package frc.robot.actions;

import java.util.concurrent.Callable;
import java.util.concurrent.locks.ReentrantLock;

public final class FunctionalAction extends Action {
    private Runnable startMethod;
    private Callable<Boolean> conditionMethod;
    private Runnable endMethod;

    public FunctionalAction(Runnable startMethod, Runnable runMethod, Callable<Boolean> conditionMethod, Runnable endMethod, boolean willCancel, boolean isEvent, ReentrantLock threadLock) {
        super(runMethod, willCancel, isEvent, threadLock);

        this.startMethod = startMethod;
        this.conditionMethod = conditionMethod;
        this.endMethod = endMethod;
    }

    @Override
    public void run() {
        if (this.willThreadRun()) {
            try {
                this.getThreadLock().lock();
            }
            finally {}
        }

        if (this.isEvent()) {
            if (this.willRun()) {
                this.startMethod.run();

                try {
                    while (!conditionMethod.call()) {
                        super.run();
                    }
                }
                catch (Exception e) {}

                this.endMethod.run();
            }
        }
        else {
            this.startMethod.run();

            try {
                while (!conditionMethod.call()) {
                    super.run();
                }
            }
            catch (Exception e) {}

            this.endMethod.run();
        }
        
        if (this.willThreadRun()) {
            try {
                this.getThreadLock().unlock();
            }
            finally {}
        }
    }
}
