package frc.robot.actions;

import java.util.ArrayList;
import java.lang.Thread;
import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReentrantLock;

public class ActionDeque {
    private static ActionDeque deque;

    private ArrayList<Thread> actions;

    private ActionDeque() {
        this.actions = new ArrayList<Thread>();
    }

    public static ActionDeque getInstance() {
        if (deque == null) {
            deque = new ActionDeque();
        }

        return deque;
    }

    public void run() {
        for (int i = 0; i < this.actions.size(); i++) {
            ActionThread t = (ActionThread) this.actions.get(i);
            ReentrantLock lock = t.getLock();

            if (!t.isAlive()) {
                t.start();
            }

            if (t.willCancel()) {
                this.actions.remove(i);

                i--;
            }

            if (lock != null) {
                if (lock.isLocked()) {
                    for (Thread thread : this.actions) {
                        ActionThread ct = (ActionThread) thread;

                        if (ct.getLock().equals(lock) && ct.willLoop()) {
                            ct.end();
                        }
                    }
                }
            }

            // Wait 20ms so that the thread started this loop
            // has time to start before the thread next loop starts
            try {
                Thread.sleep(20);
            }
            catch (InterruptedException ie) {}
        }
    }

    public void pushFront(Thread action) {
        if (action instanceof ActionThread) {
            this.actions.add(0, action);
        }
    }

    public void pushBack(Thread action) {
        if (action instanceof ActionThread) {
            this.actions.add(action);
        }
    }

    public Thread popFront() {
        return this.actions.remove(0);
    } 

    public Thread popBack() {
        return this.actions.remove(this.actions.size() - 1);
    }

    public int getSize() {
        return this.actions.size();
    }
}
