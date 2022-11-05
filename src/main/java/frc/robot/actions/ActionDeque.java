package frc.robot.actions;

import java.util.ArrayList;
import java.util.concurrent.locks.ReentrantLock;

public class ActionDeque {
    private static ActionDeque deque;

    private ArrayList<ActionThread> actions;

    private ActionDeque() {
        this.actions = new ArrayList<ActionThread>();
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

                while (!lock.isLocked()) {}
            }

            if (t.willCancel()) {
                if (!t.willLoop()) {
                    this.actions.remove(i);

                    i--;
                }
            }

            for (int j = 0; j < this.actions.size(); j++) {
                ActionThread ct = (ActionThread) this.actions.get(j);

                if (!ct.equals(t) && ct.getLock().equals(lock) && ct.willLoop()) {
                    ct.end();

                    if (ct.willCancel()) {
                        this.actions.remove(j);

                        j--;
                        i--;
                    }
                }
            }
        }
    }

    public void pushFront(ActionThread... actions) {
        for (ActionThread action : actions) {
            this.actions.add(0, action);
        }
    }

    public void pushBack(ActionThread... actions) {
        for (ActionThread action : actions) {
            this.actions.add(action);
        }
    }

    public ActionThread popFront() {
        return this.actions.remove(0);
    } 

    public ActionThread popBack() {
        return this.actions.remove(this.actions.size() - 1);
    }

    public int getSize() {
        return this.actions.size();
    }
}
