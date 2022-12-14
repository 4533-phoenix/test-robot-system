package frc.robot.actions;

import java.util.ArrayList;
public final class ActionDeque {
    private static ActionDeque deque;

    private ArrayList<Action> actions;

    private ActionDeque() {
        this.actions = new ArrayList<Action>();
    }

    public static ActionDeque getInstance() {
        if (deque == null) {
            deque = new ActionDeque();
        }

        return deque;
    }

    public void run() {
        for (int i = 0; i < this.actions.size(); i++) {
            Action action = (Action) this.actions.get(i);

            if (action.willThreadRun()) {
                action.start();

                while (!action.getThreadLock().isLocked()) {}
            }
            else {
                action.run();
            }

            if (action.willCancel()) {
                this.actions.remove(i);

                i--;
            }
        }
    }

    public void pushFront(Action... actions) {
        for (Action action : actions) {
            this.actions.add(0, action);
        }
    }

    public void pushBack(Action... actions) {
        for (Action action : actions) {
            this.actions.add(action);
        }
    }

    public Action popFront() {
        return this.actions.remove(0);
    } 

    public Action popBack() {
        return this.actions.remove(this.actions.size() - 1);
    }
}
