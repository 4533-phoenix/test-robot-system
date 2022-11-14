package frc.robot.actions;

public class Action implements Listener {
    private Runnable target;

    private boolean willCancel;

    private boolean isEvent;
    
    private boolean willRun = false;

    public Action(Runnable target, boolean willCancel, boolean isEvent) {
        this.target = target;

        this.willCancel = willCancel;

        this.isEvent = isEvent;
    }

    public void run() {
        if (this.isEvent) {
            if (this.willRun) {
                target.run();
            }
        }
        else {
            target.run();
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

    public boolean isEvent() {
        return this.isEvent;
    }

    public boolean willRun() {
        return this.willRun();
    }
}
