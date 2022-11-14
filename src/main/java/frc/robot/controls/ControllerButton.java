package frc.robot.controls;

import frc.robot.actions.Listener;

import java.util.ArrayList;

public final class ControllerButton {
    private ArrayList<Listener> eventThreads = new ArrayList<Listener>();

    public void addEventThread(Listener eventThread) {
        this.eventThreads.add(eventThread);
    }

    public void whenPressed() {
        for (Listener eventThread : this.eventThreads) {
            eventThread.respondTrue();
        }
    }

    public void whenReleased() {
        for (Listener eventThread : this.eventThreads) {
            eventThread.respondFalse();
        }
    }
}
