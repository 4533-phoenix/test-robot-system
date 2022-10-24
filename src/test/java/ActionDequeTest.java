package test;

import frc.robot.actions.ActionDeque;
import frc.robot.actions.ActionThread;
import java.util.concurrent.locks.ReentrantLock;
import java.util.Date;
import static org.junit.Assert.*;

import java.beans.Transient;

import org.junit.*;

public class ActionDequeTest {
    private int a = 0;
    private int b = 0;
    private int c = 0;
    private int d = 0;

    @Test
    public void runActionDequeWithActionLocks() {
        ActionDeque deque = ActionDeque.getInstance();

        ReentrantLock abLock = new ReentrantLock();
        ReentrantLock cdLock = new ReentrantLock();

        Runnable aRunnable = new Runnable() {
            @Override
            public void run() {
                a += 5;
            }
        };

        Runnable bRunnable = new Runnable() {
            @Override
            public void run() {
                b += a + 10;
            }
        };

        Runnable cRunnable = new Runnable() {
            @Override
            public void run() {
                c += 10;
            }
        };

        Runnable dRunnable = new Runnable() {
            @Override
            public void run() {
                d += c + 20;
            }
        };

        ActionThread aThread = new ActionThread(aRunnable, false, true, abLock);
        ActionThread bThread = new ActionThread(bRunnable, false, true, abLock);
        ActionThread cThread = new ActionThread(cRunnable, false, true, cdLock);
        ActionThread dThread = new ActionThread(dRunnable, false, true, cdLock);

        deque.pushBack(aThread);
        deque.pushBack(bThread);
        deque.pushBack(dThread);
        deque.pushBack(cThread);

        deque.run();

        while (aThread.isAlive() || bThread.isAlive() || cThread.isAlive() || dThread.isAlive()) {}

        assertEquals(5, a, 0);
        assertEquals(15, b, 0);
        assertEquals(10, c, 0);
        assertEquals(20, d, 0);
    }
}
