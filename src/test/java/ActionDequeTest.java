import frc.robot.actions.ActionDeque;
import frc.robot.actions.ActionThread;
import frc.robot.actions.FunctionThread;

import java.util.concurrent.Callable;
import java.util.concurrent.locks.ReentrantLock;

import static org.junit.Assert.*;
import org.junit.*;

public class ActionDequeTest {
    private int a = 0;
    private int b = 0;
    private int c = 0;
    private int d = 0;

    @Test
    public void runActionDequeWithActionThreadsWithLocks() {
        ActionDeque deque = ActionDeque.getInstance();

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

        ReentrantLock abLock = new ReentrantLock(true);
        ReentrantLock cdLock = new ReentrantLock(true);

        ActionThread aThread = new ActionThread(aRunnable, false, true, false, abLock);
        ActionThread bThread = new ActionThread(bRunnable, false, true, false, abLock);
        ActionThread cThread = new ActionThread(cRunnable, false, true, false, cdLock);
        ActionThread dThread = new ActionThread(dRunnable, false, true, false, cdLock);

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

    @Test
    public void runActionDequeWithFunctionThreadsWithLocks() {
        ActionDeque deque = ActionDeque.getInstance();

        Callable<Boolean> aConditionalMethod = new Callable<Boolean>() {
            @Override
            public Boolean call() {
                return a >= 15 && b <= -20;
            }
        };

        Runnable aRunMethod = new Runnable() {
            @Override
            public void run() {
                a += 3;
                b -= 5;
            }
        };

        Callable<Boolean> bConditionalMethod = new Callable<Boolean>() {
            @Override
            public Boolean call() {
                return c >= 20 && d <= -15;
            }
        };

        Runnable bRunMethod = new Runnable() {
            @Override
            public void run() {
                c += (a - 5);
                d += (b + 20);
            }
        };

        ReentrantLock abLock = new ReentrantLock(true);

        FunctionThread aThread = new FunctionThread(() -> {}, aRunMethod, aConditionalMethod, () -> {}, true, false, abLock);
        FunctionThread bThread = new FunctionThread(() -> {}, bRunMethod, bConditionalMethod, () -> {}, true, false, abLock);

        deque.pushBack(aThread);
        deque.pushBack(bThread);

        deque.run();

        while (aThread.isAlive() || bThread.isAlive()) {}

        assertEquals(a, 15, 0);
        assertEquals(b, -25, 0);
        assertEquals(c, 30, 0);
        assertEquals(d, -15, 0);
    }
}
