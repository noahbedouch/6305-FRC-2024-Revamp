package frc.lib.loops;

import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.lib.util.CrashTrackingRunnable;

public class Looper implements ILooper {
    public final double period;

    private boolean running;

    private final Notifier notifier;
    private final List<Loop> loops;
    private final Object taskRunningLock = new Object();
    private double timestamp = 0;
    private double dt = 0;

    private final CrashTrackingRunnable runnable = new CrashTrackingRunnable() {
        @Override
        public void runCrashTracked() {
            synchronized(taskRunningLock) {
                if(running) {
                    double now = Timer.getFPGATimestamp();

                    for(Loop loop : loops) {
                        loop.onLoop(now);
                    }

                    dt = now - timestamp;
                    timestamp = now;
                }
            }
        }
    };
    
    public Looper(double mPeriod) {
        notifier = new Notifier(runnable);
        period = mPeriod;
        running = false;
        loops = new ArrayList<>();
    }

    @Override
    public synchronized void register(Loop loop) {
        synchronized(taskRunningLock) {
            loops.add(loop);
        }
    }

    public synchronized void start() {
        if(!running) {
            System.out.println("Starting Loops");

            synchronized(taskRunningLock) {
                timestamp = Timer.getFPGATimestamp();

                for(Loop loop : loops) {
                    loop.onStart(timestamp);
                }

                running = true;
            }

            notifier.startPeriodic(period);
        }
    }

    public synchronized void stop() {
        if(running) {
            System.out.println("Stopping Loops");
            notifier.stop();

            synchronized(taskRunningLock) {
                running = false;
                timestamp = Timer.getFPGATimestamp();

                for(Loop loop : loops) {
                    System.out.println("Stopping" + loop);
                    loop.onStop(timestamp);
                }
            }
        }
    }

    public void outputToSmartDashboard() {
        SmartDashboard.putNumber("looper_dt", dt);
    }
}
