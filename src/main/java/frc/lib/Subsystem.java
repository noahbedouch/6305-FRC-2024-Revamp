package frc.lib;

import frc.lib.loops.ILooper;

public abstract class Subsystem {
    public void writeToLog() {}
    
    public void readPeriodicOutputs() {}

    public void writePeriodicOutputs() {}

    public void registerEnabledLoops(ILooper enabledLooper) {}

    public void zeroSensors() {}

    public abstract void stop();

    public abstract boolean checkSystem();

    public abstract void outputTelemetry(boolean isRobotDisabled);
}
