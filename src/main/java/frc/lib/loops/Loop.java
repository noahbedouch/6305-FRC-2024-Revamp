package frc.lib.loops;

public interface Loop {
    void onStart(double timestamp);

    void onLoop(double timestamp);

    void onStop(double timestamp);
}
