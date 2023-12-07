package org.firstinspires.ftc.teamcode.subsystems;

public interface HydraImu {
    public boolean Calibrating();
    public void Close();
    public void ResetYaw();
    public double GetYaw();
}
