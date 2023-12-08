package org.firstinspires.ftc.teamcode.subsystems;

import com.kauailabs.navx.ftc.AHRS;
import com.qualcomm.hardware.kauailabs.NavxMicroNavigationSensor;

import org.firstinspires.ftc.teamcode.objects.HydraOpMode;

public class HydraImu_navx implements HydraImu {
    protected AHRS mNavx;
    private final byte NAVX_DEVICE_UPDATE_RATE_HZ = 50;
    public HydraImu_navx(HydraOpMode opMode) {
        mNavx = AHRS.getInstance(opMode.mHardwareMap.get(NavxMicroNavigationSensor.class,
                "navx"), AHRS.DeviceDataType.kProcessedData, NAVX_DEVICE_UPDATE_RATE_HZ);
    }

    public boolean Connected() {
        return mNavx.isConnected();
    }
    public void ResetYaw() {
        mNavx.zeroYaw();
    }

    public double GetYaw() {
        return mNavx.getYaw() * -1;
    }

    public boolean Calibrating() {
        return mNavx.isCalibrating();
    }

    public void Close() {
        mNavx.close();
    }
}
