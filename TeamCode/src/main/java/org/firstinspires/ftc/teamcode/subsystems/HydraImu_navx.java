package org.firstinspires.ftc.teamcode.subsystems;

import com.kauailabs.navx.ftc.AHRS;
import com.qualcomm.hardware.kauailabs.NavxMicroNavigationSensor;

import org.firstinspires.ftc.teamcode.objects.HydraOpMode;

public class HydraImu_navx implements HydraImu {
    protected boolean mImuCalibrating = true;
    protected AHRS mNavx;
    private final byte NAVX_DEVICE_UPDATE_RATE_HZ = 50;
    public HydraImu_navx(HydraOpMode opMode) {
        mNavx = AHRS.getInstance(opMode.mHardwareMap.get(NavxMicroNavigationSensor.class, "navx"), AHRS.DeviceDataType.kProcessedData, NAVX_DEVICE_UPDATE_RATE_HZ);
    }

    public void ResetYaw() {
        mNavx.zeroYaw();
    }

    public double GetYaw() {
        return mNavx.getYaw();
    }

    public boolean Calibrating() {
        if (mImuCalibrating && !mNavx.isCalibrating()) {
            mImuCalibrating = false;
        }
        return mImuCalibrating;
    }

    public void Close() {
        mNavx.close();
    }
}
