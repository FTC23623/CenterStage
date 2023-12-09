package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "BlueParkWing", preselectTeleOp = "HyDrive")
public class HydrAuton_BlueParkWing extends HydrAuton_WingParkOnly {
    public HydrAuton_BlueParkWing() {
        setTrueForRed = false;
        setTrueForRiggingOnRight = false;
        modelFilename = "Blue_Prop.tflite";
        mOpModeName = "Blue-Wing-Park";
        mWaitTimeAtRigging = 18000;
    }
}
