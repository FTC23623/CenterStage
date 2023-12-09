package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "BlueBackstage", preselectTeleOp = "HyDrive")
public class HydrAuton_BlueBackstage extends HydrAuton_Backstage {
    public HydrAuton_BlueBackstage() {
        setTrueForRed = false;
        setTrueForRiggingOnRight = true;
        modelFilename = "Blue_Prop.tflite";
        mOpModeName = "Blue-Backstage";
    }
}
