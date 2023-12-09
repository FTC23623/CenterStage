package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "RedBackstage", preselectTeleOp = "HyDrive")
public class HydrAuton_RedBackstage extends HydrAuton_Backstage {
    public HydrAuton_RedBackstage() {
        setTrueForRed = true;
        setTrueForRiggingOnRight = false;
        modelFilename = "Red_Prop.tflite";
        mOpModeName = "Red-Backstage";
    }
}
