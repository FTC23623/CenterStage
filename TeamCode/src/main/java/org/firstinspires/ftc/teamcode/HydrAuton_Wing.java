package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.teamcode.types.HydraArmMovements;

public class HydrAuton_Wing extends HydrAuton {
    public boolean RunAuton() {
        if (autonState < 100) {
            // These states handle driving to the correct spike
            if (!AutonDriveToSpike(setTrueForRiggingOnRight)) {
                BadState();
                return true;
            }
        }
        else if (autonState < 200) {
            // These 100 level states handle dropping the pixel on the spike
            // this is the same for all autons
            if (!PixelDrop(false)) {
                BadState();
                return true;
            }
        }
        else if (autonState < 300) {
            // These 200 level states handle driving to the backdrop
            if (!AutonDriveToBackdropFromWing(setTrueForRed, false)) {
                BadState();
                return true;
            }
        }
        else if (autonState < 400) {
            // These 300 level states handle scoring forwards at the backdrop
            // this is the same for two autons
            if (!ScoreFront()) {
                BadState();
                return true;
            }
        }
        else if (autonState < 500) {
            // These 400 level states handle returning the arm home
            // this is the same for all autons
            ArmToHome();
        }
        else if (autonState == 500) {
            // this auton is complete
            return true;
        }
        else {
            BadState();
            return true;
        }
        if (autonState >= 200) {
            // now that we are driving towards the backdrop, start looking for the AprilTag
            // this just prints to telemetry for now
            ObjDet.FindAprilTag(ObjLoc);
        }
        return false;
    }
    
    /**
     * Drive the robot to the backdrop from the wing based on which spike we delivered at [ObjLoc]
     * This function is used for both wing or audience side auton op modes
     * @param flipWhenRed is set to true if we are running a red team auton
     * @return false iff there is a problem
     */
    protected boolean AutonDriveToBackdropFromWing(boolean flipWhenRed, boolean parkOnly) {
        // multiply strafes and rotates by -1 based on the starting orientation
        int flip = 1;
        if (flipWhenRed) {
            flip = -1;
        }
        switch (autonState) {
            case 200:
                switch (ObjLoc) {
                    // jump to the correct state based on the detected location
                    case ObjLocBlueLeftSpike:
                    case ObjLocRedRightSpike:
                        autonState = 230;
                        break;
                    case ObjLocBlueRightSpike:
                    case ObjLocRedLeftSpike:
                        autonState = 210;
                        break;
                    case ObjLocBlueCenterSpike:
                    case ObjLocRedCenterSpike:
                        autonState = 220;
                        break;
                    default:
                        return false;
                }
                break;
            ////////////////////////////////////////////////////////////////////////////////////////
            case 210:
                // BLUE RIGHT
                // RED LEFT
                Drive.Start(-2, 0, 0, mHeading);
                autonState += 1;
                break;
            case 211:
                // BLUE RIGHT
                // RED LEFT
                if (!Drive.Busy()) {
                    Drive.Start(0, -12 * flip, 0, mHeading);
                    autonState += 1;
                }
                break;
            case 212:
                // BLUE RIGHT
                // RED LEFT
                if (!Drive.Busy()) {
                    Drive.Start(35, 0, 0, mHeading);
                    autonState += 1;
                }
                break;
            case 213:
                // BLUE RIGHT
                // RED LEFT
                if (!Drive.Busy()) {
                    mHeading = 90 * flip;
                    Drive.Start(0, 0, -20 * flip, mHeading);
                    autonState += 1;
                }
                break;
            case 214:
                // BLUE RIGHT
                // RED LEFT
                if (!Drive.Busy() && opModeTimer.milliseconds() >= mWaitTimeAtRigging) {
                    if (parkOnly) {
                        Drive.Start(86, 0, 0, mHeading);
                        autonState = 299;
                    }
                    else {
                        Drive.Start(80, 0, 0, mHeading);
                        autonState += 1;
                    }
                }
                break;
            case 215:
                // BLUE RIGHT
                // RED LEFT
                if (!Drive.Busy()) {
                    Drive.Start(0, -22 * flip, 0, mHeading);
                    Arm.RunAction(HydraArmMovements.ArmMoveToFront);
                    autonState = 299;
                }
                break;
            ////////////////////////////////////////////////////////////////////////////////////////
            case 220:
                // CENTER
                if (!Drive.Busy()) {
                    Drive.Start(0, 18 * flip, 0, mHeading);
                    autonState += 1;
                }
                break;
            case 221:
                // CENTER
                if (!Drive.Busy() && opModeTimer.milliseconds() >= mWaitTimeAtRigging) {
                    if (parkOnly) {
                        Drive.Start(103, 0, 0, mHeading);
                        autonState = 299;
                    }
                    else {
                        Drive.Start(94, 0, 0, mHeading);
                        autonState += 1;
                    }
                }
                break;
            case 222:
                // CENTER
                if (!Drive.Busy()) {
                    Drive.Start(0, -27 * flip, 0, mHeading);
                    Arm.RunAction(HydraArmMovements.ArmMoveToFront);
                    autonState = 299;
                }
                break;
            ////////////////////////////////////////////////////////////////////////////////////////
            case 230:
                // BLUE LEFT
                // RED RIGHT
                if (!Drive.Busy()) {
                    Drive.Start(0, 22 * flip, 0, mHeading);
                    autonState += 1;
                }
                break;
            case 231:
                // BLUE LEFT
                // RED RIGHT
                if (!Drive.Busy() && opModeTimer.milliseconds() >= (mWaitTimeAtRigging + 2)) {
                    if (parkOnly) {
                        Drive.Start(93, 0, 0, mHeading);
                        autonState = 299;
                    }
                    else {
                        Drive.Start(79, 0, 0, mHeading);
                        autonState += 1;
                    }
                }
                break;
            case 232:
                // BLUE LEFT
                // RED RIGHT
                if (!Drive.Busy()) {
                    Drive.Start(0, -30 * flip, 0, mHeading);
                    Arm.RunAction(HydraArmMovements.ArmMoveToFront);
                    autonState = 299;
                }
                break;
            ////////////////////////////////////////////////////////////////////////////////////////
            case 299:
                boolean drivecomplete = !Drive.Busy();
                boolean armcomplete = true;
                if (!parkOnly) {
                    armcomplete = Arm.RunAction(HydraArmMovements.ArmMoveToFront);
                }
                if (drivecomplete && armcomplete) {
                    autonState = 300;
                }
                break;
            default:
                // something bad happened
                return false;
        }
        return true;
    }
}
