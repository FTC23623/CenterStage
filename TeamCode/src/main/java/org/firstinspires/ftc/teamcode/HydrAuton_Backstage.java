package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.teamcode.types.HydraArmMovements;
import org.firstinspires.ftc.teamcode.types.HydraObjectLocations;

public class HydrAuton_Backstage extends HydrAuton {
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
            if (!AutonDriveToBackdropFromBackstage(setTrueForRed)) {
                BadState();
                return true;
            }
        }
        else if (autonState < 400) {
            // These 300 level states handle scoring backwards at the backdrop
            // this is the same for two autons
            switch (ObjLoc) {
                case ObjLocRedLeftSpike:
                case ObjLocRedCenterSpike:
                case ObjLocBlueCenterSpike:
                case ObjLocBlueRightSpike:
                default:
                    if (!ScoreBack()) {
                        BadState();
                        autonState = 400;
                    }
                    break;
                case ObjLocRedRightSpike:
                case ObjLocBlueLeftSpike:
                    if (!ScoreFront()) {
                        BadState();
                        autonState = 400;
                    }
                    break;
            }
        }
        else if (autonState < 500) {
            // to speed up the auton, start the arm motion and then jump right to the drive away
            Arm.RunAction(HydraArmMovements.ArmMoveToHome);
            autonState = 500;
        }
        else if (autonState < 600) {
            switch (autonState) {
                case 500:
                    Arm.RunAction(HydraArmMovements.ArmMoveToHome);
                    if (!Drive.Busy()) {
                        int flip = 1;
                        if (setTrueForRed) {
                            flip = -1;
                        }
                        int dist;
                        switch (ObjLoc) {
                            case ObjLocRedLeftSpike:
                                dist = 34;
                                break;
                            case ObjLocBlueLeftSpike:
                                dist = -23;
                                break;
                            case ObjLocBlueCenterSpike:
                            case ObjLocRedCenterSpike:
                                dist = 30;
                                break;
                            case ObjLocRedRightSpike:
                            default:
                                dist = -23;
                                break;
                            case ObjLocBlueRightSpike:
                                dist = 34;
                                break;
                        }
                        Drive.Start(0, dist * flip, 0, mHeading);
                        autonState += 1;
                    }
                    break;
                case 501:
                    Arm.RunAction(HydraArmMovements.ArmMoveToHome);
                    if (!Drive.Busy()) {
                        double drive = -14;
                        switch (ObjLoc) {
                            case ObjLocBlueLeftSpike:
                            case ObjLocRedRightSpike:
                                drive = 14;
                                break;
                        }
                        Drive.Start(drive, 0, 0, mHeading);
                        autonState += 1;
                    }
                    break;
                case 502:
                    boolean armComplete = Arm.RunAction(HydraArmMovements.ArmMoveToHome);
                    boolean driveComplete = !Drive.Busy();
                    if (armComplete && driveComplete) {
                        autonState = 600;
                    }
                    break;
                default:
                    BadState();
                    return false;
            }
        }
        else if (autonState == 600) {
            // this auton is complete
            return true;
        }
        else {
            BadState();
            return true;
        }
        return false;
    }

    /**
     * Drive the robot to the backdrop from the backstage based on which spike we delivered at [ObjLoc]
     * This function is used for both backstage side auton op modes
     * @param flipForRed is set to true if we are running a red team auton
     * @return false iff there is a problem
     */
    protected boolean AutonDriveToBackdropFromBackstage(boolean flipForRed) {
        // multiply strafes and rotates by -1 based on the starting orientation
        int flip = 1;
        if (flipForRed) {
            flip = -1;
        }
        switch (autonState) {
            case 200:
                // jump to the correct state based on which spike we are at
                switch (ObjLoc) {
                    case ObjLocBlueLeftSpike:
                    case ObjLocRedRightSpike:
                        autonState = 210;
                        break;
                    case ObjLocBlueCenterSpike:
                    case ObjLocRedCenterSpike:
                        autonState = 220;
                        break;
                    case ObjLocBlueRightSpike:
                    case ObjLocRedLeftSpike:
                        autonState = 230;
                        break;
                    default:
                        return false;
                }
                break;
            ////////////////////////////////////////////////////////////////////////////////////////
            case 210:
                // BLUE LEFT SPIKE
                // RED RIGHT SPIKE
                if (!Drive.Busy()) {
                    Drive.Start(-4, 0, 0, mHeading);
                    autonState += 1;
                }
                break;
            case 211:
                // BLUE LEFT SPIKE
                // RED RIGHT SPIKE
                if (!Drive.Busy()) {
                    mHeading = 90 * flip;
                    Drive.Start(0, 0, -20 * flip, mHeading);
                    autonState += 1;
                }
                break;
            case 212:
                // BLUE LEFT SPIKE
                // RED RIGHT SPIKE
                if (!Drive.Busy()) {
                    Drive.Start(20, 9 * flip, 0, mHeading);
                    Arm.RunAction(HydraArmMovements.ArmMoveToFront);
                    autonState = 298;
                }
                break;
            ////////////////////////////////////////////////////////////////////////////////////////
            case 220:
                // CENTER SPIKE
                if (!Drive.Busy()) {
                    Drive.Start(0, 10 * flip, 0, mHeading);
                    autonState += 1;
                }
                break;
            case 221:
                // CENTER SPIKE
                if (!Drive.Busy()) {
                    Drive.Start(-14, 0, 0, mHeading);
                    Arm.RunAction(HydraArmMovements.ArmMoveToBack);
                    autonState = 299;
                }
                break;
            ////////////////////////////////////////////////////////////////////////////////////////
            case 230:
                // BLUE RIGHT SPIKE
                // RED LEFT SPIKE
                if (!Drive.Busy()) {
                    Drive.Start(-28, 0, 0, mHeading);
                    Arm.RunAction(HydraArmMovements.ArmMoveToBack);
                    autonState = 299;
                }
                break;
            ///////////////////////////////////////////////////////////////////////////////////////
            case 298: {
                boolean drivecomplete = !Drive.Busy();
                boolean armcomplete = Arm.RunAction(HydraArmMovements.ArmMoveToFront);
                if (drivecomplete && armcomplete) {
                    autonState = 300;
                }
            }
                break;
            ///////////////////////////////////////////////////////////////////////////////////////
            case 299: {
                boolean drivecomplete = !Drive.Busy();
                boolean armcomplete = Arm.RunAction(HydraArmMovements.ArmMoveToBack);
                if (drivecomplete && armcomplete) {
                    autonState = 300;
                }
            }
                break;
            default:
                // something bad happened
                return false;
        }
        return true;
    }
}
