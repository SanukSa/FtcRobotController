package org.firstinspires.ftc.teamcode.mechanisms;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

public class FlyWheelLogic {

    private DcMotor shootMotor;
    private ElapsedTime stateTimer = new ElapsedTime();

    private enum FlyWheelStatus {
        IDLE
    }

}
