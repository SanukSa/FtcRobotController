package org.firstinspires.ftc.teamcode.mechanisms;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class flyWheelMotor {

    private DcMotor flyWheelMotor;

    public void init(HardwareMap hwMap){
        flyWheelMotor = hwMap.get(DcMotor.class, "flyWheelMotor");
        flyWheelMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void setMotorSpeed(double speed){
        flyWheelMotor.setPower(speed); // -1.0 to 1.0
    }

}
