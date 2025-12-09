package org.firstinspires.ftc.teamcode.mechanisms;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class shootMotorInit {
    private DcMotorEx shootMotor;
    private double ticksPerRev;

    public void init(HardwareMap hwMap){
        shootMotor = hwMap.get(DcMotorEx.class, "shootMotor");
        shootMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        ticksPerRev = shootMotor.getMotorType().getTicksPerRev();
    }

    public void setMotorSpeed(double speed){
        shootMotor.setPower(speed);
    }

    public double getMotorRevs(){
        return shootMotor.getCurrentPosition() / ticksPerRev * 20;
    }
}
