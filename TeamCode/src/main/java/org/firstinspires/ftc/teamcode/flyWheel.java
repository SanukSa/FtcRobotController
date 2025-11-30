package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.mechanisms.flyWheelMotor;

public class flyWheel extends OpMode {
    flyWheelMotor wheelMotor = new flyWheelMotor();

    @Override
    public void init() {
        wheelMotor.init(hardwareMap);
    }

    @Override
    public void loop() {
        wheelMotor.setMotorSpeed(1);
    }
}
