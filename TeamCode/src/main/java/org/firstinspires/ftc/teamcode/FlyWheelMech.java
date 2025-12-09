package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.mechanisms.shootMotorInit;

@TeleOp
public class FlyWheelMech extends OpMode {

    shootMotorInit motor = new shootMotorInit();

    @Override
    public void init() {
        motor.init(hardwareMap);
    }

    @Override
    public void loop() {
     motor.setMotorSpeed(1);
     telemetry.addData("Motor Revs:", motor.getMotorRevs());
    }
}
