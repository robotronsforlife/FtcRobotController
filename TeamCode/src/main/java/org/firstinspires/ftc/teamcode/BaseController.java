package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.robotcore.external.Telemetry;

public class BaseController implements Runnable{

    public boolean right_bumper;
    public boolean left_bumper;

    private boolean isRunning;
    public Servo blocker;
    public Telemetry telemetry;

    public BaseController(Servo blocker){
        this.blocker = blocker;
    }

    /**
     * Stops the position update thread
     */
    public void stop(){ isRunning = false; }

    public String Status;

    @Override
    public void run() {

        while(isRunning) {

            if (right_bumper) {
                blocker.setPosition(1);
                Status = "Right Bumper";
            }
            else if (left_bumper) {
                blocker.setPosition(-1);
                Status = "Left Bumper";
            }
            else{
                Status = "While loop";
            }

        }
    }
}
