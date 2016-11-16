package org.firstinspires.ftc.teamcode;

        import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

        import org.firstinspires.ftc.teamcode.Autonomous_v4;

/**
 * Created by FTCRubies on 11/15/2016.
 */
@Autonomous(name="Blue", group="Linear Autonomous")
public class AutonomousBlue extends Autonomous_v4{
    public AutonomousBlue(){
        this.currentAlliance = AllianceColor.BLUE;
    }
}