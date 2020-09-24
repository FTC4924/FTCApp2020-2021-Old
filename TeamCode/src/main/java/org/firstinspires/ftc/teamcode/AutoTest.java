package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import org.firstinspires.ftc.teamcode.Command.CommandType;

import java.util.ArrayList;
import java.util.Arrays;

/**
 * Created by Brendan Clark on 09/24/2020 at 12:01 PM.
 */

@Autonomous(name = "AutoTest")
public class AutoTest extends AutoBase {

    public ArrayList<Command> getCommands() {
        return new ArrayList<>(
                Arrays.asList(
                        new Command(CommandType.MOVE)
                )
        );
    }

}
