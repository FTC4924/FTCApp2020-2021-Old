package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import java.util.ArrayList;

/**
 * Created by Brendan Clark on 09/24/2020 at 11:51 AM.
 */

public abstract class AutoBase extends OpMode {

    ArrayList<org.firstinspires.ftc.teamcode.Command> commands;
    org.firstinspires.ftc.teamcode.Command currentCommand;
    int commandIndex;

    public void init() {

        commands = getCommands();
        currentCommand = commands.get(0);
        commandIndex = 0;

        resetStartTime();

    }

    public void loop() {

        switch (currentCommand.commandType) {

            case MOVE:

                break;

        }

    }

    public void startNextCommand() {
        if (commandIndex < commands.size()) {

            commandIndex ++;
            currentCommand = commands.get(commandIndex);

            resetStartTime();

        }
    }

    public abstract ArrayList<Command> getCommands();

}
