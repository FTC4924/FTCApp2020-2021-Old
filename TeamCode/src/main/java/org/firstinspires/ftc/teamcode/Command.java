package org.firstinspires.ftc.teamcode;

/**
 * Created by Brendan Clark on 09/24/2020 at 11:54 AM.
 */

public class Command {

    public enum CommandType {

        MOVE

    }
    public CommandType commandType;

    public Command(CommandType commandType) {

        this.commandType = commandType;

    }
}
