package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * Manages selecting a command from the driver station.
 */
public class CommandSelector extends SubsystemBase{


    private final SendableChooser<Command> chooser = new SendableChooser<>();

    private class RunCommand extends Command{
        private Command command;

        /**
         * Ensure only one copy of the run command is ever running for this instance of CommandSelector
         */
        public RunCommand() {
            addRequirements(CommandSelector.this);
        }

        /**
         * Store the currently select command and schedule it to run
         */
        @Override
        public void initialize() {
            command = chooser.getSelected();
            CommandScheduler.getInstance().schedule(command);
        }

        /**
         * Cancel the command when this command ends
         */
        @Override
        public void end(boolean interrupted) {
            CommandScheduler.getInstance().cancel(command);
        }
    }

    /**
     * Adds a command to the list
     * 
     * @param name      name of the command 
     * @param command   command to add
     * @return  Reference of the current object for command chaining
     */
    public CommandSelector addEntry(String name, Command command) {
        chooser.addOption(name, command);
        return this;
    }


    /**
     * Sends the chooser to the dashboard. All commands must be added before this method is called
     * @param name  name of the chooser
     * @return  Reference of the current object for command chaining
     */
    public CommandSelector sendToDashboard(String name) {
        SmartDashboard.putData(name, chooser);
        return this;
    }
    
    /**
     * Gets a command that runs the selected command
     * @return  command that runs the selected command
     */
    public Command runCommandCmd() {
        return new RunCommand();
    }
}
