package org.firstinspires.ftc.teamcode.pedro.commands;

import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.behaviors.BlockedBehavior;
import com.pedropathing.ivy.behaviors.EndCondition;

import java.util.*;
import java.util.stream.Collectors;

/**
 * The Scheduler is responsible for managing and executing commands.
 * It ensures that commands are scheduled and run according to their requirements,
 * priority, conflict behavior, and interrupted behavior.
 *
 * @author Baron Henderson
 * @author Kabir Goyal
 * @author Davis Luxenberg
 */
public class Scheduler {
    public static final Scheduler INSTANCE = new Scheduler();
    private final Deque<Command> runningCommands = new ArrayDeque<>();
    private final Map<Object, Command> activeRequirements = new HashMap<>();
    private final Deque<Command> queuedCommands = new ArrayDeque<>();
    private final Deque<Command> suspendedCommands = new ArrayDeque<>();
    public Scheduler(Command... commands) {
        reset();
        schedule(commands);
    }

    /**
     * Attempts to schedule a command for execution
     *
     * @param command command to be scheduled
     */
    @SuppressWarnings("DataFlowIssue")
    public void schedule(Command command) {
        Set<Object> conflictingRequirements = command.requirements().stream()
                .filter(activeRequirements::containsKey)
                .collect(Collectors.toSet());

        Set<Command> conflictingCommands = conflictingRequirements.stream()
                .map(activeRequirements::get)
                .collect(Collectors.toSet());

        if (conflictingRequirements.isEmpty()) {
            startCommand(command);
            return;
        }

        boolean blocked = conflictingCommands.stream().anyMatch(
                conflictingCommand -> conflictingCommand.priority() > command.priority()
        );

        if (blocked) {
            if (command.blockedBehavior() == BlockedBehavior.QUEUE) {
                queuedCommands.add(command);
            }
            return;
        }

        boolean conflicting = conflictingRequirements.stream().anyMatch(
                requirement -> activeRequirements.get(requirement).priority() == command.priority()
        );

        if (conflicting) {
            switch (command.conflictBehavior()) {
                case OVERRIDE:
                    conflictingCommands.forEach(this::interrupt);
                    startCommand(command);
                    break;
                case QUEUE:
                    queuedCommands.add(command);
                    break;
                case CANCEL:
                    // Command is canceled - do not schedule
                    break;
            }
            return;
        }

        conflictingCommands.forEach(this::interrupt);
        startCommand(command);
    }


    /**
     * Attempts to schedule multiple commands for execution
     *
     * @param commands commands to be scheduled
     */
    @SuppressWarnings("DataFlowIssue")
    public void schedule(Command... commands) {
        Arrays.stream(commands).forEach(this::schedule);
    }

    private void interrupt(Command command) {
        switch (command.interruptedBehavior()) {
            case END:
                command.end(EndCondition.INTERRUPTED);
                removeRequirements(command);
                runningCommands.remove(command);
                break;
            case SUSPEND:
                command.end(EndCondition.SUSPENDED);
                removeRequirements(command);
                runningCommands.remove(command);
                suspendedCommands.add(command);
                break;
        }
    }

    private void removeRequirements(Command command) {
        command.requirements().forEach(activeRequirements::remove);
    }

    private void addRequirements(Command command) {
        command.requirements().forEach(requirement -> activeRequirements.put(requirement, command));
    }

    private void startCommand(Command command) {
        runningCommands.add(command);
        command.start();
        addRequirements(command);
    }

    /**
     * Executes all running commands. This method should be called periodically.
     */
    public void execute() {
        ArrayDeque<Command> toRemove = new ArrayDeque<>(runningCommands.size());
        if (!runningCommands.isEmpty()) {
            Iterator<Command> runningIterator = new ArrayDeque<>(runningCommands).iterator();
            while (runningIterator.hasNext()) {
                Command command = runningIterator.next();
                command.execute();
                if (command.done()) {
                    command.end(EndCondition.NATURALLY);
                    toRemove.add(command);
                    removeRequirements(command);
                    runningIterator.remove();
                }
            }
        }
        runningCommands.removeAll(toRemove);
        toRemove.clear();

        if (!queuedCommands.isEmpty()) {
            Iterator<Command> queuedIterator = new ArrayDeque<>(queuedCommands).iterator();
            while (queuedIterator.hasNext()) {
                Command command = queuedIterator.next();
                boolean canBeScheduled = command.requirements().stream().noneMatch(activeRequirements::containsKey);
                if (canBeScheduled) {
                    toRemove.add(command);
                    queuedIterator.remove();
                    startCommand(command);
                }
            }
        }
        queuedCommands.removeAll(toRemove);
        toRemove.clear();

        if (!suspendedCommands.isEmpty()) {
            Iterator<Command> suspendedIterator = new ArrayDeque<>(suspendedCommands).iterator();
            while (suspendedIterator.hasNext()) {
                Command command = suspendedIterator.next();
                boolean canBeScheduled = command.requirements().stream().noneMatch(activeRequirements::containsKey);
                if (canBeScheduled) {
                    toRemove.add(command);
                    suspendedIterator.remove();
                    runningCommands.add(command);
                    addRequirements(command);
                }
            }
        }
        suspendedCommands.removeAll(toRemove);
        toRemove.clear();
    }


    /**
     * Resets the scheduler by clearing all running commands
     */
    public void reset() {
        runningCommands.clear();
        activeRequirements.clear();
        queuedCommands.clear();
        suspendedCommands.clear();
    }

    public boolean isRunning(Command command) {
        return runningCommands.contains(command);
    }

    public boolean isScheduled(Command command) {
        return isRunning(command) || suspendedCommands.contains(command) || queuedCommands.contains(command);
    }

    public void cancel(Command command) {
        if (isScheduled(command)) {
            command.end(EndCondition.INTERRUPTED);
            removeRequirements(command);
            runningCommands.remove(command);
            suspendedCommands.remove(command);
            queuedCommands.remove(command);
        }
    }
}