package frc.robot.utility;

import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.wpilibj.Timer;

public class RumbleProfile {

    private Timer timer;
    private List<RumbleStep> steps;
    private int index;

    public RumbleProfile() {
        this.steps = new ArrayList<>();
        this.timer = new Timer();
        index = 0;
    }

    public void addStep(RumbleStep vs) {
        if (vs.getInterval() > 0)
            this.steps.add(vs);
    }

    public void reset() {
        timer.stop();
        timer.reset();
        index = 0;
    }

    public RumbleStep shift() {
        if (!timer.isRunning()) {
            timer.start();
        }
        var currentStep = steps.get(index % steps.size());

        if (timer.advanceIfElapsed(currentStep.getInterval())) {
            index++;
            timer.reset();
        }

        return steps.get(index % steps.size());
    }

}
