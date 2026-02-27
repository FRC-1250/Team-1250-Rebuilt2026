package frc.robot.utility;

import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.wpilibj.Timer;

public class AgitationProfile {
    private Timer timer;
    private List<AgitationStep> steps;
    private int index;

    public AgitationProfile() {
        this.steps = new ArrayList<>();
        this.timer = new Timer();
        index = 0;
    }

    public void addStep(AgitationStep as) {
        if (as.getInterval() > 0)
            this.steps.add(as);
    }

    public void reset() {
        timer.stop();
        timer.reset();
        index = 0;
    }

    public double shift() {
        if (!timer.isRunning()) {
            timer.start();
        }

        var currentStep = steps.get(index % steps.size());

        if (timer.advanceIfElapsed(currentStep.getInterval())) {
            index++;
            currentStep = steps.get(index % steps.size());
            timer.reset();
        }

        return currentStep.getPosition();
    }
}
