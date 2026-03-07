package frc.robot.utility;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.generated.TunerConstants;

public class CircleFeildZones {
    public Translation2d Target;
    public Double radius1;
    public Double radius2;

    public CircleFeildZones(Translation2d target, Double radius1, Double radius2) {
        Target = target;
        this.radius1 = radius1;
        this.radius2 = radius2;
    }

}
