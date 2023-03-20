package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Arm extends SubsystemBase {

    private final Shoulder shoulder;
    private final Elbow elbow;
    private final Claw claw;

    public Arm() {
        super();
        this.shoulder = new Shoulder();
        this.elbow = new Elbow();
        this.claw = new Claw();
        this.addChild("Shoulder", this.shoulder);
        this.addChild("Elbow", this.elbow);
        this.addChild("Claw", this.claw);
    }

    public Shoulder getShoulder() {
        return this.shoulder;
    }

    public Elbow getElbow() {
        return this.elbow;
    }

    public Claw getClaw() {
        return this.claw;
    }
}
