package frc.robot.subsystems;

import java.util.function.Consumer;
import java.util.function.Function;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj2.command.ProfiledPIDSubsystem;

public abstract class TunablePIDSubsystem extends ProfiledPIDSubsystem {

    private String baseTopic;

    /*
     * private DoubleEntry pEntry;
     * private DoubleEntry iEntry;
     * private DoubleEntry dEntry;
     * 
     * private final DoubleSubscriber pSub;
     * private final AtomicReference<Double> pValue;
     */
    public TunablePIDSubsystem(String baseTopic) {
        super(createController(baseTopic), 0);
        /*
         * super(new ProfiledPIDController(
         * getP(baseTopic),
         * getI(baseTopic),
         * getD(baseTopic),
         * new TrapezoidProfile.Constraints(
         * getMaxVelocity(baseTopic),
         * getMaxAcceleration(baseTopic))),
         * 0);
         */
        this.baseTopic = baseTopic;

        // Preferences.initDouble(getPTopic(baseTopic), 0);
        /*
         * DoubleTopic topic;
         * var nt = NetworkTableInstance.getDefault();
         * 
         * this.pValue = new AtomicReference<Double>();
         * var pTopic = getPTopic(baseTopic);
         * this.pSub =
         * nt.getDoubleTopic(getPTopic(baseTopic)).getEntry(getP(baseTopic));
         * nt.addListener(this.pSub, EnumSet.of(NetworkTableEvent.Kind.kValueAll),
         * event -> this.pValue.set(event.valueData.value.getDouble()));
         * 
         * 
         * topic = nt.getDoubleTopic(getITopic(baseTopic));
         * this.iEntry = topic.getEntry(getI(baseTopic));
         * 
         * topic = nt.getDoubleTopic(getDTopic(baseTopic));
         * this.dEntry = topic.getEntry(getD(baseTopic));
         */
    }

    @Override
    public void periodic() {
        super.periodic();
        this.update(TunablePIDSubsystem::getPTopic, this::setP);
        this.update(TunablePIDSubsystem::getITopic, this::setI);
        this.update(TunablePIDSubsystem::getDTopic, this::setD);
        this.update(TunablePIDSubsystem::getMaxVelocityTopic, this::setMaxVelocity);
        this.update(TunablePIDSubsystem::getMaxAccelerationTopic, this::setMaxAcceleration);
    }

    private void update(Function<String, String> getTopic, Consumer<Double> set) {
        var topic = getTopic.apply(this.baseTopic);
        var val = Preferences.getDouble(topic, 0);
        set.accept(val);
    }

    private void setP(double p) {
        this.getController().setP(p);
        Preferences.setDouble(getPTopic(this.baseTopic), p);
    }

    private static double getP(String baseTopic) {
        return Preferences.getDouble(getPTopic(baseTopic), 0);
    }

    private void setI(double i) {
        this.getController().setI(i);
        Preferences.setDouble(getITopic(this.baseTopic), i);
    }

    private static double getI(String baseTopic) {
        return Preferences.getDouble(getITopic(baseTopic), 0);
    }

    private void setD(double d) {
        this.getController().setD(d);
        Preferences.setDouble(getDTopic(this.baseTopic), d);
    }

    private static double getD(String baseTopic) {
        return Preferences.getDouble(getDTopic(baseTopic), 0);
    }

    private void setMaxVelocity(double maxVelocity) {
        Preferences.setDouble(getMaxVelocityTopic(this.baseTopic), maxVelocity);
    }

    private static double getMaxVelocity(String baseTopic) {
        return Preferences.getDouble(getMaxVelocityTopic(baseTopic), 0);
    }

    private void setMaxAcceleration(double maxAcceleration) {
        Preferences.setDouble(getMaxVelocityTopic(this.baseTopic), maxAcceleration);
    }

    private static double getMaxAcceleration(String baseTopic) {
        return Preferences.getDouble(getMaxAccelerationTopic(baseTopic), 0);
    }

    private static String getPTopic(String baseTopic) {
        return combineTopic(baseTopic, "P");
    }

    private static String getITopic(String baseTopic) {
        return combineTopic(baseTopic, "I");
    }

    private static String getDTopic(String baseTopic) {
        return combineTopic(baseTopic, "D");
    }

    private static String getMaxVelocityTopic(String baseTopic) {
        return combineTopic(baseTopic, "MaxVelocityRadsPerSecond");
    }

    private static String getMaxAccelerationTopic(String baseTopic) {
        return combineTopic(baseTopic, "MaxAccelerationRadsPerSecondSquared");
    }

    private static String combineTopic(String baseTopic, String path) {
        return baseTopic + "/" + path;
    }

    private static ProfiledPIDController createController(String baseTopic) {
        Preferences.initDouble(getPTopic(baseTopic), 0);
        Preferences.initDouble(getITopic(baseTopic), 0);
        Preferences.initDouble(getDTopic(baseTopic), 0);
        Preferences.initDouble(getMaxVelocityTopic(baseTopic), 0);
        Preferences.initDouble(getMaxAccelerationTopic(baseTopic), 0);
        return new ProfiledPIDController(
                getP(baseTopic),
                getI(baseTopic),
                getD(baseTopic),
                new TrapezoidProfile.Constraints(
                        getMaxVelocity(baseTopic),
                        getMaxAcceleration(baseTopic)));
    }

}
