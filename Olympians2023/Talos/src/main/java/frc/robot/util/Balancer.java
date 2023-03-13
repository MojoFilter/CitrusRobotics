package frc.robot.util;

import edu.wpi.first.wpilibj.interfaces.Accelerometer;
import frc.robot.Constants.Balance;

public class Balancer {
    private final Accelerometer accel;

    private int state;
    private int ticks;

    public Balancer(Accelerometer accel) {
        this.accel = accel;
        this.reset();
    }

    public void reset() {
        this.state = 0;
        this.ticks = 0;
    }

    public double getBalanceSpeed() {
        switch (this.state) {
            case 0:
                // drive forwards to approach station, exit when tilt is detected
                if (getTilt() > Balance.OnChargeStationDegrees) {
                    this.ticks++;
                }
                if (ticks > SecondsToTicks(Balance.DebounceTime)) {
                    this.state = 1;
                    this.ticks = 0;
                    return Balance.TalosSpeedSlow;
                }
                return Balance.TalosSpeedFast;
            case 1:
                // driving up charge station, drive slower, stopping when level
                if (this.getTilt() < Balance.LevelDegrees) {
                    this.ticks++;
                }
                if (this.ticks > SecondsToTicks(Balance.DebounceTime)) {
                    state = 2;
                    ticks = 0;
                    return 0;
                }
                return Balance.TalosSpeedSlow;
            case 2:
                // on charge station, stop motors and wait for end of auto
                if (Math.abs(getTilt()) <= Balance.LevelDegrees / 2) {
                    ticks++;
                }
                if (ticks > SecondsToTicks(Balance.DebounceTime)) {
                    state = 4;
                    ticks = 0;
                    return 0;
                }
                if (getTilt() >= Balance.LevelDegrees) {
                    return 0.1;
                } else if (getTilt() <= -Balance.LevelDegrees) {
                    return -0.1;
                }
            case 3:
                return 0;
        }
        return 0;
    }

    // Same as auto balance above, but starts auto period by scoring
    // a game piece on the back bumper of the robot
    public double getScoreAndBalanceSpeed() {
        switch (this.state) {
            case 0:
                // drive back, then forwards, then back again to knock off and score game piece
                this.ticks++;
                if (this.ticks < SecondsToTicks(Balance.FirstTapTime)) {
                    return -Balance.TalosSpeedFast;
                } else if (this.ticks < SecondsToTicks(Balance.FirstTapTime + Balance.ScoringBackUpTime)) {
                    return Balance.TalosSpeedFast;
                } else if (this.ticks < SecondsToTicks(
                        Balance.FirstTapTime + Balance.ScoringBackUpTime + Balance.SecondTapTime)) {
                    return -Balance.TalosSpeedFast;
                } else {
                    this.ticks = 0;
                    this.state = 1;
                    return 0;
                }
            case 1:
                // drive forwards until on charge station
                if (this.getTilt() > Balance.OnChargeStationDegrees) {
                    this.ticks++;
                }
                if (ticks > SecondsToTicks(Balance.DebounceTime)) {
                    this.state = 2;
                    this.ticks = 0;
                    return Balance.TalosSpeedSlow;
                }
                return Balance.TalosSpeedFast;
            case 2:
                // driving up charge station, drive slower, stopping when level
                if (getTilt() < Balance.LevelDegrees) {
                    this.ticks++;
                }
                if (ticks > SecondsToTicks(Balance.DebounceTime)) {
                    this.state = 3;
                    this.ticks = 0;
                    return 0;
                }
                return Balance.TalosSpeedSlow;
            case 3:
                // on charge station, ensure robot is flat, then end auto
                if (Math.abs(this.getTilt()) <= Balance.LevelDegrees / 2) {
                    this.ticks++;
                }
                if (this.ticks > SecondsToTicks(Balance.DebounceTime)) {
                    this.state = 4;
                    this.ticks = 0;
                    return 0;
                }
                if (this.getTilt() >= Balance.LevelDegrees) {
                    return Balance.TalosSpeedSlow / 2;
                } else if (getTilt() <= -Balance.LevelDegrees) {
                    return -Balance.TalosSpeedSlow / 2;
                }
            case 4:
                return 0;
        }
        return 0;
    }

    private double getPitch() {
        var ac = this.accel;
        return Math.atan2((-ac.getX()),
                Math.sqrt(ac.getY() * ac.getY() + ac.getZ() * ac.getZ())) * Balance.RAD2DEG;
    }

    private double getRoll() {
        return Math.atan2(this.accel.getY(), this.accel.getZ()) * Balance.RAD2DEG;
    }

    private double getTilt() {
        var pitch = this.getPitch();
        var roll = this.getRoll();
        var direction = pitch + roll > 0 ? 1 : -1;
        return Math.sqrt(Math.pow(pitch, 2) + Math.pow(roll, 2)) * direction;
    }

    private static int SecondsToTicks(double seconds) {
        return (int) (seconds * Balance.TicksPerSecond);
    }
}
