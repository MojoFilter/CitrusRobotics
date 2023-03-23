package frc.robot.util;

import com.kauailabs.navx.frc.AHRS;

public class NavxBalancer extends Balancer {

    private AHRS navx;

    public NavxBalancer(AHRS navx) {
        super();
        this.navx = navx;
    }

    @Override
    protected double getRoll() {
        return this.navx.getRoll();
    }

    @Override
    protected double getPitch() {
        return this.navx.getPitch();
    }

}
