package frc.team5115.subsystems.shooter;

public enum SpeedRequest {
    AutonomouseSpinUp(Type.Slow),
    InAllianceZone(Type.Slow),
    ManualSpinUp(Type.Slow),

    AutonomouseShoot(Type.Calculated),
    SafeShoot(Type.Calculated),
    ManualShoot(Type.Calculated);

    public enum Type {
        Slow,
        Calculated
    }

    public final Type type;

    SpeedRequest(Type type) {
        this.type = type;
    }

    @Override
    public String toString() {
        return String.format("%s requests %s", super.toString(), type.toString());
    }
}
