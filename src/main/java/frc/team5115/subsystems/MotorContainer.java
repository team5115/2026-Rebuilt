package frc.team5115.subsystems;

import com.revrobotics.spark.SparkMax;
import java.util.ArrayList;

public interface MotorContainer {
    public default void getSparks(ArrayList<SparkMax> sparks) {}
}
