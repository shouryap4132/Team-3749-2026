package frc.robot.utils;

import java.util.function.Supplier;

import org.littletonrobotics.junction.networktables.LoggedNetworkBoolean;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;
import org.littletonrobotics.junction.networktables.LoggedNetworkString;

public class TunableInput<T> {
    private final Supplier<T> supplier;
    private T inputValue;

    private TunableInput(Supplier<T> supplier, T defaultValue) {
        this.supplier = supplier;
        this.inputValue = defaultValue;
    }

    public static TunableInput<Double> number(String key, double defaultValue) {
        LoggedNetworkNumber input = new LoggedNetworkNumber("/Tuning/" + key, defaultValue);
        return new TunableInput<>(input::get, defaultValue);
    }

    public static TunableInput<Boolean> bool(String key, boolean defaultValue) {
        LoggedNetworkBoolean input = new LoggedNetworkBoolean("/Tuning/" + key, defaultValue);
        return new TunableInput<>(input::get, defaultValue);
    }

    public static TunableInput<String> string(String key, String defaultValue) {
        LoggedNetworkString input = new LoggedNetworkString("/Tuning/" + key, defaultValue);
        return new TunableInput<>(input::get, defaultValue);
    }

    public T get() {
        inputValue = supplier.get();
        return inputValue;
    }
}