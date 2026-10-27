package autopilot.core;

import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.params.ParameterizedTest;
import org.junit.jupiter.params.provider.CsvSource;
import org.junit.jupiter.params.provider.EnumSource;
import org.junit.jupiter.params.provider.MethodSource;

import java.util.stream.Stream;

import static org.junit.jupiter.api.Assertions.*;

class BrakeControllerTest {

    private static final double MAX_PRESSURE = 100.0;
    private static final double NORMAL_SPEED_KMH = 50.0;
    private static final double ABS_SPEED_KMH = 60.0;
    private static final double ABS_REQUEST = 85.0;
    private static final double ABS_HIGH_PULSE = 90.0;
    private static final double ABS_LOW_PULSE = 60.0;

    private BrakeController brakeController;

    @BeforeEach
    void setUp() {
        brakeController = new BrakeController();
    }

    // Intentionally keep a small inline CSV example; larger tables live in CSV resource files.
    @ParameterizedTest(name = "[{index}] pressure={0}, speed={1}, gear={2} -> {3}")
    @CsvSource({
            "0.0,      20.0, D, 0.0",
            "40.0,     50.0, N, 40.0",
            "120.0,    20.0, R, 100.0",
            "80.0,     50.0, D, 80.0",
            "85.0,     30.0, D, 85.0",
            "80.0001,  30.0, D, 80.0001",
            "80.0,     30.0001, D, 80.0",
            "80.0001,  30.0001, D, 90.0",
            "120.0,    60.0, D, 90.0"
    })
    void apply_pressureAndSpeedLimits_returnsExpectedPressure(
            double requestedPressure, double speedKmh, GearBox.Gear gear, double expectedPressure) {
        assertEquals(expectedPressure, brakeController.apply(requestedPressure, speedKmh, gear));
    }

    @ParameterizedTest(name = "Invalid pressure {0}")
    @MethodSource("invalidNumbers")
    void apply_invalidPressure_rejectsWithoutChangingPressure(double invalidPressure) {
        brakeController.apply(20.0, NORMAL_SPEED_KMH, GearBox.Gear.D);
        assertThrows(IllegalArgumentException.class,
                () -> brakeController.apply(invalidPressure, NORMAL_SPEED_KMH, GearBox.Gear.D));
        assertEquals(20.0, brakeController.getPressure());
    }

    @ParameterizedTest(name = "Invalid speed {0}")
    @MethodSource("invalidNumbers")
    void apply_invalidSpeed_rejectsWithoutChangingPressure(double invalidSpeed) {
        brakeController.apply(20.0, NORMAL_SPEED_KMH, GearBox.Gear.D);
        assertThrows(IllegalArgumentException.class,
                () -> brakeController.apply(50.0, invalidSpeed, GearBox.Gear.D));
        assertEquals(20.0, brakeController.getPressure());
    }

    private static Stream<Double> invalidNumbers() {
        return Stream.of(-1.0, Double.NaN, Double.POSITIVE_INFINITY, Double.NEGATIVE_INFINITY);
    }

    @Test
    void apply_parkGear_throwsWithoutChangingPressure() {
        brakeController.apply(25.0, NORMAL_SPEED_KMH, GearBox.Gear.D);
        assertThrows(IllegalStateException.class,
                () -> brakeController.apply(60.0, NORMAL_SPEED_KMH, GearBox.Gear.P));
        assertEquals(25.0, brakeController.getPressure());
    }

    @Test
    void apply_nullGear_throwsNullPointerException() {
        assertThrows(NullPointerException.class,
                () -> brakeController.apply(50.0, NORMAL_SPEED_KMH, null));
    }

    @Test
    void apply_absPulses_alternateAndResetAfterNormalBraking() {
        assertEquals(ABS_HIGH_PULSE,
                brakeController.apply(ABS_REQUEST, ABS_SPEED_KMH, GearBox.Gear.D));
        assertEquals(ABS_LOW_PULSE,
                brakeController.apply(ABS_REQUEST, ABS_SPEED_KMH, GearBox.Gear.D));
        assertEquals(ABS_HIGH_PULSE,
                brakeController.apply(ABS_REQUEST, ABS_SPEED_KMH, GearBox.Gear.D));

        assertEquals(20.0, brakeController.apply(20.0, ABS_SPEED_KMH, GearBox.Gear.D));
        assertEquals(ABS_HIGH_PULSE,
                brakeController.apply(ABS_REQUEST, ABS_SPEED_KMH, GearBox.Gear.D));
    }

    @Test
    void apply_speedAtAbsThreshold_resetsNextPulseToHigh() {
        brakeController.apply(ABS_REQUEST, ABS_SPEED_KMH, GearBox.Gear.D); // next pulse low
        assertEquals(ABS_REQUEST, brakeController.apply(ABS_REQUEST, 30.0, GearBox.Gear.D));
        assertEquals(ABS_HIGH_PULSE,
                brakeController.apply(ABS_REQUEST, ABS_SPEED_KMH, GearBox.Gear.D));
    }

    @Test
    void emergencyBrake_duringAbsSequence_resetsPulseToHigh() {
        brakeController.apply(ABS_REQUEST, ABS_SPEED_KMH, GearBox.Gear.D); // next ABS pulse would be low
        brakeController.emergencyBrake(GearBox.Gear.D);
        assertEquals(MAX_PRESSURE, brakeController.getPressure());
        assertTrue(brakeController.isWarningActive());
        assertEquals(ABS_HIGH_PULSE,
                brakeController.apply(ABS_REQUEST, ABS_SPEED_KMH, GearBox.Gear.D));
    }

    @ParameterizedTest(name = "Emergency braking in {0}")
    @EnumSource(value = GearBox.Gear.class, names = {"D", "R", "N"})
    void emergencyBrake_validGear_activatesMaximumPressureAndWarning(GearBox.Gear gear) {
        brakeController.emergencyBrake(gear);
        assertAll("Emergency braking",
                () -> assertEquals(MAX_PRESSURE, brakeController.getPressure()),
                () -> assertTrue(brakeController.isWarningActive()));
    }

    @Test
    void emergencyBrake_parkGear_throwsAndDoesNotWarn() {
        assertThrows(IllegalStateException.class,
                () -> brakeController.emergencyBrake(GearBox.Gear.P));
        assertFalse(brakeController.isWarningActive());
        assertEquals(0.0, brakeController.getPressure());
    }

    @Test
    void emergencyBrake_nullGear_throwsAndDoesNotWarn() {
        assertThrows(NullPointerException.class, () -> brakeController.emergencyBrake(null));
        assertFalse(brakeController.isWarningActive());
    }
}
