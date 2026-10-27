package autopilot.core;

import autopilot.exception.IllegalGearChangeException;
import autopilot.exception.SensorFailureException;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.params.ParameterizedTest;
import org.junit.jupiter.params.provider.CsvFileSource;
import org.junit.jupiter.params.provider.MethodSource;

import java.util.stream.Stream;

import static org.junit.jupiter.api.Assertions.*;

class GearBoxTest {

    private static final double STOPPED_SPEED_KMH = 0.0;

    private double currentSpeed;
    private GearBox gearBox;

    @BeforeEach
    void setUp() {
        currentSpeed = STOPPED_SPEED_KMH;
        gearBox = new GearBox(() -> currentSpeed);
    }

    // Begin each case in the requested gear via legitimate transitions, never by modifying internals.
    private void moveToGear(GearBox.Gear startingGear) {
        if (startingGear == GearBox.Gear.P) {
            return;
        }
        gearBox.changeTo(GearBox.Gear.R);
        if (startingGear == GearBox.Gear.N) {
            gearBox.changeTo(GearBox.Gear.N);
        } else if (startingGear == GearBox.Gear.D) {
            gearBox.changeTo(GearBox.Gear.D);
        }
    }

    @ParameterizedTest(name = "[{index}] {0} -> {1} at {2} km/h: allowed={3}")
    @CsvFileSource(resources = "/gear-transitions.csv", numLinesToSkip = 1)
    void changeTo_transitionMatrix_changesOnlyWhenAllowed(
            GearBox.Gear from, GearBox.Gear to, double speedKmh, boolean allowed) {
        moveToGear(from);
        currentSpeed = speedKmh;

        if (allowed) {
            assertDoesNotThrow(() -> gearBox.changeTo(to));
            assertEquals(to, gearBox.getCurrentGear());
        } else {
            assertThrows(IllegalGearChangeException.class, () -> gearBox.changeTo(to));
            assertEquals(from, gearBox.getCurrentGear(), "A rejected shift must leave the previous gear unchanged");
        }
    }

    @Test
    void changeTo_sameGear_ignoresSpeedSensorAndLeavesGearUnchanged() {
        gearBox.setSpeedSensor(() -> {
            throw new SensorFailureException("Must not read speed on a no-op shift");
        });
        assertDoesNotThrow(() -> gearBox.changeTo(GearBox.Gear.P));
        assertEquals(GearBox.Gear.P, gearBox.getCurrentGear());
    }

    @ParameterizedTest(name = "Invalid speed: {0}")
    @MethodSource("invalidSpeeds")
    void changeTo_invalidSpeed_rejectsWithoutChangingGear(double invalidSpeed) {
        currentSpeed = invalidSpeed;
        assertThrows(IllegalArgumentException.class, () -> gearBox.changeTo(GearBox.Gear.R));
        assertEquals(GearBox.Gear.P, gearBox.getCurrentGear());
    }

    private static Stream<Double> invalidSpeeds() {
        return Stream.of(-1.0, Double.NaN, Double.POSITIVE_INFINITY, Double.NEGATIVE_INFINITY);
    }

    @Test
    void changeTo_sensorFailure_propagatesAndLeavesGearUnchanged() {
        gearBox.setSpeedSensor(() -> {
            throw new SensorFailureException("Speed unavailable");
        });
        assertThrows(SensorFailureException.class, () -> gearBox.changeTo(GearBox.Gear.R));
        assertEquals(GearBox.Gear.P, gearBox.getCurrentGear());
    }

    @Test
    void changeTo_nullTarget_rejectsWithoutChangingGear() {
        assertThrows(NullPointerException.class, () -> gearBox.changeTo(null));
        assertEquals(GearBox.Gear.P, gearBox.getCurrentGear());
    }

    @Test
    void setSpeedSensor_nullValue_doesNotReplaceExistingSensor() {
        assertThrows(NullPointerException.class, () -> gearBox.setSpeedSensor(null));
        gearBox.changeTo(GearBox.Gear.R);
        assertEquals(GearBox.Gear.R, gearBox.getCurrentGear());
    }
}
