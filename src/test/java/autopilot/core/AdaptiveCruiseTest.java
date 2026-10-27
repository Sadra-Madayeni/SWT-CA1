package autopilot.core;

import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.params.ParameterizedTest;
import org.junit.jupiter.params.provider.Arguments;
import org.junit.jupiter.params.provider.MethodSource;
import org.junit.jupiter.params.provider.ValueSource;

import java.util.stream.Stream;

import static org.junit.jupiter.api.Assertions.*;

class AdaptiveCruiseTest {

    private static final double TARGET_SPEED_KMH = 100.0;
    private static final double SAFE_DISTANCE_METERS = 50.0;
    private static final double EMERGENCY_DISTANCE_METERS = 25.0;
    private static final double RELEASE_DISTANCE_METERS = 60.0;
    private static final double HYSTERESIS_BRAKE_PRESSURE = 10.0;
    private static final double ZERO_AMOUNT = 0.0;
    private static final double DECISION_TOLERANCE = 0.0001;

    private AdaptiveCruise adaptiveCruise;

    @BeforeEach
    void setUp() {
        adaptiveCruise = new AdaptiveCruise(TARGET_SPEED_KMH);
    }

    @ParameterizedTest(name = "Invalid speed {0}")
    @ValueSource(doubles = {-1.0, -50.0, Double.NaN, Double.POSITIVE_INFINITY, Double.NEGATIVE_INFINITY})
    void safeDistanceFor_invalidSpeed_throwsIllegalArgumentException(double invalidSpeed) {
        assertThrows(IllegalArgumentException.class, () -> adaptiveCruise.safeDistanceFor(invalidSpeed));
    }

    @Test
    void safeDistanceFor_validSpeed_returnsHalfTheSpeed() {
        assertEquals(0.0, adaptiveCruise.safeDistanceFor(0.0));
        assertEquals(20.0, adaptiveCruise.safeDistanceFor(40.0));
        assertEquals(SAFE_DISTANCE_METERS, adaptiveCruise.safeDistanceFor(TARGET_SPEED_KMH));
    }

    @ParameterizedTest(name = "[{index}] speed={0}, distance={1}, mode={2}")
    @MethodSource("drivingScenarios")
    void update_independentDrivingConditions_returnsCorrectDecision(
            double speedKmh, double distanceMeters, AdaptiveCruise.Mode expectedMode, double expectedAmount) {
        AdaptiveCruise.Decision decision = adaptiveCruise.update(speedKmh, distanceMeters);
        assertAll("Cruise control decision",
                () -> assertEquals(expectedMode, decision.mode()),
                () -> assertEquals(expectedAmount, decision.amount(), DECISION_TOLERANCE));
    }

    private static Stream<Arguments> drivingScenarios() {
        return Stream.of(
                Arguments.of(100.0, 0.0, AdaptiveCruise.Mode.EMERGENCY_BRAKE, 100.0),
                Arguments.of(100.0, EMERGENCY_DISTANCE_METERS, AdaptiveCruise.Mode.EMERGENCY_BRAKE, 100.0),
                Arguments.of(100.0, 25.001, AdaptiveCruise.Mode.BRAKE, 49.998),
                Arguments.of(100.0, 40.0, AdaptiveCruise.Mode.BRAKE, 20.0),
                Arguments.of(100.0, 49.0, AdaptiveCruise.Mode.BRAKE, 2.0),
                Arguments.of(100.0, SAFE_DISTANCE_METERS, AdaptiveCruise.Mode.HOLD, ZERO_AMOUNT),
                Arguments.of(100.0, 80.0, AdaptiveCruise.Mode.HOLD, ZERO_AMOUNT),
                Arguments.of(80.0, 40.0, AdaptiveCruise.Mode.HOLD, ZERO_AMOUNT),
                Arguments.of(70.0, 80.0, AdaptiveCruise.Mode.ACCELERATE, 5.0),
                Arguments.of(98.0, 80.0, AdaptiveCruise.Mode.ACCELERATE, 2.0),
                Arguments.of(105.0, 80.0, AdaptiveCruise.Mode.HOLD, ZERO_AMOUNT),
                Arguments.of(0.0, 0.0, AdaptiveCruise.Mode.EMERGENCY_BRAKE, 100.0),
                Arguments.of(0.0, 20.0, AdaptiveCruise.Mode.ACCELERATE, 5.0));
    }

    @ParameterizedTest(name = "Invalid speed {0}")
    @MethodSource("invalidNumbers")
    void update_invalidSpeed_throwsIllegalArgumentException(double invalidSpeed) {
        assertThrows(IllegalArgumentException.class, () -> adaptiveCruise.update(invalidSpeed, 80.0));
    }

    @ParameterizedTest(name = "Invalid distance {0}")
    @MethodSource("invalidNumbers")
    void update_invalidDistance_throwsIllegalArgumentException(double invalidDistance) {
        assertThrows(IllegalArgumentException.class, () -> adaptiveCruise.update(80.0, invalidDistance));
    }

    private static Stream<Double> invalidNumbers() {
        return Stream.of(-0.1, Double.NaN, Double.POSITIVE_INFINITY, Double.NEGATIVE_INFINITY);
    }

    @Test
    void update_brakingState_releasesExactlyAtHysteresisBoundary() {
        assertDecision(adaptiveCruise.update(100.0, 40.0), AdaptiveCruise.Mode.BRAKE, 20.0);
        assertDecision(adaptiveCruise.update(100.0, RELEASE_DISTANCE_METERS - 0.001),
                AdaptiveCruise.Mode.BRAKE, HYSTERESIS_BRAKE_PRESSURE);
        // Same speed: at exactly 50 * 1.2 = 60 meters hysteresis must release.
        assertDecision(adaptiveCruise.update(100.0, RELEASE_DISTANCE_METERS),
                AdaptiveCruise.Mode.HOLD, ZERO_AMOUNT);
        assertDecision(adaptiveCruise.update(100.0, 59.0), AdaptiveCruise.Mode.HOLD, ZERO_AMOUNT);
    }

    @Test
    void update_emergencyBraking_remainsInHysteresisUntilRelease() {
        assertDecision(adaptiveCruise.update(100.0, EMERGENCY_DISTANCE_METERS),
                AdaptiveCruise.Mode.EMERGENCY_BRAKE, 100.0);
        assertDecision(adaptiveCruise.update(100.0, SAFE_DISTANCE_METERS),
                AdaptiveCruise.Mode.BRAKE, HYSTERESIS_BRAKE_PRESSURE);
        assertDecision(adaptiveCruise.update(100.0, RELEASE_DISTANCE_METERS),
                AdaptiveCruise.Mode.HOLD, ZERO_AMOUNT);
        assertDecision(adaptiveCruise.update(90.0, 80.0), AdaptiveCruise.Mode.ACCELERATE, 5.0);
    }

    @Test
    void update_invalidInput_doesNotClearActiveHysteresis() {
        adaptiveCruise.update(100.0, 40.0);
        assertThrows(IllegalArgumentException.class,
                () -> adaptiveCruise.update(100.0, Double.NaN));
        assertDecision(adaptiveCruise.update(100.0, 55.0),
                AdaptiveCruise.Mode.BRAKE, HYSTERESIS_BRAKE_PRESSURE);
    }

    @Test
    void update_afterHysteresisRelease_entersBrakingAgainAtUnsafeDistance() {
        adaptiveCruise.update(100.0, 40.0);
        adaptiveCruise.update(100.0, RELEASE_DISTANCE_METERS);
        assertDecision(adaptiveCruise.update(100.0, 49.0), AdaptiveCruise.Mode.BRAKE, 2.0);
        assertDecision(adaptiveCruise.update(100.0, 50.0),
                AdaptiveCruise.Mode.BRAKE, HYSTERESIS_BRAKE_PRESSURE);
    }

    private static void assertDecision(AdaptiveCruise.Decision decision,
                                       AdaptiveCruise.Mode expectedMode, double expectedAmount) {
        assertAll("Cruise decision",
                () -> assertEquals(expectedMode, decision.mode()),
                () -> assertEquals(expectedAmount, decision.amount(), DECISION_TOLERANCE));
    }
}
