package autopilot.core;

import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.DisplayName;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.params.ParameterizedTest;
import org.junit.jupiter.params.provider.Arguments;
import org.junit.jupiter.params.provider.CsvSource;
import org.junit.jupiter.params.provider.MethodSource;
import org.junit.jupiter.params.provider.ValueSource;

import java.util.stream.Stream;

import static org.junit.jupiter.api.Assertions.*;

class AdaptiveCruiseTest {

    private static final double TARGET_SPEED_KMH = 100.0;
    private static final double EMERGENCY_BRAKE_MAX_PRESSURE = 100.0;
    private static final double HYSTERESIS_PRESSURE = 10.0;
    private static final double ZERO_AMOUNT = 0.0;

    private AdaptiveCruise adaptiveCruise;

    @BeforeEach
    void setUp() {
        adaptiveCruise = new AdaptiveCruise(TARGET_SPEED_KMH);
    }

    @ParameterizedTest(name = "Invalid speed value {0} must throw IllegalArgumentException")
    @ValueSource(doubles = {-1.0, -50.0, Double.NaN, Double.POSITIVE_INFINITY, Double.NEGATIVE_INFINITY})
    @DisplayName("safeDistanceFor should reject negative or non-finite speed values")
    void safeDistanceFor_invalidSpeed_throwsIllegalArgumentException(double invalidSpeed) {
        assertThrows(IllegalArgumentException.class, () -> adaptiveCruise.safeDistanceFor(invalidSpeed));
    }

    @ParameterizedTest(name = "Invalid distance value {0} must throw IllegalArgumentException")
    @ValueSource(doubles = {-0.1, -20.0, Double.NaN, Double.POSITIVE_INFINITY, Double.NEGATIVE_INFINITY})
    @DisplayName("update should reject negative or non-finite distance values")
    void update_invalidDistance_throwsIllegalArgumentException(double invalidDistance) {
        assertThrows(IllegalArgumentException.class, () -> adaptiveCruise.update(60.0, invalidDistance));
    }

    @ParameterizedTest(name = "Speed {0} km/h -> Safe distance {1} meters")
    @CsvSource({
            "0.0, 0.0",
            "40.0, 20.0",
            "100.0, 50.0"
    })
    @DisplayName("safeDistanceFor should compute distance as speed multiplied by 0.5")
    void safeDistanceFor_validSpeeds_returnsHalfSpeedInMeters(double speedKmh, double expectedSafeDistance) {
        assertEquals(expectedSafeDistance, adaptiveCruise.safeDistanceFor(speedKmh));
    }

    @ParameterizedTest(name = "[{index}] Speed: {0}, Dist: {1} -> Mode: {2}, Amount: {3}")
    @MethodSource("provideCruiseDrivingScenarios")
    @DisplayName("update should make accurate control decisions under boundary and normal scenarios")
    void update_variousConditions_makesCorrectDecision(
            double currentSpeed,
            double distanceMeters,
            AdaptiveCruise.Mode expectedMode,
            double expectedAmount
    ) {
        AdaptiveCruise.Decision decision = adaptiveCruise.update(currentSpeed, distanceMeters);

        assertAll("Cruise Decision Verification",
                () -> assertEquals(expectedMode, decision.mode()),
                () -> assertEquals(expectedAmount, decision.amount(), 0.001)
        );
    }

    private static Stream<Arguments> provideCruiseDrivingScenarios() {
        return Stream.of(
               
                Arguments.of(100.0, 25.0, AdaptiveCruise.Mode.EMERGENCY_BRAKE, EMERGENCY_BRAKE_MAX_PRESSURE),

                Arguments.of(100.0, 10.0, AdaptiveCruise.Mode.EMERGENCY_BRAKE, EMERGENCY_BRAKE_MAX_PRESSURE),

                Arguments.of(100.0, 40.0, AdaptiveCruise.Mode.BRAKE, 20.0),
   
                Arguments.of(80.0, 40.0, AdaptiveCruise.Mode.HOLD, ZERO_AMOUNT),
       
                Arguments.of(70.0, 80.0, AdaptiveCruise.Mode.ACCELERATE, 5.0),
   
                Arguments.of(98.0, 80.0, AdaptiveCruise.Mode.ACCELERATE, 2.0),
      
                Arguments.of(100.0, 80.0, AdaptiveCruise.Mode.HOLD, ZERO_AMOUNT),

                Arguments.of(105.0, 80.0, AdaptiveCruise.Mode.HOLD, ZERO_AMOUNT),
             
                Arguments.of(0.0, 0.0, AdaptiveCruise.Mode.EMERGENCY_BRAKE, EMERGENCY_BRAKE_MAX_PRESSURE)
        );
    }

    @Test
    @DisplayName("Should maintain hysteresis braking until strictly reaching release distance")
    void update_brakingActive_maintainsHysteresisUntilReachingReleaseDistance() {
 
        adaptiveCruise.update(100.0, 30.0);

        AdaptiveCruise.Decision hysteresisDecision = adaptiveCruise.update(100.0, 59.0);
        assertAll("Hysteresis State Verification",
                () -> assertEquals(AdaptiveCruise.Mode.BRAKE, hysteresisDecision.mode()),
                () -> assertEquals(HYSTERESIS_PRESSURE, hysteresisDecision.amount())
        );

        AdaptiveCruise.Decision releasedDecision = adaptiveCruise.update(90.0, 60.0);
        assertAll("Released State Verification",
                () -> assertEquals(AdaptiveCruise.Mode.ACCELERATE, releasedDecision.mode()),
                () -> assertEquals(5.0, releasedDecision.amount())
        );
    }
}