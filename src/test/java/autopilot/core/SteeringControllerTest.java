package autopilot.core;

import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.params.ParameterizedTest;
import org.junit.jupiter.params.provider.CsvFileSource;
import org.junit.jupiter.params.provider.MethodSource;

import java.util.stream.Stream;

import static org.junit.jupiter.api.Assertions.*;

class SteeringControllerTest {

    private static final double NORMAL_SPEED_KMH = 50.0;
    private static final double MAX_CHANGE_PER_TICK_DEGREES = 60.0;

    private SteeringController controller;

    @BeforeEach
    void setUp() {
        controller = new SteeringController();
    }

    @ParameterizedTest(name = "[{index}] target {0} at speed {1}")
    @CsvFileSource(resources = "/steering-cases.csv", numLinesToSkip = 1)
    void update_requestedAngleAndSpeed_clampsTargetAndMovesInCorrectDirection(
            double requestedAngle, double speedKmh, double expectedTarget, double expectedCurrentAngle) {
        double result = controller.update(requestedAngle, speedKmh);

        assertAll("Steering target and current position",
                () -> assertEquals(expectedTarget, controller.getTargetAngleDegrees()),
                () -> assertEquals(expectedCurrentAngle, result),
                () -> assertEquals(expectedCurrentAngle, controller.getCurrentAngleDegrees()));
    }

    @Test
    void update_positiveLargeTarget_movesAtMostSixtyDegreesPerTick() {
        assertEquals(MAX_CHANGE_PER_TICK_DEGREES, controller.update(140.0, NORMAL_SPEED_KMH));
        assertEquals(120.0, controller.update(140.0, NORMAL_SPEED_KMH));
        assertEquals(140.0, controller.update(140.0, NORMAL_SPEED_KMH));
    }

    @Test
    void update_negativeLargeTarget_movesAtMostSixtyDegreesPerTick() {
        assertEquals(-MAX_CHANGE_PER_TICK_DEGREES, controller.update(-140.0, NORMAL_SPEED_KMH));
        assertEquals(-120.0, controller.update(-140.0, NORMAL_SPEED_KMH));
        assertEquals(-140.0, controller.update(-140.0, NORMAL_SPEED_KMH));
    }

    @Test
    void update_directionReversal_movesTowardNewNegativeTarget() {
        controller.update(100.0, NORMAL_SPEED_KMH); // current +60
        assertEquals(0.0, controller.update(-100.0, NORMAL_SPEED_KMH));
        assertEquals(-60.0, controller.update(-100.0, NORMAL_SPEED_KMH));
        assertEquals(-100.0, controller.update(-100.0, NORMAL_SPEED_KMH));
    }

    @Test
    void update_highSpeedAfterLowSpeed_clampsTargetAndMovesTowardNewLimit() {
        controller.update(300.0, NORMAL_SPEED_KMH); // current +60, target +300
        controller.update(300.0, NORMAL_SPEED_KMH); // current +120

        assertEquals(90.0, controller.update(300.0, 121.0));
        assertEquals(90.0, controller.getTargetAngleDegrees());
    }

    @Test
    void selfCenter_positiveAngle_reachesZeroGradually() {
        controller.update(100.0, NORMAL_SPEED_KMH);
        controller.update(100.0, NORMAL_SPEED_KMH); // +100

        assertEquals(40.0, controller.selfCenter());
        assertEquals(0.0, controller.getTargetAngleDegrees());
        assertEquals(0.0, controller.selfCenter());
    }

    @Test
    void selfCenter_negativeAngle_reachesZeroGradually() {
        controller.update(-100.0, NORMAL_SPEED_KMH);
        controller.update(-100.0, NORMAL_SPEED_KMH); // -100

        assertEquals(-40.0, controller.selfCenter());
        assertEquals(0.0, controller.getTargetAngleDegrees());
        assertEquals(0.0, controller.selfCenter());
    }

    @ParameterizedTest(name = "Invalid angle {0}")
    @MethodSource("invalidAngles")
    void update_invalidAngle_throwsWithoutChangingSteeringState(double invalidAngle) {
        controller.update(45.0, NORMAL_SPEED_KMH);

        assertThrows(IllegalArgumentException.class,
                () -> controller.update(invalidAngle, NORMAL_SPEED_KMH));
        assertEquals(45.0, controller.getCurrentAngleDegrees());
        assertEquals(45.0, controller.getTargetAngleDegrees());
    }

    private static Stream<Double> invalidAngles() {
        return Stream.of(Double.NaN, Double.POSITIVE_INFINITY, Double.NEGATIVE_INFINITY);
    }

    @ParameterizedTest(name = "Invalid speed {0}")
    @MethodSource("invalidSpeeds")
    void update_invalidSpeed_throwsWithoutChangingSteeringState(double invalidSpeed) {
        controller.update(-45.0, NORMAL_SPEED_KMH);

        assertThrows(IllegalArgumentException.class,
                () -> controller.update(90.0, invalidSpeed));
        assertEquals(-45.0, controller.getCurrentAngleDegrees());
        assertEquals(-45.0, controller.getTargetAngleDegrees());
    }

    private static Stream<Double> invalidSpeeds() {
        return Stream.of(-10.0, Double.NaN, Double.POSITIVE_INFINITY, Double.NEGATIVE_INFINITY);
    }
}
