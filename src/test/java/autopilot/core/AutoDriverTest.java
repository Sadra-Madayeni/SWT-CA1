package autopilot.core;

import autopilot.exception.SensorFailureException;
import autopilot.sensor.LaneSensor;
import autopilot.sensor.ObstacleSensor;
import autopilot.sensor.SpeedSensor;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.params.ParameterizedTest;
import org.junit.jupiter.params.provider.Arguments;
import org.junit.jupiter.params.provider.MethodSource;

import java.util.stream.Stream;

import static org.junit.jupiter.api.Assertions.*;

class AutoDriverTest {

    private static final double TARGET_SPEED_KMH = 80.0;
    private static final double CLEAR_DISTANCE_METERS = 100.0;
    private static final double CENTERED_LANE_METERS = 0.0;
    private static final double CORRECTION_DEGREES = 45.0;
    private static final double SAFE_MODE_BRAKE_PRESSURE = 30.0;

    private SpeedSensor speedSensor;
    private ObstacleSensor obstacleSensor;
    private LaneSensor laneSensor;
    private AutoDriver autoDriver;

    @BeforeEach
    void setUp() {
        speedSensor = () -> TARGET_SPEED_KMH;
        obstacleSensor = () -> CLEAR_DISTANCE_METERS;
        laneSensor = () -> CENTERED_LANE_METERS;
        autoDriver = new AutoDriver(speedSensor, obstacleSensor, laneSensor, TARGET_SPEED_KMH);
    }

    @Test
    void tick_normalDrivingConditions_returnsExpectedCruiseAndSteering() {
        AutoDriver.DriveResult result = autoDriver.tick(GearBox.Gear.D);
        assertAll("Normal driving",
                () -> assertEquals(0.0, result.steeringAngleDegrees()),
                () -> assertEquals(0.0, result.brakePressure()),
                () -> assertEquals(AdaptiveCruise.Mode.HOLD, result.cruiseMode()),
                () -> assertFalse(result.warningActive()),
                () -> assertFalse(result.safeMode()));
    }

    @ParameterizedTest(name = "Lane offset {0} -> steering {1} degrees")
    @MethodSource("laneCorrectionScenarios")
    void tick_laneOffsetDetected_appliesExactCorrection(double offsetMeters, double expectedSteering) {
        laneSensor = () -> offsetMeters;
        autoDriver = new AutoDriver(speedSensor, obstacleSensor, laneSensor, TARGET_SPEED_KMH);

        AutoDriver.DriveResult result = autoDriver.tick(GearBox.Gear.D);
        assertAll("Correct steering direction and magnitude",
                () -> assertEquals(expectedSteering, result.steeringAngleDegrees()),
                () -> assertEquals(AdaptiveCruise.Mode.HOLD, result.cruiseMode()),
                () -> assertFalse(result.safeMode()));
    }

    private static Stream<Arguments> laneCorrectionScenarios() {
        return Stream.of(
                Arguments.of(1.0, -CORRECTION_DEGREES),
                Arguments.of(-1.0, CORRECTION_DEGREES));
    }


    @ParameterizedTest(name = "Failed sensor: {0}")
    @MethodSource("failingSensors")
    void tick_sensorFailure_entersSafeMode(String failingSensor) {
        if ("SPEED".equals(failingSensor)) {
            speedSensor = () -> {
                throw new SensorFailureException("Speed sensor failed");
            };
        } else {
            obstacleSensor = () -> {
                throw new SensorFailureException("Obstacle sensor failed");
            };
        }
        autoDriver = new AutoDriver(speedSensor, obstacleSensor, laneSensor, TARGET_SPEED_KMH);

        AutoDriver.DriveResult result = autoDriver.tick(GearBox.Gear.D);
        assertAll("Sensor failure activates safe mode",
                () -> assertTrue(result.safeMode()),
                () -> assertEquals(SAFE_MODE_BRAKE_PRESSURE, result.brakePressure()),
                () -> assertEquals(0.0, result.steeringAngleDegrees()),
                () -> assertEquals(AdaptiveCruise.Mode.BRAKE, result.cruiseMode()),
                () -> assertFalse(result.warningActive()));
    }

    private static Stream<String> failingSensors() {
        return Stream.of("SPEED", "OBSTACLE");
    }
}
