package autopilot.core;

import autopilot.exception.SensorFailureException;
import autopilot.sensor.LaneSensor;
import autopilot.sensor.ObstacleSensor;
import autopilot.sensor.SpeedSensor;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.params.ParameterizedTest;
import org.junit.jupiter.params.provider.CsvSource;
import org.junit.jupiter.params.provider.ValueSource;

import static org.junit.jupiter.api.Assertions.*;

class AutoDriverTest {

    private static final double TARGET_SPEED_KMH = 80.0;
    private static final double INITIAL_OBSTACLE_DISTANCE_METERS = 100.0;
    private static final double ZERO_LANE_OFFSET = 0.0;
    private static final double SAFE_MODE_BRAKE_PRESSURE = 30.0;

    private SpeedSensor speedSensor;
    private ObstacleSensor obstacleSensor;
    private LaneSensor laneSensor;
    private AutoDriver autoDriver;

    @BeforeEach
    void setUp() {
        speedSensor = new SpeedSensor() {
            @Override
            public double getSpeedKmh() { return TARGET_SPEED_KMH; }
        };

        obstacleSensor = new ObstacleSensor() {
            @Override
            public double getDistanceMeters() { return INITIAL_OBSTACLE_DISTANCE_METERS; }
        };

        laneSensor = new LaneSensor() {
            @Override
            public double getLaneOffsetMeters() { return ZERO_LANE_OFFSET; }
        };

        autoDriver = new AutoDriver(speedSensor, obstacleSensor, laneSensor, TARGET_SPEED_KMH);
    }

    @Test
    void tick_normalDrivingConditions_returnsValidDriveResult() {
        AutoDriver.DriveResult result = autoDriver.tick(GearBox.Gear.D);

        assertNotNull(result);
        assertFalse(result.safeMode());
        assertNotNull(result.cruiseMode());
        assertEquals(0.0, result.brakePressure());
    }

    @ParameterizedTest
    @CsvSource({
            "1.0, true",
            "-1.0, false"
    })
    void tick_laneOffsetDetected_adjustsSteeringDirection(double offset, boolean shouldBeNegativeAngle) {
        laneSensor = new LaneSensor() {
            @Override
            public double getLaneOffsetMeters() { return offset; }
        };

        autoDriver = new AutoDriver(speedSensor, obstacleSensor, laneSensor, TARGET_SPEED_KMH);
        AutoDriver.DriveResult result = autoDriver.tick(GearBox.Gear.D);

        if (shouldBeNegativeAngle) {
            assertTrue(result.steeringAngleDegrees() < 0.0);
        } else {
            assertTrue(result.steeringAngleDegrees() > 0.0);
        }
    }

    @ParameterizedTest
    @ValueSource(strings = {"SPEED", "OBSTACLE"})
    void tick_sensorFailure_triggersSafeMode(String failingSensor) {
        if ("SPEED".equals(failingSensor)) {
            speedSensor = new SpeedSensor() {
                @Override public double getSpeedKmh() { throw new SensorFailureException("Speed sensor failed"); }
            };
        } else if ("OBSTACLE".equals(failingSensor)) {
            obstacleSensor = new ObstacleSensor() {
                @Override public double getDistanceMeters() { throw new SensorFailureException("Obstacle sensor failed"); }
            };
        }

        autoDriver = new AutoDriver(speedSensor, obstacleSensor, laneSensor, TARGET_SPEED_KMH);
        AutoDriver.DriveResult result = autoDriver.tick(GearBox.Gear.D);

        assertTrue(result.safeMode());
        assertEquals(SAFE_MODE_BRAKE_PRESSURE, result.brakePressure());
        assertEquals(0.0, result.steeringAngleDegrees());
        assertEquals(AdaptiveCruise.Mode.BRAKE, result.cruiseMode());
        assertFalse(result.warningActive());
    }
}