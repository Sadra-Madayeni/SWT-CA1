package autopilot.core;

import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.params.ParameterizedTest;
import org.junit.jupiter.params.provider.CsvSource;

import static org.junit.jupiter.api.Assertions.*;

class SteeringControllerTest {

    private SteeringController controller;

    @BeforeEach
    void setUp() {
        controller = new SteeringController();
    }

    @Test
    void update_withinTickLimit_reachesTargetImmediately() {
        double result = controller.update(45.0, 50.0);
        assertEquals(45.0, result);
        assertEquals(45.0, controller.getCurrentAngleDegrees());
        assertEquals(45.0, controller.getTargetAngleDegrees());
    }

    @Test
    void update_exceedsTickLimit_movesByTickMax() {
        double result = controller.update(100.0, 50.0);
        assertEquals(60.0, result);
        assertEquals(60.0, controller.getCurrentAngleDegrees());
        assertEquals(100.0, controller.getTargetAngleDegrees());
        
        result = controller.update(100.0, 50.0);
        assertEquals(100.0, result);
    }

    @ParameterizedTest
    @CsvSource({
            "500.0, 50.0, 450.0",
            "-500.0, 50.0, -450.0",
            "150.0, 130.0, 90.0",
            "-150.0, 130.0, -90.0"
    })
    void update_clampsToAppropriateLimits(double requestedAngle, double speed, double expectedTarget) {
        controller.update(requestedAngle, speed);
        assertEquals(expectedTarget, controller.getTargetAngleDegrees());
    }

    @Test
    void selfCenter_movesTowardZero() {
        controller.update(100.0, 50.0);
        controller.update(100.0, 50.0); 
        
        double result = controller.selfCenter();
        assertEquals(40.0, result);
        assertEquals(0.0, controller.getTargetAngleDegrees());
        
        result = controller.selfCenter();
        assertEquals(0.0, result);
    }

    @Test
    void update_negativeSpeed_throwsIllegalArgumentException() {
        assertThrows(IllegalArgumentException.class, () -> controller.update(30.0, -10.0));
    }
}