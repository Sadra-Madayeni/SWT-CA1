package autopilot.core;

import autopilot.exception.IllegalGearChangeException;
import autopilot.sensor.SpeedSensor;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.params.ParameterizedTest;
import org.junit.jupiter.params.provider.CsvSource;

import static org.junit.jupiter.api.Assertions.*;

class GearBoxTest {

    private GearBox gearBox;
    private double currentSpeed;

    @BeforeEach
    void setUp() {
        SpeedSensor speedSensor = () -> currentSpeed;
        gearBox = new GearBox(speedSensor);
        currentSpeed = 0.0;
    }

    @Test
    void initialGear_isPark() {
        assertEquals(GearBox.Gear.P, gearBox.getCurrentGear());
    }

    @Test
    void changeTo_sameGear_doesNothing() {
        currentSpeed = 100.0;
        assertDoesNotThrow(() -> gearBox.changeTo(GearBox.Gear.P));
    }

    @ParameterizedTest
    @CsvSource({"0.0", "4.9"})
    void changeTo_parkToReverse_validSpeed_changesGear(double speed) {
        currentSpeed = speed;
        gearBox.changeTo(GearBox.Gear.R);
        assertEquals(GearBox.Gear.R, gearBox.getCurrentGear());
    }

    @Test
    void changeTo_parkToReverse_invalidSpeed_throwsException() {
        currentSpeed = 5.0;
        assertThrows(IllegalGearChangeException.class, () -> gearBox.changeTo(GearBox.Gear.R));
    }

    @Test
    void changeTo_reverseToDrive_atZeroSpeed_changesGear() {
        gearBox.changeTo(GearBox.Gear.R);
        
        currentSpeed = 0.0;
        gearBox.changeTo(GearBox.Gear.D);
        assertEquals(GearBox.Gear.D, gearBox.getCurrentGear());
    }

    @Test
    void changeTo_reverseToDrive_moving_throwsException() {
        gearBox.changeTo(GearBox.Gear.R);
        currentSpeed = 1.0;
        assertThrows(IllegalGearChangeException.class, () -> gearBox.changeTo(GearBox.Gear.D));
    }

    @Test
    void changeTo_reverseToNeutral_allowedAtAnySpeed() {
        gearBox.changeTo(GearBox.Gear.R);
        currentSpeed = 50.0;
        gearBox.changeTo(GearBox.Gear.N);
        assertEquals(GearBox.Gear.N, gearBox.getCurrentGear());
    }

    @Test
    void changeTo_negativeSpeed_throwsIllegalArgumentException() {
        currentSpeed = -1.0;
        assertThrows(IllegalArgumentException.class, () -> gearBox.changeTo(GearBox.Gear.R));
    }
}