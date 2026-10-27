package autopilot.core;

import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.DisplayName;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.params.ParameterizedTest;
import org.junit.jupiter.params.provider.CsvSource;
import org.junit.jupiter.params.provider.EnumSource;
import org.junit.jupiter.params.provider.ValueSource;

import static org.junit.jupiter.api.Assertions.*;

class BrakeControllerTest {

    private static final double MAX_ALLOWED_PRESSURE = 100.0;
    private static final double NORMAL_CITY_SPEED_KMH = 50.0;
    private static final double ABS_TRIGGER_SPEED_KMH = 60.0;
    private static final double ABS_HIGH_PULSE_PRESSURE = 90.0;
    private static final double ABS_LOW_PULSE_PRESSURE = 60.0;
    private static final double ABS_ACTIVE_PRESSURE_REQUEST = 85.0;

    private BrakeController brakeController;

    @BeforeEach
    void setUp() {
        brakeController = new BrakeController();
    }

    @ParameterizedTest(name = "Invalid pressure value {0} must throw IllegalArgumentException")
    @ValueSource(doubles = {-0.1, -10.0, Double.NaN, Double.POSITIVE_INFINITY, Double.NEGATIVE_INFINITY})
    @DisplayName("Should throw IllegalArgumentException when requested pressure is negative or non-finite")
    void apply_invalidPressure_throwsIllegalArgumentException(double invalidPressure) {
        assertThrows(IllegalArgumentException.class, () ->
                brakeController.apply(invalidPressure, NORMAL_CITY_SPEED_KMH, GearBox.Gear.D)
        );
    }

    @ParameterizedTest(name = "Invalid speed value {0} must throw IllegalArgumentException")
    @ValueSource(doubles = {-1.0, -50.0, Double.NaN, Double.POSITIVE_INFINITY, Double.NEGATIVE_INFINITY})
    @DisplayName("Should throw IllegalArgumentException when speed is negative or non-finite")
    void apply_invalidSpeed_throwsIllegalArgumentException(double invalidSpeed) {
        assertThrows(IllegalArgumentException.class, () ->
                brakeController.apply(50.0, invalidSpeed, GearBox.Gear.D)
        );
    }

    @Test
    @DisplayName("Should throw IllegalStateException when braking while gear is in PARK")
    void apply_gearInPark_throwsIllegalStateException() {
        assertThrows(IllegalStateException.class, () ->
                brakeController.apply(50.0, NORMAL_CITY_SPEED_KMH, GearBox.Gear.P)
        );
    }

    @Test
    @DisplayName("Should throw NullPointerException when gear is null")
    void apply_nullGear_throwsNullPointerException() {
        assertThrows(NullPointerException.class, () ->
                brakeController.apply(50.0, NORMAL_CITY_SPEED_KMH, null)
        );
    }

    @ParameterizedTest(name = "Requested Pressure: {0}, Speed: {1} -> Expected Pressure: {2}")
    @CsvSource({
            "0.0, 20.0, 0.0",      
            "40.0, 50.0, 40.0",   
            "120.0, 20.0, 100.0",     
            "80.0, 50.0, 80.0",      
            "85.0, 30.0, 85.0",  
            "80.1, 30.1, 90.0"        
    })
    @DisplayName("Should correctly handle bounded pressure and boundary conditions of ABS activation")
    void apply_variousConditions_returnsExpectedBoundedOrAbsPressure(double pressureReq, double speed, double expected) {
        double actualPressure = brakeController.apply(pressureReq, speed, GearBox.Gear.D);
        assertEquals(expected, actualPressure);
    }

    @Test
    @DisplayName("Should alternate ABS pulses and reset next pulse to high after exiting ABS condition")
    void apply_absCondition_alternatesPulsesAndResetsAfterNormalBraking() {
       
        assertEquals(ABS_HIGH_PULSE_PRESSURE,
                brakeController.apply(ABS_ACTIVE_PRESSURE_REQUEST, ABS_TRIGGER_SPEED_KMH, GearBox.Gear.D));

      
        assertEquals(ABS_LOW_PULSE_PRESSURE,
                brakeController.apply(ABS_ACTIVE_PRESSURE_REQUEST, ABS_TRIGGER_SPEED_KMH, GearBox.Gear.D));

        assertEquals(20.0,
                brakeController.apply(20.0, ABS_TRIGGER_SPEED_KMH, GearBox.Gear.D));

        assertEquals(ABS_HIGH_PULSE_PRESSURE,
                brakeController.apply(ABS_ACTIVE_PRESSURE_REQUEST, ABS_TRIGGER_SPEED_KMH, GearBox.Gear.D));
    }

    @ParameterizedTest(name = "Emergency braking allowed in gear {0}")
    @EnumSource(value = GearBox.Gear.class, names = {"D", "R", "N"})
    @DisplayName("Should successfully apply emergency brake for all valid driving gears")
    void emergencyBrake_validGears_activatesMaxPressureAndWarning(GearBox.Gear gear) {
        brakeController.emergencyBrake(gear);

        assertAll("Emergency Brake Verification",
                () -> assertEquals(MAX_ALLOWED_PRESSURE, brakeController.getPressure()),
                () -> assertTrue(brakeController.isWarningActive())
        );
    }

    @Test
    @DisplayName("Should throw IllegalStateException when emergency brake is triggered in PARK")
    void emergencyBrake_gearInPark_throwsIllegalStateException() {
        assertThrows(IllegalStateException.class, () ->
                brakeController.emergencyBrake(GearBox.Gear.P)
        );
    }
}