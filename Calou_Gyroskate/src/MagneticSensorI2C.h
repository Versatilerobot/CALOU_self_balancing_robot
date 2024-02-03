#ifndef MAGNETICSENSORI2C_LIB_H
#define MAGNETICSENSORI2C_LIB_H

#include "Arduino.h"
#include <Wire.h>
#include "common/foc_utils.h"
#include "common/hardware_utils.h"
#include "common/Sensor.h"

struct MagneticSensorI2CConfig_s  {
  int chip_address;
  long clock_speed;
  int bit_resolution;
  int angle_register;
  int data_start_bit; 
};
// some predefined structures
extern MagneticSensorI2CConfig_s AS5600_I2C,AS5048_I2C;

class MagneticSensorI2C: public Sensor{
 public:
    /**
     * MagneticSensorI2C class constructor
     * @param chip_address  I2C chip address
     * @param bits number of bits of the sensor resolution 
     * @param angle_register_msb  angle read register msb
     * @param _bits_used_msb number of used bits in msb
     */
    MagneticSensorI2C(uint8_t _chip_address, int _bit_resolution, uint8_t _angle_register_msb, int _msb_bits_used);

    /**
     * MagneticSensorI2C class constructor
     * @param config  I2C config
     */
    MagneticSensorI2C(MagneticSensorI2CConfig_s config);

    static MagneticSensorI2C AS5600();
    
    /** sensor initialise pins */
    void init();

    // implementation of abstract functions of the Sensor class
    /** get current angle (rad) */
    float getAngle() override;
    /** get current angular velocity (rad/s) **/
    float getVelocity() override;
    /**
     *  set current angle as zero angle 
     * return the angle [rad] difference
     */
    float initRelativeZero() override;
    /**
     *  set absolute zero angle as zero angle
     * return the angle [rad] difference
     */
    float initAbsoluteZero() override;
    /** returns 1 because it is the absolute sensor */
    int hasAbsoluteZero() override;
    /** returns 0  maning it doesn't need search for absolute zero */

    int needsAbsoluteZeroSearch() override;

    /* the speed of the i2c clock signal */
    long clock_speed;

    /* the pin used for i2c data */
    int sda_pin;
    
    /* the pin used for i2c clock */
    int scl_pin;

  private:
    float cpr; //!< Maximum range of the magnetic sensor
    uint16_t lsb_used; //!< Number of bits used in LSB register
    uint8_t lsb_mask;
    uint8_t msb_mask;
    
    // I2C variables
    uint8_t angle_register_msb; //!< I2C angle register to read
    uint8_t chip_address; //!< I2C chip select pins

    // I2C functions
    /** Read one I2C register value */
    int read(uint8_t angle_register_msb);

    word zero_offset; //!< user defined zero offset
    /**
     * Function getting current angle register value
     * it uses angle_register variable
     */
    int getRawCount();

    // total angle tracking variables
    float full_rotation_offset; //!<number of full rotations made
    float angle_data_prev; //!< angle in previous position calculation step

    // velocity calculation variables
    float angle_prev; //!< angle in previous velocity calculation step
    long velocity_calc_timestamp; //!< last velocity calculation timestamp

};


#endif
