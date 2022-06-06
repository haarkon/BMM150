/**
 * @author Hugues Angelis, based on Christian Dupaty library (thanks to him)
 *
 * @section LICENSE
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 *
 * @section DESCRIPTION
 *
 * BMM150 - 3 Axis magnetometer
 * 
 * Datasheet : https://www.bosch-sensortec.com/products/motion-sensors/magnetometers-bmm150/
 *
 * This library is made from BOSCH Drivers : https://github.com/BoschSensortec/BMM150-Sensor-API
 * This version is Mbed-os 6 compatible, using SPI interface to communicate with Mikroe MikroBus GEOMAGNETIC CLICK :
 * https://www.mikroe.com/geomagnetic-click
 */
 
#ifndef _BMM150_H_
#define _BMM150_H_
 
 
/** Includes */
#include "mbed.h"
#include "BMM150_defs.h"
 
class BMM150 {
 
  public:
/**
 * \class BMM150 : BMM150.h
 * BMM150 : 3 Axis magnetic sensor with 0.6µT sensibility - Using SPI interface - non blocking
 * \brief More informations at https://www.bosch-sensortec.com/products/motion-sensors/magnetometers-bmm150/
 * \note We use a mikroBus Geomagnetic Click board as a compass. The card data are available at https://www.mikroe.com/geomagnetic-click
 * \note As the Dataready pin of the BMM150 is left unconnected on the mikroBus card, we use a preset that allows BMM150 to perform continuously the measurment of each axis
 *
 * \code
 * #include "mbed.h"
 * #include "rtos.h"
 * #include "BMM150.h"
 * 
 * BMM150 compass (PE_6,PE_5,PE_2,PE_4);
 * 
 * int main()
 * {
 *     float bearing;
 *     int cr;
 * 
 *     printf("Boot Up - NUCLEO\n");
 * 
 *     cr = compass.initialize(BMM150_PRESETMODE_HIGHACCURACY);
 *     if (cr!=0) printf("Initialisation Error\n");
 *     else printf("Initialisation complete\n");
 * 
 *     while(true) {
 *         bearing = compass.getBearing();
 *         printf ("heading = %f\n",bearing);
 *         ThisThread::sleep_for(25ms); //As data are refreshed at 20 Hz (50ms), you will read 2 time the same value   
 *     }
 * }
 * \endcode
 */
 
/**
 * Constructor of BMM150 SPI object.
 * @brief Constructor of the mikroBus geomagnetic click board used as compass, using the SPI bus interface
 * @param mosi : the Mbed pin (PinName) used as Master Out - Slave In
 * @param miso : the Mbed pin (PinName) used as Master In - Slave Out
 * @param sck  : the Mbed pin (PinName) used as Serial ClocK
 * @param cs   : the Mbed pin (PinName) used as Chip Select
 * @param frequency : the maximum allowed bitrate of the SPI bus (default value is 10 Mbit/s)
 * @param mode : the SPI bus mode (should be 0 ou 3, default valus is 0)
 */
 
 BMM150(PinName mosi, PinName miso, PinName sck, PinName cs, int frequency = 10000000, int mode = 0);
    
/**
 * @brief initialze the device
 * @param presetMode : use table below to choose the preset mode
 *
 * |               | ODR  | Noise | Profil Name                    | nXY | nZ |
 * |:--------------|:----:|:-----:|:------------------------------:|:---:|:--:|
 * | Low Power     | 10Hz | 1µT   | BMM150_PRESETMODE_LOWPOWER     | 3   | 3  |
 * | Regular       | 10Hz | 0.6µT | BMM150_PRESETMODE_REGULAR      | 9   | 15 |
 * | Enhanced      | 10Hz | 0.5µT | BMM150_PRESETMODE_ENHANCED     | 15  | 27 |
 * | High Accuracy | 20Hz | 0.3µT | BMM150_PRESETMODE_HIGHACCURACY | 47  | 83 |
 *
 * @return error code : 0 = initialisation done, 1 = intialisation failed
 */
    uint8_t initialize(int presetMode);
 
/**
 * @brief get the last valid value of the bearing in degre
 * @return angle in degre (trigonometric)
 * @note return only the last valid value (ie : no error) of the bearing  
 */
    float getBearing();
 
protected :

    /**
        \brief Read magnetometer data
    */
    void read_mag_data(bmm150_mag_data *value);
 
    /**
        @brief This internal API is used to obtain the compensated
        magnetometer x axis data(micro-tesla) in float.
    */
    int16_t compensate_x(int16_t mag_data_z, uint16_t data_rhall);
 
    /**
        @brief This internal API is used to obtain the compensated
        magnetometer Y axis data(micro-tesla) in int.
    */
    int16_t compensate_y(int16_t mag_data_z, uint16_t data_rhall);
 
    /**
        @brief This internal API is used to obtain the compensated
        magnetometer Z axis data(micro-tesla) in int.
    */
    int16_t compensate_z(int16_t mag_data_z, uint16_t data_rhall);
 
    /**
        \brief Set power mode
    */
    void set_op_mode(uint8_t op_mode);
 
    /**
        @brief This internal API reads the trim registers of the sensor and stores
        the trim values in the "trim_data" of device structure.
    */
    void read_trim_registers();
 
    /**
        @brief This internal API writes the op_mode value in the Opmode bits
        (bits 1 and 2) of 0x4C register.
    */
    void write_op_mode(uint8_t op_mode);
 
    /**
        \brief Set preset mode mode
    */
    void set_preset_mode(uint8_t mode);
 
    /**
        @brief This internal API sets/resets the power control bit of 0x4B register.
    */
    void set_power_control_bit(uint8_t pwrcntrl_bit);
 
    /**
        @brief This internal API sets the device from suspend to sleep mode
        by setting the power control bit to '1' of 0x4B register
    */
    void suspend_to_sleep_mode();
 
    /**
        @brief This API is used to set the preset mode of the sensor.
    */
    void set_presetmode(uint8_t preset_mode);
 
    /**
        Self test functionality
    */
    /*
        int8_t perform_self_test(uint8_t self_test_mode);
        int8_t perform_normal_self_test();
        void enable_normal_self_test(uint8_t *self_test_enable);
        int8_t validate_normal_self_test();
        int8_t perform_adv_self_test();
        void adv_self_test_settings();
        void adv_self_test_measurement(uint8_t self_test_current, int16_t *data_z);
        int8_t validate_adv_self_test(int16_t positive_data_z, int16_t negative_data_z);
        void set_adv_self_test_current(uint8_t self_test_current);
        void set_control_measurement_xyz(struct bmm150_settings settings);
    */
 
    /**
        @brief This internal API sets the preset mode ODR and repetition settings.
    */
    void set_odr_xyz_rep(struct bmm150_settings settings);
 
    /**
        @brief This internal API sets the xy repetition value in the 0x51 register.
    */
    void set_xy_rep(struct bmm150_settings settings);
 
    /**
        @brief This internal API sets the z repetition value in the 0x52 register.
    */
    void set_z_rep(struct bmm150_settings settings);
 
    /**
        @brief This internal API is used to set the output data rate of the sensor.
    */
    void set_odr(struct bmm150_settings settings);
 
    /**
        @brief This API is used to perform soft-reset of the sensor
        where all the registers are reset to their default values except 0x4B.
    */
    void soft_reset();

    /**
        @brief This API is used to perform soft-reset of the sensor
        where all the registers are reset to their default values except 0x4B.
    */
    void run_self_test();

    /**
        @brief This API is used to perform soft-reset of the sensor
        where all the registers are reset to their default values except 0x4B.
    */
    uint8_t validate_test_results();

    /**
        @brief This API is used to perform soft-reset of the sensor
        where all the registers are reset to their default values except 0x4B.
    */
    float computeBearing();

    void activate_X_axis();
    void desactivate_X_axis();
    void activate_Y_axis();
    void desactivate_Y_axis();
    void activate_Z_axis();
    void desactivate_Z_axis();

    struct bmm150_settings settings;
    struct bmm150_raw_mag_data raw_mag_data;
    struct bmm150_mag_data mag_data;
    struct bmm150_trim_registers trim_data;
 
    void spi_write(short address, short byte);
    //void spi_write(short address) ;
    void spi_read(short address, uint8_t* buffer, short length);
    void spi_read(short address, int8_t* buffer, short length);
    uint8_t spi_read(short address);
 
    private:
    SPI *_spi;
    DigitalOut *_cs;
};
 
 
#endif