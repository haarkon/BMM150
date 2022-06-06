/*
Christian Dupaty 03/2021
Library and demo for BMM150 see datasheet here : https://www.bosch-sensortec.com/products/motion-sensors/magnetometers-bmm150/
Adaptation of BOSCH driver https://github.com/BoschSensortec/BMM150-Sensor-API
for ARM MBED and tested on NUCLEO-L073RZ and GEOMAGNETIC CLICK
https://www.mikroe.com/geomagnetic-click 
*/
 
#include "mbed.h"
#include "BMM150.h"

#define DPI 6.283185307179586476925286766559
#define PI  3.1415926535897932384626433832795
 
BMM150::BMM150(PinName mosi, PinName miso, PinName sck, PinName cs, int frequency, int mode)
{
    _spi = new SPI(mosi, miso, sck);
    _cs = new DigitalOut(cs,1);
    
    //100KHz, as specified by the datasheet.
    _spi->frequency(frequency);
    _spi->format(8, mode);
}
 
uint8_t BMM150::initialize(int presetMode) 
{
    uint8_t check;
    
    /* Power up the sensor from suspend to sleep mode */
    //printf("Go To Sleep\n");
    suspend_to_sleep_mode();
    //ThisThread::sleep_for(3ms);
    //printf("sleep Mode\n");
 
    /* Function to update trim values */
    //printf("Read Registers\n");
    read_trim_registers();
    //printf("Trim read\n");
 
    /* Self Test Run */
    //printf("Running Tests\n");
    run_self_test();
    //printf("Tests done\n");
    
    /* Check Self Test */
    //printf("Validate Tests\n");
    check = validate_test_results();
    if (check!=BMM150_OK) return -1;
    //printf("Test validated\n");

    /* Setting the power mode as normal */
    //printf("Setting Mode\n");
    set_op_mode(BMM150_NORMAL_MODE);
    //printf ("Mode set\n");

    /*  Setting the preset mode
        i.e. data rate = 10Hz XY-rep = 1 Z-rep = 2*/
    //printf ("Setting Presets");
    set_presetmode(presetMode);
    //printf("preset OK\n");

    return BMM150_OK;
}

float BMM150::getBearing(){
    static float bearing = 360.0f;
    if (spi_read(BMM150_DATA_READY_STATUS)&0x01) bearing = computeBearing();
    return bearing;    
}

float BMM150::computeBearing (){

    double bearing;
    bmm150_mag_data value;
    
    /* Reading of magnetic fields */
    read_mag_data(&value);
    //printf("X=%d, Y=%d\n",value.x,value.y);
    bearing = atan2((double)value.x, (double)value.y);
    if (bearing < 0.0) bearing += DPI;
    if (bearing > DPI) bearing -= DPI;

    return (float)(bearing*180.0/PI);
}

void BMM150::read_mag_data(bmm150_mag_data *value) {
    
    int8_t reg_data[BMM150_XYZR_DATA_LEN] = {0};
    bmm150_raw_mag_data raw_mag_data;

    spi_read(BMM150_DATA_X_LSB, reg_data, BMM150_XYZR_DATA_LEN);
 
    /* Mag X axis data */
    raw_mag_data.raw_data_x = ((((short)reg_data[0]>>3)&0x1F) + ((short)reg_data[1]*32));
    /* Mag Y axis data */
    raw_mag_data.raw_data_y = ((((short)reg_data[2]>>3)&0x1F) + ((short)reg_data[3]*32));
    /* Mag Z axis data */
    raw_mag_data.raw_data_z = ((((short)reg_data[4]>>1)&0x7F) + ((short)reg_data[5]*128));
    /* Mag R-HALL data */
    raw_mag_data.raw_data_r = ((((short)reg_data[6]>>2)&0x3F) + ((short)reg_data[7]*64));
 
    printf("Xr = %d, Yr = %d, Zr = %d\t",raw_mag_data.raw_data_x,raw_mag_data.raw_data_y,raw_mag_data.raw_data_z);

    /* Compensated Mag X data in int16_t format */
    value->x = (int)compensate_x(raw_mag_data.raw_data_x, raw_mag_data.raw_data_r);
    /* Compensated Mag Y data in int16_t format */
    value->y = (int)compensate_y(raw_mag_data.raw_data_y, raw_mag_data.raw_data_r);
    /* Compensated Mag Z data in int16_t format */
    value->z = (int)compensate_z(raw_mag_data.raw_data_z, raw_mag_data.raw_data_r);
}
 
/*
    @brief This internal API is used to obtain the compensated
    magnetometer X axis data(micro-tesla) in int16_t.
*/
int16_t BMM150::compensate_x(int16_t mag_data_x, uint16_t data_rhall) 
{
    int16_t retval;
    uint16_t process_comp_x0 = 0;
    int32_t process_comp_x1;
    uint16_t process_comp_x2;
    int32_t process_comp_x3;
    int32_t process_comp_x4;
    int32_t process_comp_x5;
    int32_t process_comp_x6;
    int32_t process_comp_x7;
    int32_t process_comp_x8;
    int32_t process_comp_x9;
    int32_t process_comp_x10;
 
    /* Overflow condition check */
    if (mag_data_x != BMM150_XYAXES_FLIP_OVERFLOW_ADCVAL) {
        if (data_rhall != 0) {
            /* Availability of valid data*/
            process_comp_x0 = data_rhall;
        } else if (trim_data.dig_xyz1 != 0) {
            process_comp_x0 = trim_data.dig_xyz1;
        } else {
            process_comp_x0 = 0;
        }
        if (process_comp_x0 != 0) {
            /* Processing compensation equations*/
            process_comp_x1 = ((int32_t)trim_data.dig_xyz1) * 16384;
            process_comp_x2 = ((uint16_t)(process_comp_x1 / process_comp_x0)) - ((uint16_t)0x4000);
            retval = ((int16_t)process_comp_x2);
            process_comp_x3 = (((int32_t)retval) * ((int32_t)retval));
            process_comp_x4 = (((int32_t)trim_data.dig_xy2) * (process_comp_x3 / 128));
            process_comp_x5 = (int32_t)(((int16_t)trim_data.dig_xy1) * 128);
            process_comp_x6 = ((int32_t)retval) * process_comp_x5;
            process_comp_x7 = (((process_comp_x4 + process_comp_x6) / 512) + ((int32_t)0x100000));
            process_comp_x8 = ((int32_t)(((int16_t)trim_data.dig_x2) + ((int16_t)0xA0)));
            process_comp_x9 = ((process_comp_x7 * process_comp_x8) / 4096);
            process_comp_x10 = ((int32_t)mag_data_x) * process_comp_x9;
            retval = ((int16_t)(process_comp_x10 / 8192));
            retval = (retval + (((int16_t)trim_data.dig_x1) * 8)) / 16;
        } else {
            retval = BMM150_OVERFLOW_OUTPUT;
        }
    } else {
        /* Overflow condition */
        retval = BMM150_OVERFLOW_OUTPUT;
    }
 
    return retval;
}
 
/*
    @brief This internal API is used to obtain the compensated
    magnetometer Y axis data(micro-tesla) in int16_t.
*/
int16_t BMM150::compensate_y(int16_t mag_data_y, uint16_t data_rhall) {
    int16_t retval;
    uint16_t process_comp_y0 = 0;
    int32_t process_comp_y1;
    uint16_t process_comp_y2;
    int32_t process_comp_y3;
    int32_t process_comp_y4;
    int32_t process_comp_y5;
    int32_t process_comp_y6;
    int32_t process_comp_y7;
    int32_t process_comp_y8;
    int32_t process_comp_y9;
 
    /* Overflow condition check */
    if (mag_data_y != BMM150_XYAXES_FLIP_OVERFLOW_ADCVAL) {
        if (data_rhall != 0) {
            /* Availability of valid data*/
            process_comp_y0 = data_rhall;
        } else if (trim_data.dig_xyz1 != 0) {
            process_comp_y0 = trim_data.dig_xyz1;
        } else {
            process_comp_y0 = 0;
        }
        if (process_comp_y0 != 0) {
            /*Processing compensation equations*/
            process_comp_y1 = (((int32_t)trim_data.dig_xyz1) * 16384) / process_comp_y0;
            process_comp_y2 = ((uint16_t)process_comp_y1) - ((uint16_t)0x4000);
            retval = ((int16_t)process_comp_y2);
            process_comp_y3 = ((int32_t) retval) * ((int32_t)retval);
            process_comp_y4 = ((int32_t)trim_data.dig_xy2) * (process_comp_y3 / 128);
            process_comp_y5 = ((int32_t)(((int16_t)trim_data.dig_xy1) * 128));
            process_comp_y6 = ((process_comp_y4 + (((int32_t)retval) * process_comp_y5)) / 512);
            process_comp_y7 = ((int32_t)(((int16_t)trim_data.dig_y2) + ((int16_t)0xA0)));
            process_comp_y8 = (((process_comp_y6 + ((int32_t)0x100000)) * process_comp_y7) / 4096);
            process_comp_y9 = (((int32_t)mag_data_y) * process_comp_y8);
            retval = (int16_t)(process_comp_y9 / 8192);
            retval = (retval + (((int16_t)trim_data.dig_y1) * 8)) / 16;
        } else {
            retval = BMM150_OVERFLOW_OUTPUT;
        }
    } else {
        /* Overflow condition*/
        retval = BMM150_OVERFLOW_OUTPUT;
    }
 
    return retval;
}
 
/*
    @brief This internal API is used to obtain the compensated
    magnetometer Z axis data(micro-tesla) in int16_t.
*/
int16_t BMM150::compensate_z(int16_t mag_data_z, uint16_t data_rhall) {
    int32_t retval;
    int16_t process_comp_z0;
    int32_t process_comp_z1;
    int32_t process_comp_z2;
    int32_t process_comp_z3;
    int16_t process_comp_z4;
 
    if (mag_data_z != BMM150_ZAXIS_HALL_OVERFLOW_ADCVAL) {
        if ((trim_data.dig_z2 != 0) && (trim_data.dig_z1 != 0)
                && (data_rhall != 0) && (trim_data.dig_xyz1 != 0)) {
            /*Processing compensation equations*/
            process_comp_z0 = ((int16_t)data_rhall) - ((int16_t) trim_data.dig_xyz1);
            process_comp_z1 = (((int32_t)trim_data.dig_z3) * ((int32_t)(process_comp_z0))) / 4;
            process_comp_z2 = (((int32_t)(mag_data_z - trim_data.dig_z4)) * 32768);
            process_comp_z3 = ((int32_t)trim_data.dig_z1) * (((int16_t)data_rhall) * 2);
            process_comp_z4 = (int16_t)((process_comp_z3 + (32768)) / 65536);
            retval = ((process_comp_z2 - process_comp_z1) / (trim_data.dig_z2 + process_comp_z4));
 
            /* saturate result to +/- 2 micro-tesla */
            if (retval > BMM150_POSITIVE_SATURATION_Z) {
                retval =  BMM150_POSITIVE_SATURATION_Z;
            } else {
                if (retval < BMM150_NEGATIVE_SATURATION_Z) {
                    retval = BMM150_NEGATIVE_SATURATION_Z;
                }
            }
            /* Conversion of LSB to micro-tesla*/
            retval = retval / 16;
        } else {
            retval = BMM150_OVERFLOW_OUTPUT;
 
        }
    } else {
        /* Overflow condition*/
        retval = BMM150_OVERFLOW_OUTPUT;
    }
 
    return (int16_t)retval;
}
 
void BMM150::set_presetmode(uint8_t preset_mode) {
    switch (preset_mode) {
        case BMM150_PRESETMODE_LOWPOWER:
            /*  Set the data rate x,y,z repetition
                for Low Power mode */
            settings.data_rate = BMM150_DATA_RATE_10HZ;
            settings.xy_rep = BMM150_LOWPOWER_REPXY;
            settings.z_rep = BMM150_LOWPOWER_REPZ;
            set_odr_xyz_rep(settings);
            break;
        case BMM150_PRESETMODE_REGULAR:
            /*  Set the data rate x,y,z repetition
                for Regular mode */
            settings.data_rate = BMM150_DATA_RATE_10HZ;
            settings.xy_rep = BMM150_REGULAR_REPXY;
            settings.z_rep = BMM150_REGULAR_REPZ;
            set_odr_xyz_rep(settings);
            break;
        case BMM150_PRESETMODE_HIGHACCURACY:
            /*  Set the data rate x,y,z repetition
                for High Accuracy mode */
            settings.data_rate = BMM150_DATA_RATE_20HZ;
            settings.xy_rep = BMM150_HIGHACCURACY_REPXY;
            settings.z_rep = BMM150_HIGHACCURACY_REPZ;
            set_odr_xyz_rep(settings);
            break;
        case BMM150_PRESETMODE_ENHANCED:
            /*  Set the data rate x,y,z repetition
                for Enhanced Accuracy mode */
            settings.data_rate = BMM150_DATA_RATE_10HZ;
            settings.xy_rep = BMM150_ENHANCED_REPXY;
            settings.z_rep = BMM150_ENHANCED_REPZ;
            set_odr_xyz_rep(settings);
            break;
        default:
            break;
    }
}
 
void BMM150::set_odr_xyz_rep(struct bmm150_settings settings) {
    /* Set the ODR */
    set_odr(settings);
    /* Set the XY-repetitions number */
    set_xy_rep(settings);
    /* Set the Z-repetitions number */
    set_z_rep(settings);
}
 
void BMM150::set_xy_rep(struct bmm150_settings settings) {
    uint8_t rep_xy;
    rep_xy = settings.xy_rep;
    spi_write(BMM150_REP_XY_ADDR, rep_xy);
 
}
 
void BMM150::set_z_rep(struct bmm150_settings settings) {
    uint8_t rep_z;
    rep_z = settings.z_rep;
    spi_write(BMM150_REP_Z_ADDR, rep_z);
}
 
 
void BMM150::soft_reset() 
{
    uint8_t reg_data;
 
    reg_data = spi_read(BMM150_POWER_CONTROL_ADDR);
    reg_data = reg_data | BMM150_SET_SOFT_RESET;
    spi_write(BMM150_POWER_CONTROL_ADDR, reg_data);
    while ((spi_read(BMM150_POWER_CONTROL_ADDR)&BMM150_SET_SOFT_RESET)!=0); //blocking !
}
 
 
void BMM150::set_odr(struct bmm150_settings settings) 
{
    uint8_t reg_data;
 
    reg_data = spi_read(BMM150_OP_MODE_ADDR);
    /*Set the ODR value */
    reg_data = BMM150_SET_BITS(reg_data, BMM150_ODR, settings.data_rate);
    spi_write(BMM150_OP_MODE_ADDR, reg_data);
}
 
void BMM150::spi_write(short address, short data) 
{
    char temp[2],dummy[2];
    temp[0]=address;
    temp[1]=data;
    _cs->write(0);
    _spi->write(temp, 2, dummy, 0);
    _cs->write(1);
}
 
 
void BMM150::spi_read(short address, uint8_t* buffer, short length) 
{
    char temp[1],i;
    char *dummy = (char*) malloc (length+1);
    temp[0]=(char)address+0x80;
    _cs->write(0);
    _spi->write(temp, 1, dummy, length+1);
    _cs->write(1);
    for(i=0;i<length;i++) *(buffer+i)=(uint8_t)*(dummy+i+1);
    free (dummy);
}
 
 
void BMM150::spi_read(short address, int8_t* buffer, short length) 
{
    char temp[1],i;
    char *dummy = (char*) malloc(length+1);
    temp[0]=(char)address+0x80;
    _cs->write(0);
    _spi->write(temp,1,dummy,length+1);
    _cs->write(1);
    for(i=0;i<length;i++) *(buffer+i)=(int8_t)*(dummy+i+1);
    free (dummy);
}
 
uint8_t BMM150::spi_read(short address) 
{
    char temp[1], dummy[2];
    temp[0]=address+0x80;
    _cs->write(0);
    _spi->write(temp,1,dummy,2);
    _cs->write(1);
    return dummy[1];
}
 
void BMM150::set_op_mode(uint8_t pwr_mode) {
    /* Select the power mode to set */
    switch (pwr_mode) {
        case BMM150_NORMAL_MODE:
            /*  If the sensor is in suspend mode
                put the device to sleep mode */
            suspend_to_sleep_mode();
            /* write the op mode */
            write_op_mode(pwr_mode);
            break;
        case BMM150_FORCED_MODE:
            /*  If the sensor is in suspend mode
                put the device to sleep mode */
            suspend_to_sleep_mode();
            /* write the op mode */
            write_op_mode(pwr_mode);
            break;
        case BMM150_SLEEP_MODE:
            /*  If the sensor is in suspend mode
                put the device to sleep mode */
            suspend_to_sleep_mode();
            /* write the op mode */
            write_op_mode(pwr_mode);
            break;
        case BMM150_SUSPEND_MODE:
            /* Set the power control bit to zero */
            set_power_control_bit(BMM150_POWER_CNTRL_DISABLE);
            break;
        default:
            break;
    }
}
 
void BMM150::suspend_to_sleep_mode(void) {
    set_power_control_bit(BMM150_POWER_CNTRL_ENABLE);
    /* waiting for a the chip ID - ie : means that boot sequence complete */
    while (spi_read(BMM150_CHIP_ID_ADDR) != BMM150_CHIP_ID); //blocking about 3ms
}
 
 
void BMM150::read_trim_registers() {
    uint8_t trim_x1y1[2] = {0};
    uint8_t trim_xyz_data[4] = {0};
    uint8_t trim_xy1xy2[10] = {0};
    uint16_t temp_msb = 0;
 
    /* Trim register value is read */
    spi_read(BMM150_DIG_X1, trim_x1y1, 2);
    spi_read(BMM150_DIG_Z4_LSB, trim_xyz_data, 4);
    spi_read(BMM150_DIG_Z2_LSB, trim_xy1xy2, 10);
    /*  Trim data which is read is updated
        in the device structure */
    trim_data.dig_x1 = (char)trim_x1y1[0];
    trim_data.dig_y1 = (char)trim_x1y1[1];
    trim_data.dig_x2 = (char)trim_xyz_data[2];
    trim_data.dig_y2 = (char)trim_xyz_data[3];
    temp_msb = ((uint16_t)trim_xy1xy2[3]) << 8;
    trim_data.dig_z1 = (uint16_t)(temp_msb | trim_xy1xy2[2]);
    temp_msb = ((uint16_t)trim_xy1xy2[1]) << 8;
    trim_data.dig_z2 = (int16_t)(temp_msb | trim_xy1xy2[0]);
    temp_msb = ((uint16_t)trim_xy1xy2[7]) << 8;
    trim_data.dig_z3 = (int16_t)(temp_msb | trim_xy1xy2[6]);
    temp_msb = ((uint16_t)trim_xyz_data[1]) << 8;
    trim_data.dig_z4 = (int16_t)(temp_msb | trim_xyz_data[0]);
    trim_data.dig_xy1 = trim_xy1xy2[9];
    trim_data.dig_xy2 = (char)trim_xy1xy2[8];
    temp_msb = ((uint16_t)(trim_xy1xy2[5] & 0x7F)) << 8;
    trim_data.dig_xyz1 = (uint16_t)(temp_msb | trim_xy1xy2[4]);
 
}
 
void BMM150::write_op_mode(uint8_t op_mode) {
    uint8_t reg_data = 0;
    reg_data = spi_read(BMM150_OP_MODE_ADDR);
    /* Set the op_mode value in Opmode bits of 0x4C */
    reg_data = BMM150_SET_BITS(reg_data, BMM150_OP_MODE, op_mode);
    spi_write(BMM150_OP_MODE_ADDR, reg_data);
}
 
void BMM150::set_power_control_bit(uint8_t pwrcntrl_bit) {
    uint8_t reg_data = 0;
    /* Power control register 0x4B is read */
    reg_data = spi_read(BMM150_POWER_CONTROL_ADDR);
    if ((reg_data&0x01)==0) {
        /* Sets the value of power control bit */
        reg_data = BMM150_SET_BITS_POS_0(reg_data, BMM150_PWR_CNTRL, pwrcntrl_bit);
        spi_write(BMM150_POWER_CONTROL_ADDR, reg_data);
    }
}
 
void BMM150::activate_X_axis() {
     uint8_t reg_data = 0;
    /* Power control register 0x4B is read */
    reg_data = spi_read(BMM150_AXES_ENABLE_ADDR);
    /* Sets the value of power control bit */
    reg_data = reg_data & ~0x08;
    spi_write(BMM150_AXES_ENABLE_ADDR, reg_data);
}

void BMM150::desactivate_X_axis() {
     uint8_t reg_data = 0;
    /* Power control register 0x4B is read */
    reg_data = spi_read(BMM150_AXES_ENABLE_ADDR);
    /* Sets the value of power control bit */
    reg_data = reg_data | 0x08;
    spi_write(BMM150_AXES_ENABLE_ADDR, reg_data);
}

void BMM150::activate_Y_axis() {
     uint8_t reg_data = 0;
    /* Power control register 0x4B is read */
    reg_data = spi_read(BMM150_AXES_ENABLE_ADDR);
    /* Sets the value of power control bit */
    reg_data = reg_data & ~0x10;
    spi_write(BMM150_AXES_ENABLE_ADDR, reg_data);
}

void BMM150::desactivate_Y_axis() {
     uint8_t reg_data = 0;
    /* Power control register 0x4B is read */
    reg_data = spi_read(BMM150_AXES_ENABLE_ADDR);
    /* Sets the value of power control bit */
    reg_data = reg_data | 0x10;
    spi_write(BMM150_AXES_ENABLE_ADDR, reg_data);
}

void BMM150::activate_Z_axis() {
     uint8_t reg_data = 0;
    /* Power control register 0x4B is read */
    reg_data = spi_read(BMM150_AXES_ENABLE_ADDR);
    /* Sets the value of power control bit */
    reg_data = reg_data & ~0x20;
    spi_write(BMM150_AXES_ENABLE_ADDR, reg_data);
}

void BMM150::desactivate_Z_axis() {
     uint8_t reg_data = 0;
    /* Power control register 0x4B is read */
    reg_data = spi_read(BMM150_AXES_ENABLE_ADDR);
    /* Sets the value of power control bit */
    reg_data = reg_data | 0x20;
    spi_write(BMM150_AXES_ENABLE_ADDR, reg_data);
}

void BMM150::run_self_test(){
    uint8_t reg_data = 0;
    /* Power control register 0x4B is read */
    reg_data = spi_read(BMM150_OP_MODE_ADDR);
    /* Sets the value of power control bit */
    reg_data = reg_data | 0x01;
    spi_write(BMM150_OP_MODE_ADDR, reg_data);
    while ((spi_read(BMM150_OP_MODE_ADDR)&0x01)!=0x01); //blocking
}

uint8_t BMM150::validate_test_results() {

    uint8_t reg_data[BMM150_XYZR_DATA_LEN];
    spi_read(BMM150_DATA_X_LSB, reg_data, BMM150_XYZR_DATA_LEN);
    if ((reg_data[0]&0x01)&&(reg_data[2]&0x01)&&(reg_data[4]&0x01)) return BMM150_OK;
    else return -1;
}