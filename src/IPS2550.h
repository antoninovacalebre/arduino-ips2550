#ifndef IPS2550_h
#define IPS2550_h

#include <Arduino.h>
#include <Wire.h>

#define CONFIG_WAIT_MS 10

enum VDD
{
    VDD_3V3,
    VDD_5V0
};

enum OutputMode
{
    DIFFERENTIAL,
    SINGLE_ENDED
};

const double GAIN_FACTORS[] = {2.0, 2.1, 2.18, 2.29, 2.38, 2.5, 2.59, 2.72, 2.83, 2.97,
                               3.09, 3.24, 3.36, 3.53, 3.67, 3.85, 4.0, 4.2, 4.36, 4.58,
                               4.76, 4.99, 5.19, 5.45, 5.66, 5.94, 6.17, 6.48, 6.73, 7.06,
                               7.34, 7.7, 8.0, 8.4, 8.72, 9.16, 9.51, 9.99, 10.38, 10.89,
                               11.31, 11.88, 12.34, 12.96, 13.46, 14.13, 14.67, 15.41, 16.0, 16.8,
                               17.45, 18.32, 19.02, 19.98, 20.75, 21.79, 22.62, 23.76, 24.68, 25.91,
                               26.91, 28.26, 29.34, 30.81, 32.0, 33.6, 34.9, 36.64, 38.05, 39.95,
                               41.5, 43.58, 45.25, 47.51, 49.36, 51.83, 53.82, 56.52, 58.69, 61.62,
                               64.0, 67.2, 69.79, 73.28, 76.1, 79.9, 83.01, 87.16, 90.5, 95.02,
                               98.72, 103.66, 107.65, 113.03, 117.38, 123.24};

class IPS2550
{
public:
    IPS2550(TwoWire &i2c, uint8_t i2c_addr);

    uint8_t m_i2c_addr;
    TwoWire *m_i2c = NULL;

private:
    uint16_t read_register(uint8_t reg_addr);
    uint16_t read_register_masked(uint8_t reg_addr, uint16_t mask);
    void write_register(uint8_t reg_addr, uint16_t value);
    void write_register_masked(uint8_t reg_addr, uint16_t value, uint16_t mask);

    void set_voltage(VDD vdd);
    void set_automatic_gain_control(boolean enabled);
    void set_master_gain_boost(boolean enabled);
    void set_master_gain_code(uint8_t code);
    void set_fine_gain_1(uint8_t code);
    void set_fine_gain_2(uint8_t code);
    void set_offset_1(int sign, uint8_t code);
    void set_offset_2(int sign, uint8_t code);
    void set_current_bias(uint8_t code);
    void set_output_mode(OutputMode om);

    double get_tx_frequency();
    VDD get_vdd();
    OutputMode get_output_mode();
    boolean get_automatic_gain_control();
    uint8_t get_master_gain_code();
    double get_master_gain();
    boolean get_master_gain_boost();
    uint8_t get_fine_gain_1_code();
    uint8_t get_fine_gain_2_code();
    double get_fine_gain_1();
    double get_fine_gain_2();
    int get_offset_sign_1();
    int get_offset_sign_2();
    uint8_t get_offset_code_1();
    uint8_t get_offset_code_2();
    double get_offset_perc_1();
    double get_offset_perc_2();
    double get_tx_current_bias_uA();

    double get_rx1();
    double get_rx2();
    double get_rx1_avg();
    double get_rx2_avg();

    double estimate_vtx_rms();
    double estimate_vtx_pp();
    double estimate_vtx();
};

unsigned bit_length(unsigned n);
unsigned first_bit_set(unsigned n);
unsigned crc(unsigned word, unsigned polynomial, unsigned filler = 0);

#endif
