#include "Arduino.h"
#include "IPS2550.h"

uint64_t bit_length(uint64_t n)
{
    uint64_t i = 1;
    while ((n >> i) != 0)
        i++;
    return i;
}

uint64_t first_bit_set(uint64_t n)
{
    if (n == 0)
        return 0;

    uint64_t i = 0;
    while (((n >> i) & 0b1) != 1)
        i++;
    return i;
}

uint64_t crc(uint64_t word, uint64_t polynomial, uint64_t filler)
{
    uint64_t g = bit_length(polynomial);
    uint64_t n = g - 1;
    word = (word << n) | filler;
    while ((word >> n) != 0)
    {
        uint64_t first_one = bit_length(word);
        uint64_t xor_mask = ~(0xFF << g) << (first_one - g);
        uint64_t xor_val = polynomial << (first_one - g);
        word = ((word & xor_mask) ^ xor_val) | (word & ~xor_mask);
    }

    return word & ~(0xFF << g);
}

void IPS2550::init(TwoWire &i2c, uint8_t i2c_addr)
{
    m_i2c = &i2c;
    m_i2c_addr = i2c_addr;
}

int IPS2550::read_register(uint8_t reg_addr)
{
    m_i2c->beginTransmission(m_i2c_addr);
    m_i2c->write(reg_addr);
    m_i2c->endTransmission();

    m_i2c->requestFrom(m_i2c_addr, (uint8_t)2);
    // while (!m_i2c->available())
    // {
    // }
    uint16_t data = m_i2c->read() << 8;
    data = data | m_i2c->read();
    uint8_t crc_bits = data & 0x0007;
    uint16_t reg = data >> 5;

    uint64_t message = ((reg << 5) & 0xFF00) | (reg & 0x0007);
    if (crc(message, 0b1011, crc_bits) != 0)
    {
        Serial.println("Read failed");
        return -1;
    }

    return reg;
}

int IPS2550::read_register_masked(uint8_t reg_addr, uint16_t mask)
{
    int reg = read_register(reg_addr);
    if (reg < 0)
        return -1;

    return (reg & mask) >> first_bit_set(mask);
}

void IPS2550::write_register(uint32_t reg_addr, uint32_t value)
{
    uint32_t crc_in = ((reg_addr & 0x007F) << 17) | ((value & 0x07F8) << 5) | (value & 0x0007);
    uint8_t crc_bits = crc(crc_in, 0b1011, 0);
    uint16_t message = (value << 5) | 0x18 | crc_bits;

    Serial.println("ADSADASD");
    Serial.println(reg_addr, BIN);
    Serial.println(value, BIN);
    Serial.println(crc_bits, BIN);
    Serial.println(crc_in, BIN);

    m_i2c->beginTransmission(m_i2c_addr);
    m_i2c->write(reg_addr);
    m_i2c->endTransmission();

    m_i2c->beginTransmission(m_i2c_addr);
    m_i2c->write(message >> 8);
    m_i2c->write(message & 0x00FF);
    m_i2c->endTransmission();
}

void IPS2550::write_register_masked(uint8_t reg_addr, uint16_t value, uint16_t mask)
{
    int reg = read_register(reg_addr);
    if (reg < 0)
        return;

    uint16_t ureg = (uint16_t)reg;

    uint16_t new_word = (ureg & ~mask) | (value & mask);

    write_register(reg_addr, new_word);
}

void IPS2550::set_voltage(VDD vdd)
{
    write_register_masked(0x41, vdd, 0b1);
    write_register_masked(0x01, vdd, 0b1);
    delay(CONFIG_WAIT_MS);
}

void IPS2550::set_automatic_gain_control(boolean enabled)
{
    uint8_t val = enabled ? 0 : 1;

    write_register_masked(0x40, val << 9, 0b1 << 9);
    write_register_masked(0x00, val << 9, 0b1 << 9);
    delay(CONFIG_WAIT_MS);
}

void IPS2550::set_master_gain_boost(boolean enabled)
{
    uint8_t val = enabled ? 1 : 0;

    write_register_masked(0x42, val << 7, 0b1 << 7);
    write_register_masked(0x02, val << 7, 0b1 << 7);
    delay(CONFIG_WAIT_MS);
}

void IPS2550::set_master_gain_code(uint8_t code)
{
    code = code <= 95 ? code : 95;

    // write_register_masked(0x42, code, 0x007F);
    write_register_masked(0x02, code, 0x007F);
    delay(CONFIG_WAIT_MS);
}

void IPS2550::set_fine_gain_1(uint8_t code)
{
    code = code <= 0x7F ? code : 0x7F;

    write_register_masked(0x43, code, 0x007F);
    write_register_masked(0x03, code, 0x007F);
    delay(CONFIG_WAIT_MS);
}

void IPS2550::set_fine_gain_2(uint8_t code)
{
    code = code <= 0x7F ? code : 0x7F;

    write_register_masked(0x45, code, 0x007F);
    write_register_masked(0x05, code, 0x007F);
    delay(CONFIG_WAIT_MS);
}

void IPS2550::set_offset_1(int sign, uint8_t code)
{
    code = code <= 0x7F ? code : 0x7F;
    uint8_t sign_code = (sign < 0 ? 1 : 0) << 7;

    write_register_masked(0x44, sign_code | code, 0x00FF);
    write_register_masked(0x04, sign_code | code, 0x00FF);
    delay(CONFIG_WAIT_MS);
}

void IPS2550::set_offset_2(int sign, uint8_t code)
{
    code = code <= 0x7F ? code : 0x7F;
    uint8_t sign_code = (sign < 0 ? 1 : 0) << 7;

    write_register_masked(0x46, sign_code | code, 0x00FF);
    write_register_masked(0x06, sign_code | code, 0x00FF);
    delay(CONFIG_WAIT_MS);
}

void IPS2550::set_current_bias(uint8_t code)
{
    write_register_masked(0x47, code, 0x00FF);
    write_register_masked(0x07, code, 0x00FF);
    delay(CONFIG_WAIT_MS);
}

void IPS2550::set_output_mode(OutputMode om)
{
    write_register_masked(0x40, om << 1, 0b1 << 1);
    write_register_masked(0x00, om << 1, 0b1 << 1);
    delay(CONFIG_WAIT_MS);
}

double IPS2550::get_tx_frequency()
{
    return read_register_masked(0x6E, 0x07FF) * 20000.0;
}

VDD IPS2550::get_vdd()
{
    uint16_t code = read_register_masked(0x01, 0b1);
    return code == 0 ? VDD_3V3 : VDD_5V0;
}

OutputMode IPS2550::get_output_mode()
{
    uint16_t code = read_register_masked(0x01, 0b1 << 1);
    return code == 0 ? DIFFERENTIAL : SINGLE_ENDED;
}

boolean IPS2550::get_automatic_gain_control()
{
    return read_register_masked(0x00, 0b1 << 9) == 0;
}

uint8_t IPS2550::get_master_gain_code()
{
    return read_register_masked(0x02, 0x007F);
}

double IPS2550::get_master_gain()
{
    return GAIN_FACTORS[get_master_gain_code()];
}

boolean IPS2550::get_master_gain_boost()
{
    return read_register_masked(0x02, 0b1 << 7) == 1;
}

uint8_t IPS2550::get_fine_gain_1_code()
{
    return read_register_masked(0x03, 0x007F);
}

uint8_t IPS2550::get_fine_gain_2_code()
{
    return read_register_masked(0x05, 0x007F);
}

double IPS2550::get_fine_gain_1()
{
    return 1.0 + get_fine_gain_1_code() * 0.125 / 100.0;
}

double IPS2550::get_fine_gain_2()
{
    return 1.0 + get_fine_gain_2_code() * 0.125 / 100.0;
}

int IPS2550::get_offset_sign_1()
{
    return read_register_masked(0x04, 0b1 << 7) == 0 ? 1 : -1;
}

int IPS2550::get_offset_sign_2()
{
    return read_register_masked(0x06, 0b1 << 7) == 0 ? 1 : -1;
}

uint8_t IPS2550::get_offset_code_1()
{
    return read_register_masked(0x04, 0x007F);
}

uint8_t IPS2550::get_offset_code_2()
{
    return read_register_masked(0x06, 0x007F);
}

double IPS2550::get_offset_perc_1()
{
    return get_offset_sign_1() * get_offset_code_1() * 0.0015 / 100.0;
}

double IPS2550::get_offset_perc_2()
{
    return get_offset_sign_2() * get_offset_code_2() * 0.0015 / 100.0;
}

double IPS2550::get_tx_current_bias_uA()
{
    uint16_t code = read_register_masked(0x07, 0x00FF);
    double multiplier = pow(2, (code >> 6) * 2);
    double base = code & 0x003F;
    return multiplier * base * 0.5;
}
