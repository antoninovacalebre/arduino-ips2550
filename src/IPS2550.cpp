#include "Arduino.h"
#include "IPS2550.h"

uint32_t bit_length(uint32_t n)
{
    uint32_t i = 1;
    while ((n >> i) != 0)
        i++;
    return i;
}

uint32_t first_bit_set(uint32_t n)
{
    if (n == 0)
        return 0;

    uint32_t i = 0;
    while (((n & (1 << i)) >> i) != 0)
        i++;
    return i;
}

uint16_t crc(uint32_t word, uint8_t polynomial, uint8_t filler)
{
    uint16_t g = bit_length(polynomial);
    uint16_t n = g - 1;
    uint16_t word = (word << n) | filler;
    while ((word >> n) != 0)
    {
        uint16_t first_one = bit_length(word);
        uint16_t xor_mask = ~(0xFF << g) << (first_one - g);
        uint16_t xor_val = polynomial << (first_one - g);
        word = ((word & xor_mask) ^ xor_val) | (word & ~xor_mask);
    }

    return word & ~(0xFF << g);
}

uint16_t get_bits_in_word(uint16_t word, uint16_t read_mask)
{
    return (word & read_mask) >> first_bit_set(read_mask);
}

IPS2550::IPS2550(TwoWire &i2c, uint8_t i2c_addr)
{
    m_i2c = &i2c;
    m_i2c_addr = i2c_addr;
}

uint16_t IPS2550::read_reg(uint8_t reg_addr)
{
    m_i2c->beginTransmission(m_i2c_addr);
    m_i2c->write(reg_addr);
    m_i2c->endTransmission();

    m_i2c->requestFrom(m_i2c_addr, 2);
    uint16_t reg = m_i2c->read() << 8;
    reg = reg | m_i2c->read();
    uint8_t crc_bits = reg & 0b111;
    reg = reg >> 5;

    uint16_t message = ((reg << 5) & 0xFF00) | (reg & 0b111);
    if (crc(message, 0b1011, crc_bits) != 0)
        return 0x00;

    return reg;
}

void IPS2550::write_reg(uint8_t reg_addr, uint16_t value)
{
    uint32_t crc_in = (((reg_addr & 0x007F) << 1) << 16) | (((value & 0x07F8) >> 3) << 8) | (value & 0x0007);
    uint16_t crc_bits = crc(crc_in, 0b1011);
    uint16_t message = (value << 5) | 0x18 | crc_bits;

    m_i2c->beginTransmission(m_i2c_addr);
    m_i2c->write(reg_addr);
    m_i2c->write(message >> 8);
    m_i2c->write(message & 0x00FF);
    m_i2c->endTransmission();
}

void IPS2550::write_register_bits(uint8_t reg_addr, uint16_t clear_mask, uint16_t set_mask)
{
    uint16_t reg = read_reg(reg_addr);

    uint16_t new_word = (reg & ~clear_mask) | (set_mask & clear_mask);

    write_reg(reg_addr, new_word);
}

void IPS2550::set_voltage(VDD vdd)
{
    write_register_bits(0x41, 0x0001, vdd);
    delay(50);
    write_register_bits(0x01, 0x0001, vdd);
    delay(50);
}

void IPS2550::set_automatic_gain_control(boolean enabled)
{
    uint16_t val = enabled ? 0 : 1;

    write_register_bits(0x00, 0x0200, val << 9);
    delay(50);
    write_register_bits(0x40, 0x0200, val << 9);
    delay(50);
}

void IPS2550::set_master_gain_boost(boolean enabled)
{
    uint16_t val = enabled ? 1 : 0;

    write_register_bits(0x02, 0x0080, val << 7);
    delay(50);
    write_register_bits(0x42, 0x0080, val << 7);
    delay(50);
}

void IPS2550::set_master_gain_code(uint8_t code)
{
    code = code <= 95 ? code : 95;

    write_register_bits(0x42, 0x007F, code);
    delay(50);
    write_register_bits(0x02, 0x007F, code);
    delay(50);
}

void IPS2550::set_fine_gain_1(uint8_t code)
{
    code = code <= 0x7F ? code : 0x7F;

    write_register_bits(0x43, 0x007F, code);
    delay(50);
    write_register_bits(0x03, 0x007F, code);
    delay(50);
}

void IPS2550::set_fine_gain_2(uint8_t code)
{
    code = code <= 0x7F ? code : 0x7F;

    write_register_bits(0x45, 0x007F, code);
    delay(50);
    write_register_bits(0x05, 0x007F, code);
    delay(50);
}

void IPS2550::set_offset_1(int sign, uint8_t code)
{
    code = code <= 0x7F ? code : 0x7F;
    uint8_t sign_code = (sign < 0 ? 0 : 1) << 7;

    write_register_bits(0x44, 0x00FF, sign_code | code);
    delay(50);
    write_register_bits(0x04, 0x00FF, sign_code | code);
    delay(50);
}

void IPS2550::set_offset_2(int sign, uint8_t code)
{
    code = code <= 0x7F ? code : 0x7F;
    uint8_t sign_code = (sign < 0 ? 0 : 1) << 7;

    write_register_bits(0x46, 0x00FF, sign_code | code);
    delay(50);
    write_register_bits(0x06, 0x00FF, sign_code | code);
    delay(50);
}

void IPS2550::set_current_bias(uint8_t code)
{
    write_register_bits(0x47, 0x00FF, code);
    delay(50);
    write_register_bits(0x07, 0x00FF, code);
    delay(50);
}

void IPS2550::set_output_mode(OutputMode om)
{
    write_register_bits(0x40, 0x0002, om << 1);
    delay(50);
    write_register_bits(0x00, 0x0002, om << 1);
    delay(50);
}