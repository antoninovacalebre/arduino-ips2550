#include "Arduino.h"
#include "IPS2550.h"

uint32_t bit_length(uint32_t n)
{
    uint32_t i = 1;
    while ((n >> i) != 0)
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

uint16_t get_bits_in_word(uint16_t word, uint16_t *bit_list, uint16_t len)
{
    uint16_t res = 0x00;
    for (uint16_t i = 0; i < len; ++i)
    {
        res = res << 1;
        res |= (word & (1 << bit_list[i])) >> bit_list[i];
    }

    return res;
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

void IPS2550::write_register_bits(uint8_t reg_addr, uint16_t *bit_list, uint16_t *value_list, uint16_t len)
{
    uint16_t reg = read_reg(reg_addr);

    uint16_t mask = 0x00;
    for (uint16_t i = 0; i < len; ++i)
        mask = mask | (0x01 << bit_list[i]);
    uint16_t new_word = reg & ~mask;

    mask = 0x00;
    for (uint16_t i = 0; i < len; ++i)
        mask = mask | (value_list[i] << bit_list[i]);
    new_word = new_word | mask;

    write_reg(reg_addr, new_word);
}

void IPS2550::set_voltage(VDD vdd)
{
    uint16_t val[] = {vdd};
    uint16_t bit[] = {0};

    write_register_bits(0x41, bit, val, 1);
    delay(50);
    write_register_bits(0x01, bit, val, 1);
    delay(50);
}

void IPS2550::set_automatic_gain_control(boolean enabled)
{
    uint16_t val[] = {enabled ? 0 : 1};
    uint16_t bit[] = {9};

    write_register_bits(0x00, bit, val, 1);
    delay(50);
    write_register_bits(0x40, bit, val, 1);
    delay(50);
}

void IPS2550::set_master_gain_boost(boolean enabled)
{
    uint16_t val[] = {enabled ? 1 : 0};
    uint16_t bit[] = {7};

    write_register_bits(0x02, bit, val, 1);
    delay(50);
    write_register_bits(0x42, bit, val, 1);
    delay(50);
}

void IPS2550::set_master_gain_code(uint8_t code)
{
    code = code <= 95 ? code : 95;

    uint16_t val[7];
    for (uint16_t i = 0; i < 7; ++i)
        val[i] = (code & (0x1 << i)) >> i;

    uint16_t bit[] = {0, 1, 2, 3, 4, 5, 6};

    write_register_bits(0x42, bit, val, 7);
    delay(50);
    write_register_bits(0x02, bit, val, 7);
    delay(50);
}

void IPS2550::set_fine_gain_1(uint8_t code)
{
    code = code <= 0x7F ? code : 0x7F;

    uint16_t val[7];
    for (uint16_t i = 0; i < 7; ++i)
        val[i] = (code & (0x1 << i)) >> i;

    uint16_t bit[] = {0, 1, 2, 3, 4, 5, 6};

    write_register_bits(0x43, bit, val, 7);
    delay(50);
    write_register_bits(0x03, bit, val, 7);
    delay(50);
}

void IPS2550::set_fine_gain_2(uint8_t code)
{
    code = code <= 0x7F ? code : 0x7F;

    uint16_t val[7];
    for (uint16_t i = 0; i < 7; ++i)
        val[i] = (code & (0x1 << i)) >> i;

    uint16_t bit[] = {0, 1, 2, 3, 4, 5, 6};

    write_register_bits(0x45, bit, val, 7);
    delay(50);
    write_register_bits(0x05, bit, val, 7);
    delay(50);
}

void IPS2550::set_offset_1(int sign, uint8_t code)
{
    code = code <= 0x7F ? code : 0x7F;
    uint8_t sign_code = sign < 0 ? 0 : 1;

    uint16_t val[8];
    for (uint16_t i = 0; i < 7; ++i)
        val[i] = (code & (0x1 << i)) >> i;
    val[7] = sign_code;

    uint16_t bit[] = {0, 1, 2, 3, 4, 5, 6, 7};

    write_register_bits(0x44, bit, val, 8);
    delay(50);
    write_register_bits(0x04, bit, val, 8);
    delay(50);
}

void IPS2550::set_offset_2(int sign, uint8_t code)
{
    code = code <= 0x7F ? code : 0x7F;
    uint8_t sign_code = sign < 0 ? 0 : 1;

    uint16_t val[8];
    for (uint16_t i = 0; i < 7; ++i)
        val[i] = (code & (0x1 << i)) >> i;
    val[7] = sign_code;

    uint16_t bit[] = {0, 1, 2, 3, 4, 5, 6, 7};

    write_register_bits(0x46, bit, val, 8);
    delay(50);
    write_register_bits(0x06, bit, val, 8);
    delay(50);
}

void IPS2550::set_current_bias(uint8_t code)
{
    uint16_t val[8];
    for (uint16_t i = 0; i < 8; ++i)
        val[i] = (code & (0x1 << i)) >> i;

    uint16_t bit[] = {0, 1, 2, 3, 4, 5, 6, 7};

    write_register_bits(0x47, bit, val, 8);
    delay(50);
    write_register_bits(0x07, bit, val, 8);
    delay(50);
}

void IPS2550::set_output_mode(OutputMode om)
{
    uint16_t val[] = {om};
    uint16_t bit[] = {1};

    write_register_bits(0x40, bit, val, 1);
    delay(50);
    write_register_bits(0x00, bit, val, 1);
    delay(50);
}