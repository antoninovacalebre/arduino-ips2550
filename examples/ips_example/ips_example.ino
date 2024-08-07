#include <Arduino.h>
#include <Wire.h>

#include <IPS2550.h>

#define SERIAL_BAUDRATE 115200

#define IPS_I2C_ADDR 24
#define IPS_I2C_FREQ 100000

#define PIN_RX1 A0
#define PIN_RX2 A1
#define PIN_REF A2

IPS2550 ips;

double get_vrx(int pin_rx, int pin_ref);
double get_vrx_avg(int pin_rx, int pin_ref, int nsamples, int delay_ms);

double estimate_vtx_rms(IPS2550 *a_ips, int pin_rx, int pin_ref);
double estimate_vtx_pp(IPS2550 *a_ips, int pin_rx, int pin_ref);
double estimate_vtx(IPS2550 *a_ips, int pin_rx, int pin_ref);

void setup()
{
    Serial.begin(SERIAL_BAUDRATE);

    Wire.setClock(IPS_I2C_FREQ);
    Wire.begin();

    ips.init(Wire, IPS_I2C_ADDR);

    ips.set_voltage(VDD_3V3);
    ips.set_automatic_gain_control(false);
    ips.set_master_gain_boost(true);
    ips.set_master_gain_code(48);
    ips.set_offset_1(1, 0);
    ips.set_current_bias(0xFF);
    ips.set_output_mode(SINGLE_ENDED);
}

void loop()
{
    double freq = ips.get_tx_frequency();
    double vtx = estimate_vtx_pp(&ips, PIN_RX1, PIN_REF);
    double vrx1 = get_vrx(PIN_RX1, PIN_REF);
    double vrx2 = get_vrx(PIN_RX2, PIN_REF);

    Serial.print(freq);
    Serial.print("\t");
    Serial.print(vtx);
    Serial.print("\t");
    Serial.print(vrx1);
    Serial.print("\t");
    Serial.println(vrx2);
}

double get_vrx(int pin_rx, int pin_ref)
{
    return (analogRead(pin_rx) - analogRead(pin_ref)) / 1023 * 3.3;
}

double get_vrx_avg(int pin_rx, int pin_ref, int nsamples, int delay_ms)
{
    double rx = 0.0;
    for (unsigned i = 0; i < nsamples; ++i)
    {
        rx += get_vrx(pin_rx, pin_ref);
        delay(delay_ms);
    }
    return rx / nsamples;
}

double estimate_vtx_rms(IPS2550 *a_ips, int pin_rx, int pin_ref)
{
    double vtx = 0.0;

    int starting_offset_sign = a_ips->get_offset_sign_1();
    uint8_t starting_offset = a_ips->get_offset_code_1();

    double gain = a_ips->get_master_gain();
    if (a_ips->get_master_gain_boost())
        gain *= 2;

    a_ips->set_offset_1(-1, 0x7F);
    double rx1_n = get_vrx_avg(pin_rx, pin_ref, 10, 10);

    a_ips->set_offset_1(1, 0x7F);
    double rx1_p = get_vrx_avg(pin_rx, pin_ref, 10, 10);

    a_ips->set_offset_1(starting_offset_sign, starting_offset);

    vtx = (rx1_p - rx1_n) / (gain * 0x7F * 2.0 * 0.000015);

    return vtx;
}

double estimate_vtx(IPS2550 *a_ips, int pin_rx, int pin_ref)
{
    return estimate_vtx_rms(a_ips, pin_rx, pin_ref) * sqrt(2);
}

double estimate_vtx_pp(IPS2550 *a_ips, int pin_rx, int pin_ref)
{
    return estimate_vtx(a_ips, pin_rx, pin_ref) * 2.0;
}