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

double get_rx1();
double get_rx2();
double estimate_vtx();

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
    ips.set_current_bias(0xFF);
    ips.set_output_mode(SINGLE_ENDED);
}

void loop()
{
    double freq = ips.get_tx_frequency();
    Serial.println(freq);
}

double get_rx1()
{
    return (analogRead(PIN_RX1) - analogRead(PIN_REF)) / 1023 * 3.3;
}

double get_rx2()
{
    return (analogRead(PIN_RX2) - analogRead(PIN_REF)) / 1023 * 3.3;
}