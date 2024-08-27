#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>
#include <ArduinoJson.h>

#include <ADS131M0x.h>
#include <IPS2550.h>

#define BAUDRATE 115200
#define TIMEOUT 500

// On Arduino Uno:
// SPI_PIN_MOSI 11
// SPI_PIN_MISO 12
// SPI_PIN_CLK 13

#define IPS_I2C_ADDR 24
#define IPS_I2C_FREQ 100000

#define SPI_PIN_CS_ADC 6
#define SPI_PIN_CS_INC 10

#define ADC_NCHANNELS 4

#define ADC_PIN_DRDY 7
#define ADC_PIN_RST 5
#define ADC_PIN_CLKIN 9

#define TRIGGER_PIN 3
#define BOUNCE_TIME 500 // ms
#define HOLD_TIME 1000  // ms

void set_current_spi_device(int device, int device_pins[], int spi_device_modes[], int ndevices);

double estimate_vtx_rms(IPS2550 *a_ips, ADS131M0x *a_adc);
double estimate_vtx_pp(IPS2550 *a_ips, ADS131M0x *a_adc);
double estimate_vtx(IPS2550 *a_ips, ADS131M0x *a_adc);

ADS131M0x adc;
IPS2550 ips;

// be carefull to have them all in the same order
enum SPI_DEVICES
{
    D_SCL3300,
    D_ADS131M0x
};
int spi_cs_pins[] = {SPI_PIN_CS_INC, SPI_PIN_CS_ADC};
int spi_device_modes[] = {SPI_MODE0, SPI_MODE1};

bool old_trigger;
long long unsigned timer_trigger;
long long unsigned timer_hold;
bool flag_next_measure;

void setup()
{
    Serial.begin(BAUDRATE);
    Serial.setTimeout(TIMEOUT);

    // Wire.setClock(IPS_I2C_FREQ);
    Wire.begin();

    ips.init(Wire, IPS_I2C_ADDR);
    Serial.println("boh");
    // ips.set_voltage(VDD_3V3);
    // ips.set_automatic_gain_control(false);
    // ips.set_master_gain_boost(true);
    ips.set_master_gain_code(48);
    // ips.set_offset_1(1, 0);
    // ips.set_current_bias(0xFF);
    // ips.set_output_mode(DIFFERENTIAL);

    Serial.println("-----------------------------");
    Serial.print("Supply Voltage           ");
    Serial.println(ips.get_vdd() == VDD_3V3 ? "3.3 V" : "5 V");
    Serial.print("Output Mode              ");
    Serial.println(ips.get_output_mode() == DIFFERENTIAL ? "Differential" : "Single Ended");
    Serial.print("Automatic Gain Control   ");
    Serial.println(ips.get_automatic_gain_control() == 1 ? "True" : "False");
    Serial.print("Master Gain Code         ");
    Serial.println(ips.get_master_gain_code());
    Serial.print("Master Gain Boost (2x)   ");
    Serial.println(ips.get_master_gain_boost() == 1 ? "True" : "False");
    Serial.print("TX Current Bias          ");
    Serial.print(ips.get_tx_current_bias_uA());
    Serial.println(" uA");
    Serial.print("TX Frequency             ");
    Serial.print(ips.get_tx_frequency() / 1e6);
    Serial.println(" MHz");

    SPI.begin();
    SPI.beginTransaction(SPISettings(4000000, MSBFIRST, SPI_MODE0));

    pinMode(TRIGGER_PIN, INPUT_PULLUP);
    timer_trigger = 0;
    old_trigger = false;
    flag_next_measure = false;

    pinMode(SPI_PIN_CS_INC, OUTPUT);
    pinMode(SPI_PIN_CS_ADC, OUTPUT);

    set_current_spi_device(D_ADS131M0x, spi_cs_pins, spi_device_modes, 2);
    adc.begin(ADC_NCHANNELS, SPI, SPI_PIN_CS_ADC, ADC_PIN_DRDY, ADC_PIN_RST);
    adc.set_osr(OSR_8192);
}

void loop()
{
    bool trigger = !digitalRead(TRIGGER_PIN);
    bool bounce_ended = (millis() - timer_trigger) > BOUNCE_TIME;

    if (trigger && !old_trigger && bounce_ended)
    {
        timer_trigger = millis();
        flag_next_measure = true;
    }

    StaticJsonDocument<200> reply;

    // reply["ok"]["f"] = ips.get_tx_frequency();
    // reply["ok"]["vtx"] = estimate_vtx_pp(&ips, &adc);
    // Serial.println(ips.get_tx_frequency());
    // Serial.println(estimate_vtx_pp(&ips, &adc));

    // set_current_spi_device(D_ADS131M0x, spi_cs_pins, spi_device_modes, 2);
    // bool adc_ok = adc.read_data_if_ready();
    // if (adc_ok) {
    //   for (unsigned i = 0; i < ADC_NCHANNELS; ++i)
    //     reply["ok"][String(i + 1)] = adc.get_channel_voltage(i);
    //   reply["ok"]["flag"] = flag_next_measure;
    //   flag_next_measure = false;
    // } else {
    //   reply["err"]["ads131m0x"] = "NotReady";
    //   reply.remove("ok");
    // }

    // serializeJson(reply, Serial);
    // Serial.println();

    old_trigger = trigger;
    delay(1);
}

void set_current_spi_device(int device, int device_pins[], int spi_device_modes[], int ndevices)
{
    for (int i = 0; i < ndevices; i++)
    {
        digitalWrite(device_pins[i], HIGH);
    }

    SPI.setDataMode(spi_device_modes[device]);
    delay(1);

    digitalWrite(device_pins[device], LOW);
    delay(1);
}

double estimate_vtx_rms(IPS2550 *a_ips, ADS131M0x *a_adc)
{
    double vtx = 0.0;

    int starting_offset_sign = a_ips->get_offset_sign_2();
    uint8_t starting_offset = a_ips->get_offset_code_2();

    double gain = a_ips->get_master_gain();
    if (a_ips->get_master_gain_boost())
        gain *= 2;

    a_ips->set_offset_2(-1, 0x7F);
    Serial.println(a_ips->get_offset_code_2());

    set_current_spi_device(D_ADS131M0x, spi_cs_pins, spi_device_modes, 2);
    double rx1_n = 0.0;
    for (int i = 0; i < 10; ++i)
    {
        while (!adc.read_data_if_ready())
        {
        }
        rx1_n += adc.get_channel_voltage(2);
    }
    rx1_n /= 10;
    // Serial.print(rx1_n);
    // Serial.print(" ");

    a_ips->set_offset_2(1, 0x7F);

    double rx1_p = 0.0;
    for (int i = 0; i < 10; ++i)
    {
        while (!adc.read_data_if_ready())
        {
        }
        rx1_p += adc.get_channel_voltage(2);
    }
    rx1_p /= 10;
    // Serial.print(rx1_p);
    // Serial.println("");

    a_ips->set_offset_2(starting_offset_sign, starting_offset);
    Serial.println(a_ips->get_offset_code_2());

    vtx = (rx1_p - rx1_n) / (gain * 0x7F * 2.0 * 0.000015);

    return vtx;
}

double estimate_vtx_pp(IPS2550 *a_ips, ADS131M0x *a_adc)
{
    return estimate_vtx_rms(a_ips, a_adc) * sqrt(2);
}

double estimate_vtx(IPS2550 *a_ips, ADS131M0x *a_adc)
{
    return estimate_vtx_rms(a_ips, a_adc) * 2.0;
}
