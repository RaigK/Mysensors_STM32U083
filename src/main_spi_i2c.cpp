/*
 * Phase 3 checkpoint: SPI + Wire shim validated on real hardware.
 *
 *  SPI: read RFM69 register 0x10 (VERSION). Expected: 0x24 on RFM69HW.
 *  I2C: read HDC1080 Manufacturer ID (register 0xFE). Expected: 0x5449 ("TI").
 *       Also Device ID (register 0xFF). Expected: 0x1050.
 */
#include <Arduino.h>
#include <SPI.h>
#include <Wire.h>

static constexpr uint8_t RFM69_CS_PIN  = PB6;
static constexpr uint8_t RFM69_REG_VER = 0x10;

static constexpr uint8_t HDC1080_ADDR  = 0x40;
static constexpr uint8_t HDC1080_MFR   = 0xFE;
static constexpr uint8_t HDC1080_DEV   = 0xFF;

static uint8_t rfm69_read(uint8_t reg)
{
    digitalWrite(RFM69_CS_PIN, LOW);
    SPI.transfer(reg & 0x7F);           /* MSB=0 → read */
    uint8_t v = SPI.transfer(0x00);
    digitalWrite(RFM69_CS_PIN, HIGH);
    return v;
}

static uint16_t hdc1080_read_reg(uint8_t reg)
{
    Wire.beginTransmission(HDC1080_ADDR);
    Wire.write(reg);
    uint8_t err = Wire.endTransmission(false);
    if (err) { Serial.print("I2C write err="); Serial.println(err); return 0xDEAD; }

    delay(2);   /* conversion / register settle */

    if (Wire.requestFrom(HDC1080_ADDR, (size_t)2) != 2) {
        Serial.println("I2C read short");
        return 0xDEAD;
    }
    uint8_t hi = (uint8_t)Wire.read();
    uint8_t lo = (uint8_t)Wire.read();
    return ((uint16_t)hi << 8) | lo;
}

void setup()
{
    Serial.begin(115200);
    delay(100);
    Serial.println();
    Serial.println("=== Phase 3: SPI + I2C probe ===");

    /* --- SPI / RFM69 --- */
    pinMode(RFM69_CS_PIN, OUTPUT);
    digitalWrite(RFM69_CS_PIN, HIGH);
    SPI.begin();
    SPI.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE0));

    uint8_t ver = rfm69_read(RFM69_REG_VER);
    Serial.print("RFM69 VERSION (0x10) = 0x");
    Serial.println(ver, HEX);
    Serial.println(ver == 0x24 ? "  -> OK (RFM69HW)" : "  -> UNEXPECTED");

    SPI.endTransaction();

    /* --- I2C / HDC1080 --- */
    Wire.begin();
    Wire.setClock(100000);
    uint16_t mfr = hdc1080_read_reg(HDC1080_MFR);
    Serial.print("HDC1080 MFR  (0xFE) = 0x"); Serial.println(mfr, HEX);
    Serial.println(mfr == 0x5449 ? "  -> OK (Texas Instruments)" : "  -> UNEXPECTED");

    uint16_t dev = hdc1080_read_reg(HDC1080_DEV);
    Serial.print("HDC1080 DEV  (0xFF) = 0x"); Serial.println(dev, HEX);
    Serial.println(dev == 0x1050 ? "  -> OK (HDC1080)" : "  -> UNEXPECTED");

    pinMode(PB0, OUTPUT);
}

void loop()
{
    digitalWrite(PB0, HIGH); delay(500);
    digitalWrite(PB0, LOW);  delay(500);
}
