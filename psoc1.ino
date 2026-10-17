// PSoC1 programátor (zceemja/psoc1_prog) – opravená verze
// Zapojení na Arduino Uno / Uno 328PB:
//   D8  = SCLK (P1[1])
//   D11 = XRES
//   D12 = SDATA (P1[0]) -> vyžaduje vnější Pull-up rezistor (4.7k - 10k) k VCC - funční i bez něj
//   D13 = LED diagnostika
//   5V / 3.3V a GND

#include <Arduino.h>
#include <avr/io.h>
#include <util/delay.h>

// Zajištění správné definice bitových masek
#ifndef BIT0
#define BIT0 (1 << 0)
#define BIT1 (1 << 1)
#define BIT2 (1 << 2)
#define BIT3 (1 << 3)
#define BIT4 (1 << 4)
#define BIT5 (1 << 5)
#define BIT6 (1 << 6)
#define BIT7 (1 << 7)
#endif

// Definice pinů na PORTB
#define SCL_PIN BIT0  // Pin 8 (PB0)
#define RES_PIN BIT3  // Pin 11 (PB3)
#define SDA_PIN BIT4  // Pin 12 (PB4)
#define LED_PIN BIT5  // Pin 13 (PB5)

// Půlperioda SCLK cca 1 µs
#define DELAY _delay_us(1)

// Délka pulzu XRES
#define XRES_PULSE_US 300

#include "devices.h"

#define _CLK1 PORTB |= SCL_PIN; DELAY
#define _CLK0 PORTB &= ~SCL_PIN; DELAY

#define PULL_TIMEOUT_MS 500

uint16_t dev_id = 0;
uint8_t ram_blk[64];

// 0 = OK, jinak číslo kroku, kde čip neodpověděl
uint8_t err = 0;

const uint16_t fw = 0x0003;

void data_send(const uint8_t *data, uint16_t len) {
  uint16_t k;
  uint8_t prev = 0;
  uint8_t curr = 1;

  // Nastavíme piny jako výstupní
  DDRB  |= (SDA_PIN | SCL_PIN | LED_PIN);
  PORTB |= SDA_PIN | LED_PIN;

  for (k = 0; k < len; k++) {
    // Opravená hodnota bitu (MSB first) s ohledem na prioritu operátorů
    curr = ((data[k / 8] & (1 << (7 - (k % 8)))) == 0);

    if (curr != prev) {
      prev = curr;
      PORTB ^= (SDA_PIN | SCL_PIN);
    } else {
      PORTB ^= SCL_PIN;
    }
    DELAY;
    PORTB ^= SCL_PIN;
    DELAY;
  }
}

// Čeká, až SDATA dosáhne požadované úrovně. false = timeout.
bool wait_sda(bool high, uint16_t ms) {
  uint32_t t0 = millis();
  while (((PINB & SDA_PIN) != 0) != high) {
    if (millis() - t0 > ms) return false;
  }
  return true;
}

// WAIT-AND-POLL. Vrací 0 = OK, 1/2/3 = ve které fázi to vypršelo.
uint8_t data_pull() {
  uint8_t j;

  PORTB &= ~(LED_PIN | SDA_PIN);
  DDRB &= ~SDA_PIN; // SDATA do režimu vstupu

  if (!wait_sda(false, PULL_TIMEOUT_MS)) return 1;    // SDATA nikdy nešla do L

  _CLK1;
  _CLK0;

  if (!wait_sda(true, PULL_TIMEOUT_MS)) return 2;     // Čip nezačal operaci (H)
  if (!wait_sda(false, PULL_TIMEOUT_MS)) return 3;    // Čip nedokončil operaci (L)

  for (j = 0; j < 40; j++) {
    _CLK1;
    _CLK0;
  }
  return 0;
}

// Pomocník: provede pull, při chybě uloží kód kroku (krok*10 + fáze)
bool pull_or_fail(uint8_t step) {
  uint8_t r = data_pull();
  if (r != 0) {
    if (err == 0) err = step * 10 + r;
    return false;
  }
  return true;
}

void data_read(uint8_t *buf, uint16_t size) {
  PORTB &= ~(SDA_PIN | LED_PIN);
  DDRB &= ~SDA_PIN; // Vstup pro SDATA

  uint8_t k;
  _CLK1;
  _CLK0;

  for (k = 0; k < size; k++) {
    _CLK1;
    _CLK0;
    if (k % 8 == 0) buf[k / 8] = 0;
    if ((PINB & SDA_PIN) != 0) {
      buf[k / 8] |= (1 << (7 - (k % 8)));
    }
  }
  _CLK1;
  _CLK0;
}

uint8_t read_byte(uint8_t cmd) {
  uint8_t buf[] = {(uint8_t)(0b10100000 | (cmd >> 3)), 0};
  buf[1] = cmd << 5;
  data_send(buf, 11);
  data_read(&buf[1], 8);
  data_send(buf, 1);
  return buf[1];
}

void reset_device() {
  DDRB |= RES_PIN;
  PORTB |= RES_PIN;
  _delay_us(XRES_PULSE_US);
  PORTB &= ~RES_PIN;
}

void initialise() {
  err = 0;
  DDRB |= (SCL_PIN | SDA_PIN | LED_PIN | RES_PIN);
  reset_device();
  
  data_send(init1, init1_len);
  if (!pull_or_fail(1)) return;
  
  data_send(init2, init2_len);
  if (!pull_or_fail(2)) return;
  
  // Pozor: Pokud používáte 3.3V napájení, změňte init3_5V na init3 a init3_5V_len na init3_len
  data_send(init3_5V, init3_5V_len);
}

uint16_t read_id() {
  if (err) return 0;
  data_send(id_setup, id_setup_len);
  if (!pull_or_fail(3)) return 0;
  uint16_t result = read_byte(0b11111000) << 8;
  result |= read_byte(0b11111001);
  return result;
}

uint16_t read_checksum(uint16_t dev_id) {
  if (dev_id == dev_CY8C24123A || dev_id == dev_CY8C24223A || dev_id == dev_CY8C24423A)
    data_send(checksum_1, checksum_len);
  else
    data_send(checksum_0, checksum_len);
  if (!pull_or_fail(4)) return 0;
  uint16_t result = read_byte(0b11111001) << 8;
  result |= read_byte(0b11111000);
  return result;
}

void erase_all_memory() {
  data_send(bulk_erase, bulk_erase_len);
  pull_or_fail(5);
}

void load_ram_to_chip() {
  uint8_t payload[3];
  for (uint8_t ADDR = 0; ADDR < 64; ADDR++) {
    uint8_t data = ram_blk[ADDR];
    payload[0] = 0b10010000 | (ADDR >> 3);
    payload[1] = (ADDR << 5) | (data >> 3);
    payload[2] = (data << 5) | 0b00011100;
    data_send(payload, 22);
  }
}

void set_block_num(uint8_t block) {
  uint8_t payload[3];
  payload[0] = 0b10011111;
  payload[1] = 0b01000000 | (block >> 3);
  payload[2] = 0b00011100 | (block << 5);
  data_send(payload, 22);
}

void write_block(uint16_t dev_id, uint8_t block) {
  load_ram_to_chip();
  set_block_num(block);
  if (dev_id >= dev_CY8C27143 && dev_id <= dev_CY8C27643)
    data_send(program_block_1, program_block_len);
  else
    data_send(program_block_0, program_block_len);
  pull_or_fail(6);
}

void write_secure() {
  load_ram_to_chip();
  data_send(secure, secure_len);
  pull_or_fail(7);
}

void read_block(uint8_t block, uint8_t offset) {
  set_block_num(block);
  data_send(verify_setup, verify_setup_len);
  if (!pull_or_fail(8)) return;
  for (uint8_t ADDR = 0; ADDR < 64; ADDR++) {
    ram_blk[ADDR] = read_byte(ADDR + offset);
  }
}

// Test linky SDATA: čte úroveň bez pull-upu a s pull-upem
void pin_test() {
  DDRB &= ~SDA_PIN;
  PORTB &= ~SDA_PIN;          // Bez interního pull-upu
  delay(2);
  uint8_t a = (PINB & SDA_PIN) != 0;
  PORTB |= SDA_PIN;           // S interním pull-upem
  delay(2);
  uint8_t b = (PINB & SDA_PIN) != 0;
  PORTB &= ~SDA_PIN;
  Serial.print(F("SDATA bez pull-up="));
  Serial.print(a);
  Serial.print(F(", s pull-up="));
  Serial.println(b);
}

void print_status() {
  if (err == 0) {
    Serial.print(F("PSoC OK, ID=0x"));
    Serial.println(dev_id, HEX);
  } else {
    Serial.print(F("PSoC CHYBA, kod="));
    Serial.println(err);
  }
}

void setup(void) {
  Serial.begin(9600);
  initialise();
  dev_id = read_id();
  print_status();
  Serial.print(F("> "));
}

void loop() {
  uint8_t command;
  if (Serial.readBytes(&command, 1) > 0) {
    if (command == 'd') {
      Serial.println(dev_id, HEX);
    }
    else if (command == 'D') {
      Serial.write(dev_id);
      Serial.write(dev_id >> 8);
      Serial.println();
    }
    else if (command == 'i') {
      initialise();
      dev_id = read_id();
      Serial.println();
    }
    else if (command == 'r') {
      uint8_t args[2] = {0x00, 0x80};
      Serial.readBytes(args, 2);
      read_block(args[0], args[1]);
      Serial.println();
    }
    else if (command == 'e') {
      erase_all_memory();
      Serial.println();
    }
    else if (command == 'w') {
      if (Serial.readBytes(&command, 1) > 0) {
        write_block(dev_id, command);
      }
      Serial.println();
    }
    else if (command == 'f') {
      Serial.println(fw, HEX);
    }
    else if (command == 'F') {
      Serial.write(fw);
      Serial.write(fw >> 8);
      Serial.println();
    }
    else if (command == 'c') {
      Serial.println(read_checksum(dev_id), HEX);
    }
    else if (command == 'C') {
      uint16_t cs = read_checksum(dev_id);
      Serial.write(cs);
      Serial.write(cs >> 8);
      Serial.println();
    }
    else if (command == 's') {
      Serial.write(ram_blk, 64);
      Serial.println();
    }
    else if (command == 't') {
      Serial.readBytes(ram_blk, 64);
      Serial.println();
    }
    else if (command == 'x') {
      write_secure();
      Serial.println();
    }
    else if (command == 'a') {
      reset_device();
      Serial.println();
    }
    else if (command == '?') {   // Diagnostika
      print_status();
    }
    else if (command == 'p') {   // Test SDATA pinu
      pin_test();
    }
    else if (command == '\n' || command == '\r') {
      Serial.println();
    }
    else {
      Serial.println(F("invalid"));
    }
    Serial.print(F("> "));
  }
}
