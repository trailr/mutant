/*
 Copyright (C) 2011 James Coliz, Jr. <maniacbug@ymail.com>

 This program is free software; you can redistribute it and/or
 modify it under the terms of the GNU General Public License
 version 2 as published by the Free Software Foundation.
 */

/**
 * Example of a sensor network 
 *
 * This sketch demonstrates how to use the RF24Network library to
 * manage a set of low-power sensor nodes which mostly sleep but
 * awake regularly to send readings to the base.
 *
 * The example uses TWO sensors, a 'temperature' sensor and a 'voltage'
 * sensor.
 *
 * To see the underlying frames being relayed, compile RF24Network with
 * #define SERIAL_DEBUG.
 *
 * The logical node address of each node is set in EEPROM.  The nodeconfig
 * module handles this by listening for a digit (0-9) on the serial port,
 * and writing that number to EEPROM.
 */

#include <avr/pgmspace.h>
#include <RF24Network.h>
#include <RF24.h>
#include <SPI.h>
#include <Tictocs.h>
#include <Button.h>
#include <TictocTimer.h>
#include "nodeconfig.h"
#include "sleep.h"
#include "S_message.h"
#include "printf.h"

// This is for git version tracking.  Safe to ignore
#ifdef VERSION_H
#include "version.h"
#else
const char program_version[] = "Unknown";
#endif

// Pin definitions
#ifndef PINS_DEFINED
#define __PLATFORM__ "Getting Started board"

// Pins for radio
const int rf_ce = 8;
const int rf_csn = 9;

// Pins for sensors
const int temp_pin = A2;
const int voltage_pin = A3;

// Pins for status LED, or '0' for no LED connected
const int led_red = 0; 
const int led_yellow = 0; 
const int led_green = 0; 

// Button to control modes
const int button_a = 4;

// What voltage is a reading of 1023?
const unsigned voltage_reference = 5 * 256; // 5.0V
#endif

RF24 radio(rf_ce,rf_csn);
RF24Network network(radio);

// Our node configuration 
eeprom_info_t this_node;

// How many measurements to take.  64*1024 = 65536, so 64 is the max we can fit in a uint16_t.
const int num_measurements = 64;

// Sleep constants.  In this example, the watchdog timer wakes up
// every 4s, and every single wakeup we power up the radio and send
// a reading.  In real use, these numbers which be much higher.
// Try wdt_8s and 7 cycles for one reading per minute.> 1
const wdt_prescalar_e wdt_prescalar = wdt_4s;
//const int sleep_cycles_per_transmission = 1;

// Non-sleeping nodes need a timer to regulate their sending interval
Timer send_timer(2000);

uint16_t nodes[256];
uint16_t nodesCount = 0;
uint16_t robin = 0;

#define trigPin A3
#define echoPin A4

int redPin = A0;
int greenPin = A1;
int bluePin = A2;

void setup(void)
{
  //
  // Print preamble
  //
  
  Serial.begin(57600);
  printf_begin();
  printf_P(PSTR("\n\rRF24Network/examples/sensornet/\n\r"));
  printf_P(PSTR("PLATFORM: " __PLATFORM__ "\n\r"),program_version);
  printf_P(PSTR("VERSION: %s\n\r"),program_version);
  
  //
  // Pull node address out of eeprom 
  //

  // Which node are we?
  this_node = nodeconfig_read();

  // Prepare the startup sequence
  send_timer.begin();

  //
  // Bring up the RF network
  //

  SPI.begin();
  radio.begin();
  network.begin(/*channel*/ 92, /*node address*/ this_node.address);

  setupSonar();
  LEDOff();

  delay(10000);  

  if (this_node.address != 0) {
    S_message message;
    strcpy(message.payload,"REGISTER");

    printf_P(PSTR("---------------------------------\n\r"));
    printf_P(PSTR("%lu: Node [%o] registering with root node...\n\r"),millis(),this_node.address);
    
    // Send it to the base
    RF24NetworkHeader header(/*to node*/ 0, /*type*/ false ? 's' : 'S');
    bool ok = network.write(header,&message,sizeof(message));
    if (ok)
    {
      printf_P(PSTR("%lu: APP Send ok\n\r"),millis());
    }
    else
    {
      printf_P(PSTR("%lu: APP Send failed\n\r"),millis());
    }
  }
}

void loop(void)
{
  bool doAck = false;
  uint16_t recipient = 0;
  // Update objects
  theUpdater.update();

  // Pump the network regularly
  network.update();

  // If we are the base, is there anything ready for us?
  while ( network.available() )
  {
    // If so, grab it and print it out
    RF24NetworkHeader header;
    S_message message;
    network.read(header,&message,sizeof(message));
    printf_P(PSTR("%lu: APP Received #%u %s from 0%o\n\r"),millis(),header.id,message.toString(),header.from_node);
    if (strcmp(message.payload,"REGISTER") == 0) {
      nodes[nodesCount] = header.from_node;
      nodesCount++;
    } else if (strcmp(message.payload,"REPORT") == 0) {
      doAck = true;
    } else if (strcmp(message.payload,"LEDR") == 0) {
      setColor(255,0,0);
    } else if (strcmp(message.payload,"LEDG") == 0) {
      setColor(0,255,0);
    } else if (strcmp(message.payload,"LEDB") == 0) {
      setColor(0,0,255);
    }
  }

  recipient = nodes[robin];
  robin++;
  if (robin > nodesCount) {
    robin = 0;
  }

  // If we are the kind of node that sends readings, AND it's time to send
  // a reading AND we're in the mode where we send readings...
  if ( ( this_node.address == 0 && send_timer.wasFired() && recipient != 0 ) || doAck )
  {
    S_message message;
    if (doAck) {
      long distance = getDistance();
      // printf_P(PSTR("REPORTING PROXIMITY: %03d\n\r"),distance);
      sprintf(message.payload,"P%03d",distance);
      recipient = 0;
    } else {
      strcpy(message.payload,"REPORT");
    }

    printf_P(PSTR("---------------------------------\n\r"));
    printf_P(PSTR("%lu: APP Sending %s to 0%o...\n\r"),millis(),message.toString(),recipient);
    
    // Send it to the base
    RF24NetworkHeader header(/*to node*/ recipient, /*type*/ false ? 's' : 'S');
    bool ok = network.write(header,&message,sizeof(message));
    if (ok)
    {
      // printf_P(PSTR("%lu: APP Send ok\n\r"),millis());
    }
    else
    {
      printf_P(PSTR("%lu: APP Send failed\n\r"),millis());
    }

  }

  // Listen for a new node address
  nodeconfig_listen();
}

void setupSonar(void) {
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);

  pinMode(redPin, OUTPUT);
  pinMode(greenPin, OUTPUT);
  pinMode(bluePin, OUTPUT);  
  LEDOff();
}

long getDistance(void) {
  long duration, distance;
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  duration = pulseIn(echoPin, HIGH);
  distance = (duration/2) / 29.1;
  return distance;
}

void LEDOff() {
  analogWrite(redPin, 255);
  analogWrite(greenPin, 255);
  analogWrite(bluePin, 255);  
}

void setColor(int red, int green, int blue)
{
  Serial.println("SETTING COLOR");
  analogWrite(redPin, 255-red);
  analogWrite(greenPin, 255-green);
  analogWrite(bluePin, 255-blue);  
}