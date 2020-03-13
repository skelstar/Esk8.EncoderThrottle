#define DEBUG_OUT Serial
#define PRINTSTREAM_FALLBACK
#include "Debug.hpp"

#include <Arduino.h>
#include <elapsedMillis.h>
#include <Button2.h>
#include <Wire.h>

Button2 deadman(0);
Button2 mapSwapButton(35);

#ifndef READ_TRIGGER_PERIOD
#define READ_TRIGGER_PERIOD 200
#endif

#include <EncoderThrottleLib.h>

//------------------------------------------------------------

EncoderThrottleLib encoder;

void encoder_push(i2cEncoderLibV2 *obj)
{
  encoder.clear();
  DEBUG("Encoder is pushed!");
}

void encoder_double_push(i2cEncoderLibV2 *obj)
{
  Serial.println("Encoder is double pushed!");
}

void swapMapDoubleClick(Button2 &btn)
{
  switch ((int)encoder.getMap())
  {
  case ThrottleMap::LINEAR:
    encoder.setMap(GENTLE);
    encoder.clear();
    DEBUG("ThrottleMap::GENTLE");
    break;
  case ThrottleMap::GENTLE:
    encoder.setMap(SMOOTHED);
    encoder.clear();
    DEBUG("ThrottleMap::SMOOTHED");
    break;
  case ThrottleMap::SMOOTHED:
    encoder.setMap(LINEAR);
    encoder.clear();
    DEBUG("ThrottleMap::LINEAR");
    break;
  default:
    DEBUG("DEFAULT");
    break;
  }
}

//------------------------------------------------------------

void setup(void)
{
  Serial.begin(115200);
  Serial.println("**** I2C Encoder V2 basic example ****");

  Wire.begin();
  encoder.init(/*pushed*/ encoder_push,
               /*double*/ encoder_double_push,
               /*min*/ -8,
               /*max*/ 8);
  encoder.setMap(SMOOTHED);

  mapSwapButton.setDoubleClickHandler(swapMapDoubleClick);
}

elapsedMillis since_checked_encoder;

void loop()
{
  mapSwapButton.loop();

  if (since_checked_encoder > 200)
  {
    deadman.loop();

    since_checked_encoder = 0;
    /* Check the status of the encoder and call the callback */
    uint8_t t = encoder.get(deadman.isPressed());
  }

  delay(1);
}