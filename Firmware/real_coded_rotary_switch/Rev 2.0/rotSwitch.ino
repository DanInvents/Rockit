// This program reads the rotary switch.

void readRotSwitch(){
 for (int k = 0; k < 4; k++){
  if (digitalRead(switchPins[k]) == LOW) {
    bitSet(rotValue, k); //sets bit k to 1
  }
  else {
     bitClear(rotValue, k); //sets bit k to 0
  }
 }
}

void switchStartup(){
    readRotSwitch();

  if (rotValue == 10){ //A Automatic mode
    servo1.write(EEPROM.read(2)); //EEPROM.read(2)
    servo1.attach(servo1pin);

    delay(100); //It is important to have a delay to reduce the current spike drawn by the motors
    servo2.write(EEPROM.read(4)); //EEPROM.read(4)
    servo2.attach(servo2pin);

    automatic = true;
    delay(300);
    blinkLED(EEPROM.read(0));
    delay(500);
    blinkLED(EEPROM.read(6));
    servo1.detach(); //I detach the servos to save power
    servo2.detach();
    return;
  }

  else if (rotValue == 11){ //B Timer mode
   servo1.write(EEPROM.read(2)); //EEPROM.read(2)
   servo1.attach(servo1pin);

   delay(100); //It is important to have a delay to reduce the current spike drawn by the motors
   servo2.write(EEPROM.read(4)); //EEPROM.read(4)
   servo2.attach(servo2pin);

   timer = true;
   delay(300);
   blinkLED(EEPROM.read(1));
   delay(500);
   blinkLED(EEPROM.read(6));
   servo1.detach(); //I detach the servos to save power
   servo2.detach();
   return;
  }

  else if (rotValue == 12){ //C, Configure the time for parachute deployment on automatic mode
    while(1){
      readRotSwitch();
      blinkLED(1);
      if (previousValue != rotValue){
        EEPROM.write(0, rotValue);
        EEPROM.commit();
        previousValue = rotValue;
          }
        }
      } 

   else if (rotValue == 13){ //D, Configure the time for parachute deployment on timer mode.
    while(1){
      readRotSwitch();
      blinkLED(1);
      if (previousValue != rotValue){
        EEPROM.write(1, rotValue);
        EEPROM.commit();
        previousValue = rotValue;
        }
      }
    }   
    

  else if (rotValue == 14){ //E, Adjust servo's 1 initial possition
    while(1){
      readRotSwitch();
      servo1.write(180*rotValue/15);
      servo1.attach(servo1pin);
      blinkLED(1);
      if (previousValue != rotValue){
        EEPROM.write(2, 180*rotValue/15);
        EEPROM.commit();
      }
      previousValue == rotValue;
    }
  }

  else if (rotValue == 15){ //F, Adjust servo's 1 final possition
      while(1){
        readRotSwitch();        
        servo1.write(180*rotValue/15);
        servo1.attach(servo1pin);

        blinkLED(1);
        if (previousValue != rotValue){
          EEPROM.write(3, 180*rotValue/15);
          EEPROM.commit();
        }
        previousValue == rotValue;
      }
  }

  else if (rotValue == 0){ //0, Adjust the servo's 2 initial possition
    while(1){
      readRotSwitch();
      servo2.write(180*rotValue/15);
      servo2.attach(servo2pin);

      blinkLED(1);
      if (previousValue != rotValue){
        EEPROM.write(4, 180*rotValue/15);
        EEPROM.commit();
      }
      previousValue == rotValue;
    }
  }

  else if (rotValue == 1){ //1, Adjust the servo's 2 final possition
      while(1){
        readRotSwitch();        
        servo2.write(180*rotValue/15);
        servo2.attach(servo2pin);

        blinkLED(1);
        if (previousValue != rotValue){
          EEPROM.write(5, 180*rotValue/15);
          EEPROM.commit();
        }
        previousValue == rotValue;
      }
  }

    else if (rotValue == 2){ //2, Adjust the deploy time for servo 2 after servo 1
    while(1){
      readRotSwitch();
      blinkLED(1);
      if (previousValue != rotValue){
        EEPROM.write(6, rotValue);
        EEPROM.commit();
        previousValue = rotValue;
      }
    }
  }
    else {
      while (true){
        sleep_ms(10000);
      }
  }
} 
