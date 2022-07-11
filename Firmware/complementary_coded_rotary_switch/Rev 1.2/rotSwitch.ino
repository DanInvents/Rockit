// This program reads the rotary switch.

void readRotSwitch(){
 for (int k = 0; k < 4; k++){
  if (digitalRead(switchPins[k]) == LOW) {
    bitClear(rotValue, k); //sets bit k to 0
  }
  else {
    bitSet(rotValue, k); //sets bit k to 1
  }
 }
}

void switchStartup(){
    readRotSwitch();
  if (rotValue == 10){ //A Automatic mode
    servo1.attach(28);
    servo2.attach(27);
    servo1.write(EEPROM.read(2)); //EEPROM.read(2)
    delay(100); //It is important to have a delay to reduce the current spike drawn by the motors
    servo2.write(EEPROM.read(4)); //EEPROM.read(4)

    automatic = true;
    delay(300);
    blinkLED(EEPROM.read(0));
    delay(500);
    blinkLED(EEPROM.read(6));
    return;
  }

  else if (rotValue == 11){ //B Timer mode
   servo1.attach(28);
   servo2.attach(27);
   servo1.write(EEPROM.read(2)); //EEPROM.read(2)
   delay(100); //It is important to have a delay to reduce the current spike drawn by the motors
   servo2.write(EEPROM.read(4)); //EEPROM.read(4)

   timer = true;
   delay(300);
   blinkLED(EEPROM.read(1));
   delay(500);
   blinkLED(EEPROM.read(6));
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
    servo1.attach(28);
    servo2.attach(27);
    while(1){
      readRotSwitch();
      servo1.write(180*rotValue/15);
      blinkLED(1);
      if (previousValue != rotValue){
        EEPROM.write(2, 180*rotValue/15);
        EEPROM.commit();
      }
      previousValue == rotValue;
    }
  }

  else if (rotValue == 15){ //F, Adjust servo's 1 final possition
    servo1.attach(28);
    servo2.attach(27);
      while(1){
        readRotSwitch();        
        servo1.write(180*rotValue/15); //Work on the problem with the starting possition.
        blinkLED(1);
        if (previousValue != rotValue){
          EEPROM.write(3, 180*rotValue/15);
          EEPROM.commit();
        }
        previousValue == rotValue;
      }
  }

  else if (rotValue == 0){ //0, Adjust the servo's 2 initial possition
    servo1.attach(28);
    servo2.attach(27);
    while(1){
      readRotSwitch();
      servo2.write(180*rotValue/15);
      blinkLED(1);
      if (previousValue != rotValue){
        EEPROM.write(4, 180*rotValue/15);
        EEPROM.commit();
      }
      previousValue == rotValue;
    }
  }

  else if (rotValue == 1){ //1, Adjust the servo's 2 final possition
    servo1.attach(28);
    servo2.attach(27);
      while(1){
        readRotSwitch();        
        servo2.write(180*rotValue/15); //Work on the problem with the starting possition.
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

void blinkLED(int n){ //Blinks the blue LED every 200 ms
    for (int i=0; i<n; i++){
      digitalWrite(statusLED, HIGH);
      delay(200);
      digitalWrite(statusLED, LOW);
      delay(200);
    }
}
