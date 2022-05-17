void beepnblink(){
  if (p<30 && overtime == false){ //I will transfer this to a tab once I test it
      //analogWrite(piezo, 50); //Turn the piezo on for 300ms
      digitalWrite(statusLED, HIGH);
    }
  
    else if (p == 30 && overtime == false){
      //analogWrite(piezo, 0);
      digitalWrite(statusLED, LOW);
    }

    else if (p == 200 && overtime == false){
      p = 0;
    }
  p++;

  while (overtime == true){
    blinkLED(1);
    delay(500);
  }
}
