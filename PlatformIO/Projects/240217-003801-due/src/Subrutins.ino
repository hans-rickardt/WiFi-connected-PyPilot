
double ReadCabinTemp() {

  sensorValue = analogRead(analogTemp);
  //Serial.println("temp");
  //Serial.println(sensorValue);
  //Serial.println(map(sensorValue, 2115, 3250, 32,16));

  return CToKelvin(map(sensorValue, 2115, 3250, 32, 16)); // Read here the true temperature e.g. from analog input
}
/*
  DiffVolt return a value from analog input 12volt via 15k ohm 3.3k ohm to ground
  substract referense volt for compensate engine temp and fuel
*/
double DiffVolt() {
  double  ref = 2500;
  double sensorValue = analogRead(analogVolt);
  //  Serial.print("Sensor  ");
  //  Serial.println(sensorValue);
  return   sensorValue - ref;
}

double ReadWaterTemp() {
  return CToKelvin(15.5); // Read here the true temperature e.g. from analog input
}

/* Reset CPU*/
void softReset()
{
  __DSB;
  SCB->AIRCR = ((0x5FA << SCB_AIRCR_VECTKEY_Pos) | SCB_AIRCR_SYSRESETREQ_Msk);//software reset
}


//
// Get rpm
//

double getrpm()
{

  //Update RPM every second
  //  Serial.println("Start");

  //Don't process interrupts during calculations
  detachInterrupt(digitalPinToInterrupt(interruptPin));
  rpmcount_nu = rpmcount;
  rpmcount = 0;
  timeold_nu = timeold;
  timeold = millis();
  attachInterrupt(digitalPinToInterrupt(interruptPin), rpm_fun, FALLING     );
  //Note that this would be 60*1000/(millis() - timeold)*rpmcount if the interrupt
  //happened once per revolution instead of twice. Other multiples could be used
  //for multi-bladed propellers or fans
  rpm = 60 * 1000 / (timeold - timeold_nu) * rpmcount_nu;

  return rpm;
}

//
// interrupt from RPM

void rpm_fun()
{
  //Each rotation, this interrupt function is run twice, so take that into consideration for
  //calculating RPM
  //Update count
  rpmcount++;
}


/* Get command from Serical port
  "Reset" Reset stored Cal data from rom
  "Reboot" reboot
  "Getcal" print CAL data
  "Save" CAL data in to rom*/

void serialEvent() {
  String C;
  C = Serial.readString();
  Serial.println(C);

  if ( C.equals( "Reset\n") ) {
    Serial.println("Reset Calibrate data");
//    resetFlash();
    softReset();
  } else if (C.equals( "Reboot\n") ) {
    Serial.println("Reboot system ");
    softReset();
  } else if  (C.equals( "Getcal\n") ) {
    Serial.println("Read calibite data");
//    bno_stat = 1; // send status to loop5
    //   getCaldata();
  }  else if (C.equals( "Save\n") ) {
    Serial.println("Save Calibite ");
//    bno_stat = 2;
    //   saveCal();

  } else if (C.equals( "Default\n") ) {
    Serial.println("Default Calibite ");
//    DefSensorOffsets();
//    bno_stat = 3;
    //   saveCal();

  } else if (C.equals( "Status\n") ) {
    Serial.println("Get Status ");
//    bno_stat = 4;
    //   saveCal();

  }
  else {
    Serial.println("\"Reset\" reset calibite data");
    Serial.println("\"Reboot\" reboot ");
    Serial.println("\"Getcal\" Get calibite data ");
    Serial.println("\"Save\" Calibite ");
    Serial.println("\"Default\" Default Caldata ");
    Serial.println("\"Status\" Get Status ");
  }
}
