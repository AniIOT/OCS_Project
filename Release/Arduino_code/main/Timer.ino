void TimedFunctionCaller_Init()
{
  currMillis = millis();
  prevMillis1000mstask = currMillis;
  prevMillis100mstask   = currMillis;
  prevMillis50mstask    = currMillis;

  currMicros = micros();
  prevMicros50ustask = currMicros;
  prevMicros100ustask = currMicros;
  prevMicros1000ustask = currMicros;
}

void TimedFunctionCaller()
{
  currMillis = millis();
  currMicros = micros();

  if (currMicros - prevMicros50ustask >= 50){
    //50us task
    prevMicros50ustask = currMicros;
  }

  if (currMicros - prevMicros100ustask >= 100){
    //100us task
    prevMicros100ustask = currMicros;
  }

  if (currMicros - prevMicros1000ustask >= 1000){
    //1000us task
    prevMicros1000ustask = currMicros;
  }

  if (currMillis - prevMillis50mstask >= 50)
  {
    //50ms task
    IMUGetData();
    prevMillis50mstask = currMillis;
  }

  if (currMillis - prevMillis100mstask >= 100)
  {
    //100ms task
    prevMillis100mstask = currMillis;
  }

  if (currMillis - prevMillis1000mstask >= 1000)
  {
    //1000ms task
//    IMUGetData();
    prevMillis1000mstask = currMillis;
  }
}
