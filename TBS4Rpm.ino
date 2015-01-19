#include <TbsShieldRpm.h>

enum SamplingState
{
  SS_Idle,
  
  SS_StartZeroCalibrating,
  SS_ZeroCalibrating,
  
  SS_StartCalibrating,
  SS_Calibrating,
  
  SS_StartSampling,
  SS_Sampling,
  SS_StopSampling,
  SS_SampleOverrun,
};


const uint32_t SamplePeriodUs = 1000; // 1ms 
const uint32_t ReportPeriodUs = 100000; // 100ms
const uint32_t ZeroCalibratePeriodUs = 5000000; // 5s
const uint32_t CalibratePeriodUs = 2000000; // 2s
const uint32_t PongPeriodUs = 10000000; // 10s

const uint8_t CountOutputBuffer = 128;

const String Version = "<version=0.16>";
const uint8_t LedPin = 13;

// variables
String outputData = ""; // for streaming samples
String inputCommand = "";
char lastChar = ' ';
uint32_t prevTime;
uint32_t sampleTimeout = SamplePeriodUs;
uint32_t reportTimeout = ReportPeriodUs;
uint32_t calibrateTimeout = ZeroCalibratePeriodUs;
uint32_t idlePongTimeout = PongPeriodUs;
bool idleBeat = true;

uint8_t lastChannelReady = 0;
SamplingState samplingState = SS_Idle;

TbsShieldRpm tbs(4); // four channels

// report format: 
// kPa-100;1;2,50,1005,912,805;678
// <header> kPa-100
// <count of channels in report> 1
// <channel report> 2,50,1005,912,805
//     <channel id> 2
//     <sample width>> 50
//     <min value> 1005
//     <value> 912
//     <max value> 805
// <check sum> 678
//
// samples:
// 1;2,50,1005,912,805;678<etx>
// 4;0,50,1005,912,805;1,50,1005,912,805;2,50,1005,912,805;3,50,1005,912,805;1678<etx>
//
void SendReport(uint8_t channelsReady)
{
  uint16_t prevLength = outputData.length();
  
  outputData += "$";
  outputData += channelsReady;

  for (uint8_t channelIndex = 0; channelIndex < tbs.ChannelCount(); ++channelIndex)
  {
    if (tbs.SampleForCycleReady(channelIndex))
    {
      int16_t value;
      outputData += ";";
      // channel id
      outputData += channelIndex; 
      
      // cycle width
      outputData += ",";
      outputData += tbs.SampleWidth(channelIndex);
      
      // min
      outputData += ",";
      //value = tbs.SampleMin(channelIndex);
      value = tbs.SampleMinKpa100(channelIndex);
      outputData += value;
      
      // using running average
      outputData += ",";
      //value = tbs.SampleAverage(channelIndex);
      value = tbs.SampleAverageKpa100(channelIndex);
      outputData += value;
      
      // max
      outputData += ",";
      //value = tbs.SampleMax(channelIndex);
      value = tbs.SampleMaxKpa100(channelIndex);
      outputData += value;
    }
  }

  {
    // calc checksum
    uint16_t sum = 0;
    uint16_t count = outputData.length();
    for (int index = prevLength; index < count; index++)
    {
      sum += outputData.charAt(index);
    }
    
    // add checksum 
    outputData += ";";
    outputData += sum;
  }

  // add ETX and CR LF
  outputData += "\x03\r\n"; // ETX 
}

boolean ServiceReport()
{
  const uint8_t countService = 128;
  uint16_t length = outputData.length();
  boolean needsService = (length > 0);
  if (needsService)
  {
    if (length > countService)
    {
      length = countService;
    }
    Serial.print(outputData);
    outputData = "";
  }
  return needsService;
}

void setup()
{
  // comms setup
  Serial.begin(19200);
  
  bool stateLed = HIGH;
  while (!Serial) 
  {
     digitalWrite(LedPin, HIGH);
     delay(250);
     digitalWrite(LedPin, LOW);
     delay(250);
     ; // wait for serial port to connect. Needed for Leonardo only
   }
  
  inputCommand.reserve(80);
  inputCommand = "";
    
  outputData.reserve(CountOutputBuffer);
  outputData = "";
  
  tbs.Setup();
  
  // sample timer init
  prevTime = micros();
}

void loop()
{
  uint32_t nowTime = micros();
  uint32_t deltaTime = nowTime - prevTime;
  
   
  switch (samplingState)
  {
  case SS_Idle:
    if (deltaTime > idlePongTimeout)
    {
      idlePongTimeout = PongPeriodUs;
      if (idleBeat)
      {
        Serial.println("<idle0>");
      }
      else
      {
        Serial.println("<idle1>");
      }
      idleBeat = !idleBeat;
    }
    else
    {
      idlePongTimeout -= deltaTime;
    }
    break;
    
  case SS_StartZeroCalibrating:
    calibrateTimeout = ZeroCalibratePeriodUs;
    sampleTimeout = SamplePeriodUs;
    tbs.ReadSamples(); // ignore first sample read
    tbs.ClearSamples();
    Serial.println("<calibrating...>");
    samplingState = SS_ZeroCalibrating;
    break;
    
  case SS_ZeroCalibrating:
    if (deltaTime > sampleTimeout)
    {
      sampleTimeout = SamplePeriodUs;
      tbs.ReadSamples();
    }
    else
    {
      sampleTimeout -= deltaTime;
    }
    if (deltaTime > calibrateTimeout)
    {
      samplingState = SS_Idle;
      tbs.CalibrateAtZeroWithSamples();
      Serial.println("<...calibrated>");
    }
    else
    {
      calibrateTimeout -= deltaTime;
    }
    break;

  case SS_StartCalibrating:
    calibrateTimeout = CalibratePeriodUs;
    sampleTimeout = SamplePeriodUs;
    tbs.ReadSamples(); // ignore first sample read
    tbs.ClearSamples();
    Serial.println("<min=-50.0 kPa>");
    Serial.println("<max=0.0 kPa>");
    Serial.println("<resolution=0.05 kPa>");
    samplingState = SS_Calibrating;
    break;
    
  case SS_Calibrating:
    if (deltaTime > sampleTimeout)
    {
      sampleTimeout = SamplePeriodUs;
      tbs.ReadSamples();
    }
    else
    {
      sampleTimeout -= deltaTime;
    }
    if (deltaTime > calibrateTimeout)
    {
      samplingState = SS_StartSampling;
      tbs.CalibrateCyclesWithSamples();
    }
    else
    {
      calibrateTimeout -= deltaTime;
    }
    break;
    
  case SS_StartSampling:
    sampleTimeout = SamplePeriodUs;
    reportTimeout = ReportPeriodUs;
    tbs.ClearSamples();

    Serial.println("<running>");
    samplingState = SS_Sampling;
    outputData = "";
    nowTime = micros(); // reset
    break;
    
  case SS_Sampling:
    // split into fibers of work so as to keep work spread
    // across time so sampling happens at consistent time frequency
    //
    if (deltaTime > sampleTimeout)
    {
      if (deltaTime > SamplePeriodUs)
      {
        samplingState = SS_SampleOverrun;
      }
      // read samples and reset time out to period - time past last period
      sampleTimeout = SamplePeriodUs - (deltaTime - sampleTimeout);
      lastChannelReady = tbs.ReadSamplesCycle();
    }
    else
    {
      sampleTimeout -= deltaTime;
      
      if (lastChannelReady > 0)
      {
        // update a report on channels ready
        SendReport( lastChannelReady );
        lastChannelReady = 0;
      }
      else
      {
        // send out reports in chewable pieces
        ServiceReport();
      }
    }
    break;
    
  case SS_SampleOverrun:
  case SS_StopSampling:
    if (!ServiceReport())
    {
      if (SS_SampleOverrun == samplingState)
      {
        Serial.println("<overrun>");
      }
      Serial.println("<stopped>");

      samplingState = SS_Idle;
    }
    break;
  }
    
  prevTime = nowTime;
  
  WorkAroundSerialEvent();
}

void WorkAroundSerialEvent()
{
  static uint8_t stateLed = HIGH;
  
  while (Serial.available())
  {
    digitalWrite(LedPin, stateLed);
    stateLed != stateLed;
    
    char inputChar = (char)Serial.read();
   
    if (lastChar == '\r' && inputChar == '\n' )
    {
      inputCommand.trim();
      if (samplingState == SS_ZeroCalibrating)
      {
        Serial.println("<still calibrating>");
      }
      else if (samplingState == SS_StartSampling ||
          samplingState == SS_StartZeroCalibrating ||
          samplingState == SS_StopSampling)
      {
        Serial.println("<still transitioning>");
      }
      else
      {
        // process command
        if (inputCommand == "start")
        {
          samplingState = SS_StartCalibrating;
        }
        else if (inputCommand == "stop")
        {
          samplingState = SS_StopSampling;
        }
        else if (inputCommand == "calibrate")
        {
          samplingState = SS_StartZeroCalibrating;
        }
        else if (inputCommand == "query version")
        {
          Serial.println(Version);
        }
        else
        {
          Serial.print("?");
          Serial.println(inputCommand);
        }
      }
      // prepare for next command
      inputCommand = "";
    }
    else
    {
      lastChar = inputChar;
      inputCommand += inputChar;
    }
  }
}
