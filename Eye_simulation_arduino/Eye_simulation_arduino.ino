// This code is for eye simulation 
// 1. sound detection 
// 2. i2c slave
// 3. serial connection with Raspberry pi
#include <Wire.h>

const int BUFFER_SIZE = 32;
char buf[BUFFER_SIZE];
byte nodePayload[32];

int sensorPin0 = A0; 
int sensorPin1 = A1;
int sensorPin2 = A2;
int sensorPin3 = A3;
int sensorValue0 = 0;
int sensorValue1 = 0;
int sensorValue2 = 0;
int sensorValue3 = 0;

int temp_value1 = 0;
double temp_delay1 = 0;
int temp_value2 = 0;
double temp_delay2 = 0;
int temp_value3 = 0;
double temp_delay3 = 0;

double startTime = micros();

void setup() {
  // put your setup code here, to run once:
  Wire.begin(40);                // join i2c bus with address #4
  Wire.onReceive(receiveEvent); // register event
  Wire.onRequest(sendData);
  Serial.begin(9600);  
  Serial.println("staar"); 
  Serial.println("staar"); 
}

void loop() {
  if (Serial.available() > 0) {
    // read the incoming bytes:
    int rlen = Serial.readBytes(buf, BUFFER_SIZE);
    // prints the received data
    Serial.print("I received: ");
    for(int i = 0; i < rlen; i++)
    {
      Serial.print(buf[i]);
    }
    memset(buf,0,sizeof(buf));     
  }
  
  sensorValue0 = analogRead(sensorPin0);
  sensorValue1 = analogRead(sensorPin1);
  sensorValue2 = analogRead(sensorPin2);
  sensorValue3 = analogRead(sensorPin3);

  temp_value1 = 0;
  temp_delay1 = 0;
  temp_value2 = 0;
  temp_delay2 = 0;
  temp_value3 = 0;
  temp_delay3 = 0;
  startTime = micros();

  // the sound to the nearest sensor is sensor 0
  if (sensorValue0 > 700){
    while ((micros() - startTime) < 5000){
      //delay(1);
      sensorValue1 = analogRead(sensorPin1);
      sensorValue2 = analogRead(sensorPin2);
      sensorValue3 = analogRead(sensorPin3);
      if (sensorValue1 > 500 && temp_value1 < 500){
        temp_value1 = sensorValue1;
        temp_delay1 = (micros() - startTime);
      }
      if (sensorValue2 > 500 && temp_value2 < 500){
        temp_value2 = sensorValue2;
        temp_delay2 = (micros() - startTime);
      }
      if (sensorValue3 > 500 && temp_value3 < 500){
        temp_value3 = sensorValue3;
        temp_delay3 = (micros() - startTime);
      }
    }

    detectSound("A", 0, temp_delay1, temp_delay2, temp_delay3);    
    delay(2500);
  }

  else if (sensorValue1 > 700){
    while ((micros() - startTime) < 5000){
      //delay(1);
      sensorValue0 = analogRead(sensorPin0);
      sensorValue2 = analogRead(sensorPin2);
      sensorValue3 = analogRead(sensorPin3);
      if (sensorValue0 > 500 && temp_value1 < 500){
        temp_value1 = sensorValue0;
        temp_delay1 = (micros() - startTime);
      }
      if (sensorValue2 > 500 && temp_value2 < 500){
        temp_value2 = sensorValue2;
        temp_delay2 = (micros() - startTime);
      }
      if (sensorValue3 > 500 && temp_value3 < 500){
        temp_value3 = sensorValue3;
        temp_delay3 = (micros() - startTime);
      }
    }

    detectSound("B", temp_delay1, 0, temp_delay2, temp_delay3);
    delay(2500);
  }

  else if (sensorValue2 > 700){
    while ((micros() - startTime) < 5000){
      //delay(1);
      sensorValue1 = analogRead(sensorPin1);
      sensorValue0 = analogRead(sensorPin0);
      sensorValue3 = analogRead(sensorPin3);
      if (sensorValue1 > 500 && temp_value1 < 500){
        temp_value1 = sensorValue1;
        temp_delay1 = (micros() - startTime);
      }
      if (sensorValue0 > 500 && temp_value2 < 500){
        temp_value2 = sensorValue0;
        temp_delay2 = (micros() - startTime);
      }
      if (sensorValue3 > 500 && temp_value3 < 500){
        temp_value3 = sensorValue3;
        temp_delay3 = (micros() - startTime);
      }
    }

    detectSound("C", temp_delay1, temp_delay2, 0, temp_delay3);
    delay(2500);
  }

  else if (sensorValue3 > 700){
    while ((micros() - startTime) < 5000){
      //delay(1);
      sensorValue1 = analogRead(sensorPin1);
      sensorValue2 = analogRead(sensorPin2);
      sensorValue0 = analogRead(sensorPin0);
      if (sensorValue1 > 500 && temp_value1 < 500){
        temp_value1 = sensorValue1;
        temp_delay1 = (micros() - startTime);
      }
      if (sensorValue2 > 500 && temp_value2 < 500){
        temp_value2 = sensorValue2;
        temp_delay2 = (micros() - startTime);
      }
      if (sensorValue0 > 500 && temp_value3 < 500){
        temp_value3 = sensorValue0;
        temp_delay3 = (micros() - startTime);
      }
    }

    detectSound("D", temp_delay1, temp_delay2, temp_delay3, 0);
    delay(2500);
  }
 delay(100);
}

// i2c receive data
void receiveEvent(int howMany){
  String myString = "";
  while(Wire.available()) // loop through all but the last
  {
    char c = Wire.read(); // receive byte as a character
    myString.concat(c);
  }
  
  if (myString.length() > 10){
    Serial.println(myString);         // print the integer
  }
}

// i2c send data
void sendData()
{
  Wire.write(buf, 32);
}

// sound detection
void detectSound(String sensor, int s1, int s2, int s3, int s4){
//     Serial.println(sensor);
//     Serial.println(s1);
//     Serial.println(s2);
//     Serial.println(s3);
//     Serial.println(s4);
//    if (s1 == 0){
//       s1 = 9999;
//    }
//    if (s2 == 0){
//        s2 = 9999;  
//    }
//    if (s3 == 0){
//        s3 = 9999;  
//    }
//    if (s4 == 0){
//        s4 = 9999;  
//    }
        
//    if (s1 == 9999 && s2 == 9999 && s3 == 9999 && s4 == 9999){
//        if (sensor == "A"){
//              Serial.println("A");
//        }
//        else if (sensor == "B"){
//              Serial.println("B");
//        }   
//        else if (sensor == "C"){
//              Serial.println("C");
//        }
//        else if (sensor == "D"){
//              Serial.println("D");
//        }
//    }
  if (sensor == "A"  ){
    Serial.println("command_pos_auto_0_-20");
  }
  else if (sensor == "B"){
    Serial.println("command_pos_auto_0_20");
  }  
  else if (sensor == "C" ){
    Serial.println("command_pos_auto_-20_0");
  }
  else if (sensor == "D" ){
    Serial.println("command_pos_auto_20_0");
  }
          
}
