 /*
  VCC     VU (5V USB)   Not available on all boards so use 3.3V if needed.
  GND     G             Ground
  SCL     D1 (GPIO05)   I2C clock
  SDA     D2 (GPIO04)   I2C data
  XDA     not connected
  XCL     not connected
  AD0     not connected
  INT     D8 (GPIO15)   Interrupt pin 
  */



#include <ESP8266WiFi.h>
#include <espnow.h>

#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    #include "Wire.h"
#endif

MPU6050 mpu;
bool dmpReady = false;  // set true if DMP init was successful
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint8_t fifoBuffer[64]; // FIFO storage buffer
Quaternion q;           // [w, x, y, z]         quaternion container
VectorFloat gravity;    // [x, y, z]            gravity vector
float ypr[3];    




// REPLACE WITH RECEIVER MAC Address
uint8_t broadcastAddress[] = { 0xC8, 0xF0, 0x9E, 0xA4, 0x88, 0xCC };




// Structure example to send data
// Must match the receiver structure
struct struct_message {
   byte xAxisValue;
  byte yAxisValue;
  byte zAxisValue; 
};

// Create a struct_message called myData
struct_message myData;

// Callback when data is sent
void OnDataSent(uint8_t *mac_addr, uint8_t sendStatus) {
  Serial.print("Last Packet Send Status: ");
  if (sendStatus == 0) {
    Serial.println("Delivery success\n");
  } else {
    Serial.println("Delivery fail\n");
  }
}
void setupMPU()
{
  // join I2C bus (I2Cdev library doesn't do this automatically)
  #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
      Wire.begin();
      Wire.setClock(400000); // 400kHz I2C clock. Comment this line if having compilation difficulties
  #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
      Fastwire::setup(400, true);
  #endif

  mpu.initialize();
  devStatus = mpu.dmpInitialize();
  // make sure it worked (returns 0 if so)
  if (devStatus == 0) 
  {
      // Calibration Time: generate offsets and calibrate our MPU6050
      mpu.CalibrateAccel(6);
      mpu.CalibrateGyro(6);
      mpu.setDMPEnabled(true);
      dmpReady = true;
  } 
}


void setup() {
  // Init Serial Monitor
  Serial.begin(300);

  // Set device as a Wi-Fi Station
  WiFi.mode(WIFI_STA);

  // Init ESP-NOW
  if (esp_now_init() != 0) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }

  // Once ESPNow is successfully Init, we will register for Send CB to
  // get the status of Trasnmitted packet
  esp_now_set_self_role(ESP_NOW_ROLE_CONTROLLER);
  esp_now_register_send_cb(OnDataSent);

  // Register peer
  esp_now_add_peer(broadcastAddress, ESP_NOW_ROLE_SLAVE, 1, NULL, 0);
   setupMPU();
}

void loop() {
  // Set values to send
 // if programming failed, don't try to do anything
  if (!dmpReady)
  
   return;
  // read a packet from FIFO. Get the Latest packet
  if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) 
  { 
    Serial.println("Hi :)");
     Serial.println("Tarannum and Tarun, I m working fine :)");

    mpu.dmpGetQuaternion(&q, fifoBuffer);
    mpu.dmpGetGravity(&gravity, &q);
    mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
       
        
          

    int xAxisValue = constrain(ypr[2] * 180/M_PI, -90, 90);
    int yAxisValue = constrain(ypr[1] * 180/M_PI, -90, 90);
    int zAxisValue = constrain(ypr[0] * 180/M_PI, -90, 90);    
    myData.xAxisValue = map(xAxisValue, -90, 90, 0, 254); 
    myData.yAxisValue = map(yAxisValue, -90, 90, 0, 254);
    myData.zAxisValue = map(zAxisValue, -90, 90, 0, 254);

  // Send message via ESP-NOW
 String inputData  = inputData + "values " + xAxisValue + "  " + yAxisValue + "  " + zAxisValue;
    Serial.println(inputData);

  // Allow data to get out of the UART
  
  esp_now_send(broadcastAddress, (uint8_t *)&myData, sizeof(myData));
  delay(50);

}
}