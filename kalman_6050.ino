#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>

Adafruit_MPU6050 mpu;


float i;
float ax_offset = 0, ay_offset = 0;
float gx_offset = 0, gy_offset = 0;


float rad2deg = 57.29587;
float roll_v = 0, pitch_v = 0;


float now = 0, lasttime = 0, dt = 0;


float gyro_roll = 0, gyro_pitch = 0;
float acc_roll = 0, acc_pitch = 0;
float k_roll = 0, k_pitch = 0;


float e_P[2][2] = { {1,0},{0,1} };


float k_k[2][2] = { {1,0},{0,1} };
 
void setup() {

    Wire.begin();  //sda>23  scl>5


    Serial.begin(115200);
    delay(100);


    while( !mpu.begin() )
    {
      Serial.println("Fail");
    }
    Serial.println("found!");
mpu.setAccelerometerRange(MPU6050_RANGE_2_G);
mpu.setGyroRange(MPU6050_RANGE_250_DEG);
mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);


  for(i = 1; i<=2000; i++){
    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);
    ax_offset = ax_offset + a.acceleration.x;
    ay_offset = ay_offset + a.acceleration.y;
    gx_offset = gx_offset + g.gyro.x;
    gy_offset = gy_offset + g.gyro.y;
  }
  ax_offset = ax_offset / 2000 ;
  ay_offset = ay_offset / 2000 ;
  gx_offset = gx_offset / 2000 ;
  gy_offset = gy_offset / 2000 ;
  Serial.println("roll:,pitch:");

  delay(100);
}

void loop() {

  now = millis();
  dt = (now - lasttime) / 1000.0;
  lasttime = now;


  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);



  roll_v = (g.gyro.x-gx_offset) + ((sin(k_pitch)*sin(k_roll))/cos(k_pitch))*(g.gyro.y-gy_offset) + ((sin(k_pitch)*cos(k_roll))/cos(k_pitch))*g.gyro.z;
  pitch_v = cos(k_roll)*(g.gyro.y-gy_offset) - sin(k_roll)*g.gyro.z;
  gyro_roll = k_roll + dt*roll_v;
  gyro_pitch = k_pitch + dt*pitch_v;


  e_P[0][0] = e_P[0][0] + 0.0025;
  e_P[0][1] = e_P[0][1] + 0;
  e_P[1][0] = e_P[1][0] + 0;
  e_P[1][1] = e_P[1][1] + 0.0025;

  k_k[0][0] = e_P[0][0]/(e_P[0][0]+0.3);
  k_k[0][1] = 0;
  k_k[1][0] = 0;
  k_k[1][1] = e_P[1][1]/(e_P[1][1]+0.3);




//roll
  acc_roll = atan((a.acceleration.y - ay_offset) / (a.acceleration.z))*rad2deg;
//pitch
  acc_pitch = -1*atan((a.acceleration.x - ax_offset) / sqrt(sq(a.acceleration.y - ay_offset) + sq(a.acceleration.z)))*rad2deg;

  k_roll = gyro_roll + k_k[0][0]*(acc_roll - gyro_roll);
  k_pitch = gyro_pitch + k_k[1][1]*(acc_pitch - gyro_pitch);

//更新協方差矩陣
  e_P[0][0] = (1 - k_k[0][0])*e_P[0][0];
  e_P[0][1] = 0;
  e_P[1][0] = 0;
  e_P[1][1] = (1 - k_k[1][1])*e_P[1][1];

  //Serial.print(0); // To freeze the lower limit
  //Serial.print(" ");
  //Serial.print(90); // To freeze the upper limit
  //Serial.print(" ");
  //Serial.print("roll: ");
  //Serial.print(k_roll);
  //Serial.print(" ");
  //Serial.print("pitch: ");
  Serial.print(k_pitch);
  Serial.print(" ");
  Serial.println(acc_pitch);
  // Serial.println(" ");
  delay(1);
}
