
//NAMA : NURHAN JAVIER GHAZALAH
//NIM  : 25/556004/PA/23331

#include <Adafruit_MPU6050.h> // Akses sensor MPU6050
#include <Adafruit_Sensor.h>  // Framework standar servo
#include <Wire.h>             // Untuk mengomunikasikan I2C
#include <ESP32Servo.h>       // Library untuk mengontrol servo motor 

Servo servo1;                 // Membuat objek servo1
Servo servo2;                 // Membuat objek servo2
Servo servo3;                 // Membuat objek servo3
Servo servo4;                 // Membuat objek servo4
Servo servo5;                 // Membuat objek servo5

#define TRIGMPU_PIN           // Pin triger sensor MPU6050
#define TRIGPIR_PIN 5         // Pin triger PIR
#define SERVO1_PIN 18         // Pin yang terhubung dengan servo1
#define SERVO2_PIN 17         // Pin yang terhubung dengan servo2
#define SERVO3_PIN 16         // Pin yang terhubung dengan servo3
#define SERVO4_PIN 15         // Pin yang terhubung dengan servo4
#define SERVO5_PIN 14         // Pin yang terhubung dengan servo5

int gerakanPIR;               // variabel untuk menyimpan nilai dari PIR (HIGH ATAU LOW)
int minimum = 0;              // batas atas untuk servo berputar (nilai negatif)
int reset = 90;               // variabel yang digunakan untuk me-reset posisi servo
int maksimum = 180;           // batas bawah untuk servo berputar (nilai positif)


Adafruit_MPU6050 mpu;         // mpu sebagai Adafruit_MPU6050

void setup() {

  Serial.begin(115200);      // mengaktifkan serial monitor dengan baudrate 115200

  // test  Adafuit MPU6050 bisa digunakan
  while(!Serial)
    delay(10);
  
  Serial.println("AdaFruit MPU6050 test!");

  if(!mpu.begin())
  while(1){
    delay(10);
  }
  Serial.println("MPU6050 Found!");

  pinMode(TRIGPIR_PIN,OUTPUT); // pin trig dari sensor PIR sebagai output

  // menghubungkan servo ke pin servo
  servo1.attach(SERVO1_PIN);
  servo2.attach(SERVO2_PIN);
  servo3.attach(SERVO3_PIN);
  servo4.attach(SERVO4_PIN);
  servo5.attach(SERVO5_PIN);

  delay(500);
}

void loop() {
  // put your main code here, to run repeatedly:
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);
    // rotasi di sumbu x (ROLL)
    Serial.print("Rotasi X: ");
    Serial.print(g.gyro.x);
    // mengubah nilai rotasi sumbu X dari sensor MPU6050 menjadi nilai sudut servo
    int POSISI1 = map(g.gyro.x, +4.3, -4.3, minimum, maksimum);
    servo1.write(POSISI1);
    int POSISI2 = map(g.gyro.x, +4.3, -4.3, minimum, maksimum);
    servo2.write(POSISI2);
    // rotasi di sumbu y (PITCH)
    Serial.print(" Rotasi y: ");
    Serial.print(g.gyro.y);
    //mengubah nilai rotasi sumbu y dari sensor MPU6050 menjadi nilai sudut servo
    int POSISI3 = map(g.gyro.y, +4.3, -4.3, minimum, maksimum);
    servo3.write(POSISI3);
    int POSISI4 = map(g.gyro.y, +4.3, -4.3, minimum, maksimum);
    servo4.write(POSISI4);
    //rotasi di sumbu z (YAW)
    Serial.print(" Rotasi z: ");
    Serial.print(g.gyro.z);
    //mengubah nilai rotasi sumbu z dari sensor MPU6050 menjadi nilai sudut servo
    int POSISI5 = map(g.gyro.z, -4.3, +4.3, minimum, maksimum);
    servo5.write(POSISI5);

    delay(500);

    // jika servo 5 mutlak sudutnya lebih dari nol servo akan reset ke posisi awal (90 derajat)
    if(abs(POSISI5)> 0){
      delay(1000);
      Serial.println(reset);
      servo5.write(reset);
      g.gyro.z = 0;
      delay(1000);
    }

  // program jika sensor PIR mendeteksi gerakan
  gerakanPIR = digitalRead(TRIGPIR_PIN);      // variable gerakPIR menyimpan nilai yang dibaca oleh sensor PIR
  if(gerakanPIR == HIGH){                     // jika dari hasil pembacaan PIR yang disimpan di gerakPIR mendeteksi gerakan (HIGH)
    // semua servo akan bergerak bebas lalu delay 0.5 detik
    servo1.write(30);
    servo2.write(120);
    servo3.write(30);
    servo4.write(120);
    servo5.write(180);

    delay(500);
    // semua servo akan bergerak kembali ke posisi awal (90 derajat)
    servo1.write(reset);
    servo2.write(reset);
    servo3.write(reset);
    servo4.write(reset);
    servo5.write(reset);

    delay(500);

  }  
  
  
  delay(100); // this speeds up the simulation
}
