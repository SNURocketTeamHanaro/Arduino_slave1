
/***************************************************************************
 *   Filename       : Avionics.ino
 *   Target machine : Arduino Uno with GY91 sensor module
 *
 *   Maximum stable thruput   : sample / 10 ms
 *   Maximum unstable thruput : sample / 5  ms
 *
 *      Hanaro, SNU
 *      This program will be uploaded to slave arduino
 *      Pinout: connect RST pin to RPi's GPIO No.37
 *      
 ***************************************************************************/

#include <stdio.h>

#include <SD.h>
#include <Wire.h>
#include <SPI.h>
#include <SoftwareSerial.h>

/* External libraries for peripherals */
#include <Adafruit_Sensor.h>
#include <Adafruit_BMP280.h>
#include <FaBo9Axis_MPU9250.h>

/* Mode bit for main parachute deploy mode */
#define REL_PARA_MODE ((byte)0)
#define ABS_PARA_MODE ((byte)1)

/* Calibration for stable boot */
#define INIT_ALT_DUMP_COUNT 10
#define INIT_ALT_COUNT 20

/* Parachute deploy signal */
#define MAIN 6
#define DROGUE 7

/* Data save cycle: Use maximum amount of data storage to compensate delay
 * overhead during the save phase of SD card. Since the saving overhead is
 * very^2 high (30-40 ms), number of saves needs to be minimized. */
#define PERIOD_DATA_RETRIEVAL 5 /* ms */
#define MAX_DATA_NUM 5          /* Maximum data packets stored in the memory before write */
#define SAVE_CYCLE 5            /* # free loops without calling fclose of SD */


/* Structure of the data packet. You may customize the contents in the
 * packet, though the change should be reflected in the decoder simul-
 * taneously. */
struct data_packet_t {
    uint32_t current_time; /* 4          */
    float a[3];            /* 4 * 3 = 12 */
    float g[3];            /* 4 * 3 = 12 */
    //float m[3];            /* 4 * 3 = 12 */ 
    //float temp;            /* 4          */
    //uint32_t press;      /* 4          */
    float alt_press;       /* 4          */
    float alt_max;         /* 4          */
    //byte state;            /* 1          */
};

typedef union data_t {
    struct data_packet_t data_packet;
    char data_packet_string[sizeof(struct data_packet_t)];
    byte data_packet_bytestring[sizeof(struct data_packet_t)];
} data_t;
data_t data[MAX_DATA_NUM];      /* Cache memory for acquired data */


//SoftwareSerial xBee(12, 11);    /* RX, TX */
FaBo9Axis fabo_9axis;           /* I2C */
Adafruit_BMP280 bmp;            /* I2C */

File file_sd;
char file_name[20];
uint32_t datanum = 0;
uint32_t save_cycle = 0;

uint32_t curr_time;
uint32_t prev_time = 0;
float ini_altitude;
//float a_threshold = A_THRESHOLD;
float last_altitude;
float max_altitude;
byte para_mode = REL_PARA_MODE;
uint32_t trigger_apogee = 0;
uint32_t rocket_on = 0;
uint32_t para_drogue = 0;
uint32_t para_main = 0;

char SIGNAL_TURNON = 'a';

void setup() {
  
    /* Serial communication for debug/communication begin 
    When RPi resets the arduino, this block indicates that the arduino has resetted
    by blinking #13 LED three times.*/
    pinMode(13, OUTPUT);
    for (int i=1;i<3;i++){
      digitalWrite(13, HIGH);
      delay(0.5);
      digitalWrite(13, LOW);
      delay(0.5);
    }
    Serial.begin(115200);
    /* After blink, wait untill receiving message from the RPi
     *  When recieved the message '1', LED blink indicates this. 
     */
    while(1){
      if(Serial.available()){
        SIGNAL_TURNON = Serial.read();
        if(SIGNAL_TURNON == '1'){
          digitalWrite(13, HIGH);
          delay(1000);
          digitalWrite(13, LOW);
          break;
        }
      }
    }
    //xBee.begin(38400);  /* Baud higher than 38400 may cause data loss in some Arduinos */
    delay(3000);        /* For Leonardo/Micro */
    Serial.println("[  WORK] INIT CONF");
    //xBee.print("[  WORK] INIT CONF\r\n");

    Wire.begin();

    /* Enable if you want to use ss pin(10) in Arduino Uno as cs pin */
    // pinMode(53, OUTPUT);
    // digitalWrite(53, LOW);

    /* Configure MPU9250 sensor */
    if (fabo_9axis.begin()) {
        fabo_9axis.configMPU9250(MPU9250_GFS_1000, MPU9250_AFS_16G);
        // Serial.println(cd "[STATUS] CONF: FaBo 9Axis I2C Brick");
        // xBee.print("[STATUS] CONF: FaBo 9Axis I2C Brick\r\n");
    } else {
        // Serial.println("[ ERROR] NO FaBo 9Axis I2C Brick");
        Serial.write('b');
        // xBee.print("[ ERROR] NO FaBo 9Axis I2C Brick\r\n");
        while (1)
            ; // Die here
    }

    /* Configure BMP280 sensor */
    if (bmp.begin()) {
        Serial.println("[STATUS] CONF: BMP 280");
        // xBee.print("[STATUS] Configured: BMP 280.\r\n");
    } else {
        // Serial.println("[ ERROR] NO BMP280 (Check wiring!)");
        Serial.write('c');
        // xBee.print("[ ERROR] Device error: BMP280 sensor. (Check wiring!)\r\n");
        while (1)
            ; // Die here
    }

    /* Read altitude out for calibration */
    float tmp1;
    Serial.print("[  WORK] DUMP INIT ALT: ");
    // xBee.print("[  WORK] DUMP INIT ALT: ");
    for (uint32_t i = 0; i < INIT_ALT_DUMP_COUNT; ++i) {
        delay(100);
        tmp1 = bmp.readAltitude();
        Serial.print(tmp1);
        Serial.print(" ");
        // xBee.print(tmp1);
        // xBee.print(" ");
    }
    Serial.print("\n");
    // xBee.print("\r\n");

    /* Evaluate initial altitude offset for calibration */
    float tmp2;
    float calc1 = 0;
    float calc2 = 0;
    float ax, ay, az;
    Serial.print("[  WORK] EVAL INIT ALT: ");
    // xBee.print("[  WORK] EVAL INIT ALT: ");
    for (uint32_t i = 0; i < INIT_ALT_COUNT; ++i) {
        delay(100);
        fabo_9axis.readAccelXYZ(&ax, &ay, &az);
        calc1 += (tmp1 = bmp.readAltitude());
        calc2 += (tmp2 = sqrt(ax * ax + ay * ay + az * az));
        Serial.print(tmp1);
        Serial.print(" ");
        Serial.print(tmp2);
        Serial.print("\t");
        // xBee.print(tmp1);
        // xBee.print(" ");
        // xBee.print(tmp2);
        // xBee.print("\t");
    }
    Serial.print("\n");
    //xBee.print("\r\n");

    calc1 /= INIT_ALT_COUNT;
    max_altitude = last_altitude = ini_altitude = calc1;

    calc2 /= INIT_ALT_COUNT;
//    a_threshold = calc2 + A_THRESHOLD - 1;
    Serial.write('1');  // Send cal. finished signal to RPi.ls
    /* Main loop
     * To optimize performance in the Arduino chip, main loop does not reside
     * on the loop() function. It is better to have a loop in the setup
     * function in order to avoid nasty problems involving function calls. */
      float pres_altitude = 0;
    while (1) {
        curr_time = millis();
        if (curr_time - prev_time >= PERIOD_DATA_RETRIEVAL) {
            prev_time = curr_time;

            /* Calculate A first (optimization)
             * We need to minimize the use of long-living local variables. */
            float ax, ay, az;
            fabo_9axis.readAccelXYZ(&ax, &ay, &az);
            
            /* Retrieve rest of the data */
            float gx, gy, gz;
            //float mx, my, mz;
            float temp;
            fabo_9axis.readGyroXYZ(&gx, &gy, &gz);
            //fabo_9axis.readMagnetXYZ(&mx, &my, &mz);
            fabo_9axis.readTemperature(&temp);

            /* Data acquisition in the in-body DAQ */
            //uint32_t press;
            //press = (uint32_t)analogRead(A0);

            /* Temporal data in stack (optimization) */
            pres_altitude = bmp.readAltitude() - ini_altitude; 
            /* Code for state */
            // byte state = (byte)((rocket_on << 2) + (para_drogue << 1) + para_main);

            /* Package remaining data */
            data[datanum].data_packet.a[0] = ax;
            data[datanum].data_packet.a[1] = ay;
            data[datanum].data_packet.a[2] = az;
            data[datanum].data_packet.g[0] = gx;
            data[datanum].data_packet.g[1] = gy;
            data[datanum].data_packet.g[2] = gz;
            // data[datanum].data_packet.m[0] = mx;
            // data[datanum].data_packet.m[1] = my;
            // data[datanum].data_packet.m[2] = mz;
            // data[datanum].data_packet.temp = temp;
            // data[datanum].data_packet.press = press;
            data[datanum].data_packet.current_time = curr_time;
            data[datanum].data_packet.alt_press = pres_altitude;
            data[datanum].data_packet.alt_max = max_altitude;
            // data[datanum].data_packet.state = state;


            /* Communication mode 1: write to the serial directly */

            /* Send data to the USB serial */
            Serial.write('$'); // Start character
            Serial.write('$'); // Start character
            Serial.write(data[datanum].data_packet_bytestring,
                sizeof(data[datanum].data_packet_bytestring));
            Serial.write('#'); // End character
            Serial.write('#'); // End character
            delay(30);
 
             /* Do this for next data slot in the free cache */
            ++datanum;
        }
        datanum = 0;
        
    }
}

/* For optimization of performance, loop function is not used */
void loop() {
}
