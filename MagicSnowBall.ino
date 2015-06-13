#include <Wire.h>
#include <Adafruit_MMA8451.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_L3GD20.h>
#include <math.h>

//DEFINES
#define DT 0.01f // 100Hz 
// Q diagonal 3x3 with these elements on diagonal
#define Q1 5.0f
#define Q2 100.0f
#define Q3 0.01f
// R diagonal 2x2 with these elements on diagonal
#define R1 1000.0f
#define R2 1000.0f
struct _kalman_data
{
    float x1, x2, x3;
    float p11, p12, p13, p21, p22, p23, p31, p32, p33;
    float q1,q2,q3,r1,r2;
};
typedef struct _kalman_data kalman_data;

//function prototypes
void kalman_innovate(kalman_data *data, float z1, float z2);
void kalman_init(kalman_data *data);
bool accelGetOrientation(sensors_event_t *event, sensors_vec_t *orientation);

//sensors initialization
Adafruit_L3GD20 gyro;
Adafruit_MMA8451 mma = Adafruit_MMA8451();

//variables
double ass_x,ass_y,ass_z;
float pitch,roll;
int nota;
float acc_roll,acc_pitch;
kalman_data pitch_data;
kalman_data roll_data;

void setup(void) {
  
  Serial.begin(9600);
  
  //SETTAGGIO E CALIBRAZIONE ACCELEROMETRO
  Serial.println("Adafruit MMA8451!");
  if (! mma.begin()) {
    Serial.println("Couldnt start");
    while (1);
  }
  Serial.println("MMA8451 found!");
  mma.setRange(MMA8451_RANGE_4_G);
  sensors_event_t eve; 
  mma.getEvent(&eve);
  int numcicl=50;
  double meanx=0;double   meany=0;double  meanz=0;
  for(int i=0;i<numcicl;i++){
    meanx+= eve.acceleration.x;
    meany+= eve.acceleration.y; 
    meanz+= eve.acceleration.z;  
  }
  ass_x=meanx/numcicl; ass_y=meany/numcicl; ass_z=meanz/numcicl;
  Serial.print("Range = "); Serial.print(2 << mma.getRange()); 
  
 //SETTAGGIO GIROSCOPIO
  if (!gyro.begin(gyro.L3DS20_RANGE_250DPS))
  //if (!gyro.begin(gyro.L3DS20_RANGE_500DPS))
  //if (!gyro.begin(gyro.L3DS20_RANGE_2000DPS))
  {
    Serial.println("Oops ... unable to initialize the L3GD20. Check your wiring!");
    while (1);
  }
  acc_roll=0;
  acc_pitch=0;
  kalman_init(&pitch_data);
  kalman_init(&roll_data);
  nota=0;
}

void loop() {

  /* Get a new sensor event */ 
  sensors_event_t event; 
  sensors_vec_t   orientation;
  mma.getEvent(&event);
  
  if (accelGetOrientation(&event, &orientation))
  {
    acc_roll=orientation.roll;
    acc_pitch=orientation.pitch;
    
    gyro.read();
    kalman_innovate(&pitch_data, acc_pitch, gyro.data.x);
    kalman_innovate(&roll_data, acc_roll, gyro.data.y);
   
    pitch = pitch_data.x1;
    roll = roll_data.x1;
 
    if(pitch>0&& pitch<30){
        if(nota!=1){
          Serial.println("DO");
          nota=1;
          tone(9,261.6);
        }
    }if(pitch>30&& pitch<60){
      if(nota!=2){
          Serial.println("RE");
          nota=2;
          tone(9,293.7);
        }
    }if(pitch>60&& pitch<90){
      if(nota!=3){
          Serial.println("MI");
          nota=3;
          tone(9,329.6);
        }
    }if(pitch>90&& pitch<120){
      if(nota!=4){
          Serial.println("FA");
          nota=4;
          tone(9,349.2);
        }
    }if(pitch>120&& pitch<150){
      if(nota!=5){
          Serial.println("SOL");
          tone(9,392.0);
          nota=5;
        }
    }if(pitch>150&& pitch<180){
      if(nota!=6){
          Serial.println("LA");
          nota=6;
          tone(9,440);
        }
    }
    
   }
  
  //delay(50);
}

void kalman_init(kalman_data *data)
{
    data->x1 = 0.0f;
    data->x2 = 0.0f;
    data->x3 = 0.0f;
 
    // Init P to diagonal matrix with large values since
    // the initial state is not known
    data->p11 = 1000.0f;
    data->p12 = 0.0f;
    data->p13 = 0.0f;
    data->p21 = 0.0f;
    data->p22 = 1000.0f;
    data->p23 = 0.0f;
    data->p31 = 0.0f;
    data->p32 = 0.0f;
    data->p33 = 1000.0f;
 
    data->q1 = Q1;
    data->q2 = Q2;
    data->q3 = Q3;
    data->r1 = R1;
    data->r2 = R2;
}	
 
void kalman_innovate(kalman_data *data, float z1, float z2)
{
    float y1, y2;
    float a, b, c;
    float sDet;
    float s11, s12, s21, s22;
    float k11, k12, k21, k22, k31, k32;
    float p11, p12, p13, p21, p22, p23, p31, p32, p33;
 
    // Step 1
    // x(k) = Fx(k-1) + Bu + w:
    data->x1 = data->x1 + DT*data->x2 - DT*data->x3;
    //x2 = x2;
    //x3 = x3;
 
    // Step 2
    // P = FPF'+Q
    a = data->p11 + data->p21*DT - data->p31*DT;
    b = data->p12 + data->p22*DT - data->p32*DT;
    c = data->p13 + data->p23*DT - data->p33*DT;
    data->p11 = a + b*DT - c*DT + data->q1;
    data->p12 = b;
    data->p13 = c;
    data->p21 = data->p21 + data->p22*DT - data->p23*DT;
    data->p22 = data->p22 + data->q2;
    //p23 = p23;
    data->p31 = data->p31 + data->p32*DT - data->p33*DT;
    //p32 = p32;
    data->p33 = data->p33 + data->q3; 
 
    // Step 3
    // y = z(k) - Hx(k)
    y1 = z1-data->x1;
    y2 = z2-data->x2;
 
    // Step 4
    // S = HPT' + R
    s11 = data->p11 + data->r1;
    s12 = data->p12;
    s21 = data->p21;
    s22 = data->p22 + data->r2;
 
    // Step 5
    // K = PH*inv(S)
    sDet = 1/(s11*s22 - s12*s21);
    k11 = (data->p11*s22 - data->p12*s21)*sDet;
    k12 = (data->p12*s11 - data->p11*s12)*sDet;
    k21 = (data->p21*s22 - data->p22*s21)*sDet;
    k22 = (data->p22*s11 - data->p21*s12)*sDet;
    k31 = (data->p31*s22 - data->p32*s21)*sDet;
    k32 = (data->p32*s11 - data->p31*s12)*sDet;
 
    // Step 6
    // x = x + Ky
    data->x1 = data->x1 + k11*y1 + k12*y2;
    data->x2 = data->x2 + k21*y1 + k22*y2;
    data->x3 = data->x3 + k31*y1 + k32*y2;
 
    // Step 7
    // P = (I-KH)P
    p11 = data->p11*(1.0f - k11) - data->p21*k12;
    p12 = data->p12*(1.0f - k11) - data->p22*k12;
    p13 = data->p13*(1.0f - k11) - data->p23*k12;
    p21 = data->p21*(1.0f - k22) - data->p11*k21;
    p22 = data->p22*(1.0f - k22) - data->p12*k21;
    p23 = data->p23*(1.0f - k22) - data->p13*k21;
    p31 = data->p31 - data->p21*k32 - data->p11*k31;
    p32 = data->p32 - data->p22*k32 - data->p12*k31;
    p33 = data->p33 - data->p22*k32 - data->p13*k31;
    //p33 = data->p33 - data->p23*k32 - data->p13*k31;
    data->p11 = p11; data->p12 = p12; data->p13 = p13;
    data->p21 = p21; data->p22 = p22; data->p23 = p23;
    data->p31 = p31; data->p32 = p32; data->p33 = p33;
}

bool accelGetOrientation(sensors_event_t *event, sensors_vec_t *orientation)
{
  /* Make sure the input is valid, not null, etc. */
  if (event == NULL) return false;
  if (orientation == NULL) return false;

  float t_pitch;
  float t_roll;
  float t_heading;
  float signOfZ = event->acceleration.z >= 0 ? 1.0F : -1.0F;

  /* roll: Rotation around the longitudinal axis (the plane body, 'X axis'). -90<=roll<=90    */
  /* roll is positive and increasing when moving downward                                     */
  /*                                                                                          */
  /*                                 y                                                        */
  /*             roll = atan(-----------------)                                               */
  /*                          sqrt(x^2 + z^2)                                                 */
  /* where:  x, y, z are returned value from accelerometer sensor                             */

  t_roll = event->acceleration.x * event->acceleration.x + event->acceleration.z * event->acceleration.z;
  orientation->roll = (float)atan2(event->acceleration.y, sqrt(t_roll)) * 180 / PI;

  /* pitch: Rotation around the lateral axis (the wing span, 'Y axis'). -180<=pitch<=180)     */
  /* pitch is positive and increasing when moving upwards                                     */
  /*                                                                                          */
  /*                                 x                                                        */
  /*            pitch = atan(-----------------)                                               */
  /*                          sqrt(y^2 + z^2)                                                 */
  /* where:  x, y, z are returned value from accelerometer sensor                             */

  t_pitch = event->acceleration.y * event->acceleration.y + event->acceleration.z * event->acceleration.z;
  orientation->pitch = (float)atan2(event->acceleration.x, signOfZ * sqrt(t_pitch)) * 180 / PI;

  return true;
}



