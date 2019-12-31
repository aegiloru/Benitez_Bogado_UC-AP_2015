#include <FastSerial.h>
#include "I2Cdev.h"
#include <HMC5883L.h>
#include <TinyGPS.h>
#include <IntegralSmooth.h>
#include <Filters.h>
#include <MedianFilter.h>
#include <MadgwickAHRS.h>
#include <MPU6050_6Axis_MotionApps20.h>
MedianFilter CH1_mf(5,15);
MedianFilter CH2_mf(5,15);
MedianFilter CH4_mf(5,15);
MedianFilter CH3_mf(5,15);
MedianFilter CH5_mf(5,15);
MedianFilter CH6_mf(5,15);
#include <FuerzasyAerodinamica.h>
#include <PinChangeInt.h>
#include <Servo.h>
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
#include "Wire.h"
#endif
FastSerialPort0(Serial);
FastSerialPort2(Serial2);
FastSerialPort3(Serial3);
FilterOnePole ALTLowpass(LOWPASS,50);
#include <MS561101BA.h>
#define MOVAVG_SIZE 5
MS561101BA baro = MS561101BA();
float altitude_error_i=0,vel_estimada=0,vel_X,vel_Y,cont_TV=0;
float movavg_buff[MOVAVG_SIZE];
int movavg_i=0;
Servo esc1;
Servo esc2;
Servo esc3;
Servo esc4;
const float DEG_RAD=(PI/180);
const float RAD_DEG=(180/PI);
int16_t ax, ay, az;
int16_t _gx, _gy, _gz;
int16_t _mx, _my, _mz;
float press, temp, refPress, altura=0,ref_alt=0;
int32_t rawPress_t;
float KI=0.05;
float KP1=0;
float KP2=0.9;
float alt_estimada=0;
float alt_ant=0;
float KP_sx,KP_sy,KP_sz,KP_rx,KP_ry,KP_rz,KI_rx,KI_ry,KI_rz,KD_rx,KD_ry;
#define CH1_roll A9
#define CH2_pitch A10
#define CH3_throttle A8
#define CH4_yaw A11
#define CH5_aux A12
#define ECHO_aux A14
#define CH6_aux A13
#define CH1_FLAG 1
#define CH2_FLAG 2
#define CH3_FLAG 4
#define CH4_FLAG 8
#define CH5_FLAG 16
#define ECHO_FLAG 32
#define CH6_FLAG 64
#define eu 2.718281828
volatile uint8_t bUpdateFlagsShared;
volatile uint16_t CH1_FSHARED;
volatile uint16_t CH2_FSHARED;
volatile uint16_t CH3_FSHARED;
volatile uint16_t CH4_FSHARED;
volatile uint16_t CH5_FSHARED;
volatile uint16_t CH6_FSHARED;
bool dmpReady = false;  // si se puede iniciar el DMP cambia su valor a "true"
uint8_t devStatus;      // status del dispositivo 0 = OK !0 = NOT OK 
uint16_t packetSize;    // Tamaño del paquete DMP 42 bytes standar
uint16_t fifoCount;     // cuenta los bytes que se encuentran en el FIFO
uint8_t fifoBuffer[64]; // FIFO buffer de almacenamiento
uint8_t mpuIntStatus;
// orientation/motion vars
Quaternion q;           // [w, x, y, z]         contenedor del cuaternion
VectorInt16 aa;         // [x, y, z]            medida de los valores de aceleracion
VectorInt16 aaReal;     // [x, y, z]            aceleracion quitando la gravedad
VectorInt16 aaWorld; 
VectorInt16 gg;
VectorFloat gravity; // [x, y, z]            aceleracion pasado al sistema del V1
float ypr[3];    
volatile bool mpuInterrupt = false;     
void dmpDataReady() {
  mpuInterrupt = true;
}

uint32_t CH1_start;
uint32_t CH2_start;
uint32_t CH3_start;
uint32_t CH4_start;
uint32_t CH5_start;
uint32_t CH6_start;
#ifndef CPU_FREQ
#define CPU_FREQ 16000000L
#endif



#define MPUREG_USER_CTRL 0x6A
#define MPUREG_INT_PIN_CFG 0x37 
#define MPUREG_PWR_MGMT_1 0x6B // 
#define BIT_H_RESET 0x80
#define MPU_CLK_SEL_PLLGYROX 0x01 

MadgwickAHRS MFilter;
IntegralSmooth accZ_smooth(0.993,50);
IntegralSmooth accX_smooth(0.993,70);
IntegralSmooth accY_smooth(0.993,70);
HMC5883L compass;
FuerzasyAerodinamica FM(1.07e-6,11.1e-6,0.23,0,0,0,0);
MPU6050 mpu;
TinyGPS gps;
int gps_cont=0;
int T100hz_cont=0;
int T66hz_cont=0;
float gz_des;
float normG=1;
double time1,time2,time_r,time_y,time_v,time_loop,last_time=0;
;

float dt,dt_r,dt_y,dt_v,dt_loop;
float U1=0,U2=0,U3=0,U4=0,U1_n=0,Z=0;
float V_x=0, V_y=0, V_z=0;
float V_gps=0;
int16_t GyroOffsetX,GyroOffsetY,GyroOffsetZ;
float thXAxis,thYAxis,thZAxis;
Vector acccomp;
Vector na_tierra;
Vector na_tierra_ant;
VectorX qu;
VectorX qur;
VectorX qu_des;
Omega r;
PWMX VM;
int c=0,b=0,s_cont=0,max_vspeed=250;
boolean ST=0,ALT_HOLD=0,POS_HOLD=0,SAFE_RETURN=0;
void Place(VectorX &qs){ 
  qs.qt0 = 1;
  qs.qt1 = 0;
  qs.qt2 = 0;
  qs.qt3 = 0; 
  qs.qt4 = 0; 
  qs.qt5 = 0; 
  qs.qt6 = 0; 
  qs.qt7 = 0; 
  qs.qt8 = 0;
}
float gx,gy,gz,uigz=0;
float Pitch = 0;
float Roll = 0;
float Yaw=0;
float Yaw_des=0;
unsigned char ini=0;
float roll_des=0,pitch_des=0,yaw_des=0;
static uint8_t 
read_register(uint8_t address, uint8_t reg_addr)
{
  uint8_t ret = 0;

  Wire.beginTransmission(address);
  Wire.write(reg_addr);     //sends address to read from
  if (0 != Wire.endTransmission())
    return 0;

  Wire.requestFrom(address, uint8_t(1));    // request 1 byte from device
  if( Wire.available() ) {
    ret = Wire.read();  // receive one byte
  }
  if (0 != Wire.endTransmission())
    return 0;

  return ret;
}

static bool 
write_register(byte address, int reg_addr, uint8_t value)
{
  Wire.beginTransmission(address);
  Wire.write(reg_addr);
  Wire.write(value);
  if (0 != Wire.endTransmission())
    return false;
  delay(10);
  return true;
} 

void init_mpu650(uint8_t addr)
{
  uint8_t user_ctrl;
  user_ctrl = read_register(addr, MPUREG_USER_CTRL);
  user_ctrl = user_ctrl & ~(1 << 5); // reset I2C_MST_EN bit
  write_register(addr, MPUREG_USER_CTRL,user_ctrl);
  delay(100);
  user_ctrl = read_register(addr, MPUREG_INT_PIN_CFG);
  user_ctrl = user_ctrl | (1 << 1); // set I2C_BYPASS_EN bit
  write_register(addr, MPUREG_INT_PIN_CFG,user_ctrl);
}


void setup() 
{

  Place(qur);
  Place(qu_des);

  Serial.begin(115200);
  Serial2.begin(38400);
  Serial3.begin(57600);

  esc1.attach (3);
  esc2.attach (2);
  esc3.attach (5);
  esc4.attach (6);
  esc1.writeMicroseconds(1050);
  esc2.writeMicroseconds(1050);
  esc3.writeMicroseconds(1050);
  esc4.writeMicroseconds(1050);
  pinMode (13,OUTPUT);
  digitalWrite(13,HIGH);
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
  Wire.begin();
#elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
  Fastwire::setup(400, true);
#endif
  KP_rx=0.75,KP_ry=0.75,KP_rz=0.75,KI_rx=0.35,KI_ry=0.35,KI_rz=0.21,KD_rx=0.01,KD_ry=0.01,KP_sx=3,KP_sy=3,KP_sz=2;

  TWBR = 8; //set i2c communication to 500khz1 (CPU_FRECUENCY/(16+2*TWBR))
  NewScan();
  compass.initialize();


  delay(100);
  while(!mpu.testConnection()){
    mpu.initialize();
  }

  devStatus = mpu.dmpInitialize();
  mpu.setMasterClockSpeed(13);
  delay(100);
  mpu.setXGyroOffset(162);
  mpu.setYGyroOffset(82);
  mpu.setZGyroOffset(-119);
  mpu.setXAccelOffset(-743);
  mpu.setYAccelOffset(-2694);
  mpu.setZAccelOffset(2236); 

  if (devStatus == 0) {
    mpu.setDMPEnabled(true);
    attachInterrupt(0, dmpDataReady, RISING);
    mpuIntStatus = mpu.getIntStatus();
    dmpReady = true;
    packetSize = mpu.dmpGetFIFOPacketSize();
  } 
  

  while (!compass.testConnection()){
    compass.initialize();
    init_mpu650(0x68);
  }

  baro.init(MS561101BA_ADDR_CSB_LOW); 

  analogReference(INTERNAL2V56);
  PCintPort::attachInterrupt(CH1_roll,calc_CH1,CHANGE);
  PCintPort::attachInterrupt(CH2_pitch,calc_CH2,CHANGE);
  PCintPort::attachInterrupt(CH3_throttle,calc_CH3,CHANGE);
  PCintPort::attachInterrupt(CH4_yaw, calc_CH4,CHANGE);
  PCintPort::attachInterrupt(CH5_aux, calc_CH5,CHANGE);
  PCintPort::attachInterrupt(CH6_aux, calc_CH6,CHANGE);

}
void NewScan()
{

  for(int i=0;i<128;i++)  
  {
    Wire.requestFrom(i, 1);
    while(Wire.available())
    { 
      byte c = Wire.read();
      switch (i)
      {  
      case 0x1E:
        break;
      case 0x68:  
        init_mpu650(0x68); 
        break;
      case 0x69:  
        init_mpu650(0x69); 
        break;
      default: 
        Serial.println(" unknown device!");
        break;
      }  
    }
    switch (i)
    { 
    case 0x76: 
      if (read_register(0x76, 0xA2)!=0) { 

      }  
      break;
    case 0x77: 	
      if (read_register(0x77, 0xA2)!=0) { 

      }  
      break;
    }
  }

}

void calibrateGyro(int samples)
{
  while (1){
    int ready=0;
    float sumX = 0;
    float sumY = 0;
    float sumZ = 0;
    float sigmaX = 0;
    float sigmaY = 0;
    float sigmaZ = 0;

    for (int i = 0; i < samples; ++i)
    {


      mpu.getRotation(&gg.x,&gg.y,&gg.z);
      sumX += gg.x;
      sumY += gg.y;
      sumZ += gg.z;

      sigmaX += gg.x * gg.x;
      sigmaY += gg.y * gg.y;
      sigmaZ += gg.z * gg.z;

      delay(3);
    }

    GyroOffsetX = sumX / samples;
    GyroOffsetY = sumY / samples;
    GyroOffsetZ = sumZ / samples;
    thXAxis = sqrt(abs((sigmaX / samples) - (GyroOffsetX*GyroOffsetX)));
    thYAxis = sqrt(abs((sigmaY / samples) - (GyroOffsetY*GyroOffsetY)));
    thZAxis = sqrt(abs((sigmaZ / samples) - (GyroOffsetZ*GyroOffsetZ)));
    for (int i = 0; i < 6; ++i){
      mpu.getRotation(&gg.x,&gg.y,&gg.z);
      if (abs(gg.x-GyroOffsetX)<=1) ready++;
      if (abs(gg.y-GyroOffsetY)<=1) ready++;
      if (abs(gg.z-GyroOffsetZ)<=1) ready++;
      delay(200);
    }
    gg.x=0;
    gg.y=0;
    gg.z=0;
    if (ready==18) break; 
  }
}

float mx,my,mz,naXAxis,naYAxis,naZAxis;
void acc_compensado(VectorX &qr, Vector &aux){
  float g0=(2*(qr.qt1*qr.qt3-qr.qt0*qr.qt2))*normG;
  float g1=(2*(qr.qt3*qr.qt2+qr.qt1*qr.qt0))*normG;
  float g2=(qr.qt0*qr.qt0-qr.qt1*qr.qt1-qr.qt2*qr.qt2+qr.qt3*qr.qt3)*normG;
  aux.Xaxis-=g0;
  aux.Yaxis-=g1;
  aux.Zaxis-=g2;
};
void acc_tierra(VectorX qr, Vector &aux, Vector &acc){
  VectorX q1_aux;
  VectorX q2_aux;
  q1_aux.qt0=0;
  q1_aux.qt1=aux.Xaxis;
  q1_aux.qt2=aux.Yaxis;
  q1_aux.qt3=aux.Zaxis;

  //qr=q_conjug(qr);
  q2_aux=q_mult(qr,q1_aux);
  qr=q_conjug(qr);
  q2_aux=q_mult(q2_aux,qr);
  acc.Xaxis=q2_aux.qt1;
  acc.Yaxis=q2_aux.qt2;
  acc.Zaxis=q2_aux.qt3;
};
void quaternion_Smooth (float &q0, float &q1, float &q2,float &q3, float &dt)
{

  /* Filtro pasa bajo para cuaternion */


  const float RC = 0.02;
  float alpha = dt / (RC + dt);
  float qr0,qr1,qr2,qr3;


  qr0 = ((alpha * q0) + (1.0 - alpha) * qur.qt0);
  qr1 = ((alpha * q1) + (1.0 - alpha) * qur.qt1);
  qr2 = ((alpha * q2) + (1.0 - alpha) * qur.qt2);
  qr3 = ((alpha * q3) + (1.0 - alpha) * qur.qt3);
  float norm=1/sqrt(qr0*qr0+qr1*qr1+qr2*qr2+qr3*qr3);
  qur.qt0=qr0*norm;
  qur.qt1=qr1*norm;
  qur.qt2=qr2*norm;
  qur.qt3=qr3*norm;
}
void LowpassFilter (float &value,float &value_ant,float &dt,float RC)
{

  /* Filtro pasa bajo */

  float alpha = dt / (RC + dt);
  value = ((alpha * value) + (1.0 - alpha) * value_ant);
}
int m_cont=4,p_cont=0;
float az_ant;
float alt_gps=0,alt_relative=0,yaw_gps;
float heading=0, yaw_cf=0,r_yaw=0,lat_ref,lon_ref;
float KPd=0;
float lat_gps, lon_gps,x_int=0,y_int=0,x_gps=0,y_gps=0,Vx_gps=0,Vy_gps=0,course_gps=0,distance_gps=0,distance_ant=0,lat_des=0,lon_des=0;
boolean Check_GPS=0;
int Coord_OK=0;
void GPS(void){
  long lat,lon,satelites;
  unsigned long altitud, curso;
  while (Serial2.available()){ 
   

   if (gps.encode(Serial2.read())){ 
        Coord_OK=0;  
        gps.get_position(&lat,&lon);
        V_gps=gps.f_speed_mps();
        if ((lon!=999999999) && (lat!=999999999) && lon<0 && lat<0){
          lat_gps=(lat)*0.000001;
          lon_gps=(lon)*0.000001;
            
          if (ST==0){
            lat_ref=lat_gps;
            lon_ref=lon_gps;
          } 
        }
        
    }
  }
  ++Coord_OK;
      if (Coord_OK>10){
        Coord_OK=10;
      }
  if (Coord_OK<10){
    Check_GPS=1;
  }else {
    Check_GPS=0;
  } 
}
float X_dotdot=0,Y_dotdot=0,Y_dot=0,X_dot=0,mass=1.48;
int fifoin=0;
void c_dinamica(void){


  if (mpu.getFIFOCount()>=packetSize){
    fifoin=1;
    fifoCount = mpu.getFIFOCount();
    if (fifoCount >= 1024) {
      mpu.resetFIFO();
      Serial.println("Fifo Overflow");
 } 
    else { 
      mpu.getFIFOBytes(fifoBuffer, packetSize);
      mpu.dmpGetQuaternion(&q, fifoBuffer);
      mpu.dmpGetAccel(&aa, fifoBuffer);
      mpu.dmpGetGravity(&gravity, &q);
      mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
      mpu.dmpGetLinearAccel(&aaReal, &aa, &gravity);
      mpu.dmpGetLinearAccelInWorld(&aaWorld, &aaReal, &q);
    } 
  }
  else fifoin=0;    
  mpu.getRotation(&gg.x, &gg.y, &gg.z);
  gx=(gg.x-GyroOffsetX)*(DEG_RAD)*0.0609756f;
  gy=(gg.y-GyroOffsetY)*(DEG_RAD)*0.0609756f;
  gz=-1*(gg.z-GyroOffsetZ)*(DEG_RAD)*0.0609756f;
  if (q.w>0){
    qur.qt0=(float)q.w;
    qur.qt1=(float)q.x;
    qur.qt2=(float)q.y;
    qur.qt3=(float)q.z;
  } 
  else {
    qur.qt0=-1*(float)q.w;
    qur.qt1=-1*(float)q.x;
    qur.qt2=-1*(float)q.y;
    qur.qt3=-1*(float)q.z;
  }
  Yaw=atan2((2*qur.qt0*qur.qt3 + 2*qur.qt1*qur.qt2) , (-2*qur.qt3*qur.qt3 - 2*qur.qt2*qur.qt2 +1));  
  float my_r=mx*(2*(qur.qt1*qur.qt2+qur.qt0*qur.qt3))+my*(qur.qt0*qur.qt0-qur.qt1*qur.qt1+qur.qt2*qur.qt2-qur.qt3*qur.qt3) +mz*(2*qur.qt2*qur.qt3-2*qur.qt0*qur.qt1);
  float mx_r=mx*(qur.qt0*qur.qt0+qur.qt1*qur.qt1-qur.qt2*qur.qt2-qur.qt3*qur.qt3) + my*(2*(qur.qt1*qur.qt2-qur.qt0*qur.qt3)) + mz*(2*(qur.qt0*qur.qt2+qur.qt1*qur.qt3));
  mx=mx_r*cos(-Yaw)-my_r*sin(-Yaw);// pasando la vel_Tierra al vehiculo V1
  my=my_r*cos(-Yaw)+mx_r*sin(-Yaw);
  heading = atan2(my, mx);       
  float declinationAngle = -0.27605291;
  heading += declinationAngle;
  if (heading>PI){
    heading-=2*PI;
  }
  if (heading<-PI){
    heading+=2*PI;
  }
  if (dt_y==0){
    dt_y=0.01;
  }
  else{ 
    dt_y=(micros()-time_y)*1e-6;
  }
  time_y=micros(); 

  if (ini<120 || (abs(heading-yaw_cf)>2)){
    yaw_cf=heading;
  }
  else{
    yaw_cf=(yaw_cf+gz*dt_y)*0.994+heading*0.006;
    if (yaw_cf>PI){
      yaw_cf-=2*PI;
    }
    if (yaw_cf<-PI){
      yaw_cf+=2*PI;
    }
  }

  
  KP1=(1-exp(-0.000815*abs(vel_estimada)))*0.9993;
  if(ST==1){
    alt_estimada=(altura-ref_alt)*(KP1+0.0007) + alt_ant*(0.9993-KP1);
  }
  alt_ant=alt_estimada;
}


VectorX q_conjug(VectorX q){
  q.qt0=q.qt0;
  q.qt1=-1*q.qt1;
  q.qt2=-1*q.qt2;
  q.qt3=-1*q.qt3;
  return q;
}
VectorX q_mult(VectorX &q1, VectorX &q2){
  VectorX q;
  q.qt0=(q1.qt0*q2.qt0)-(q1.qt1*q2.qt1)-(q1.qt2*q2.qt2)-(q1.qt3*q2.qt3);
  q.qt1=(q1.qt0*q2.qt1)+(q1.qt1*q2.qt0)+(q1.qt2*q2.qt3)-(q1.qt3*q2.qt2);
  q.qt2=(q1.qt0*q2.qt2)-(q1.qt1*q2.qt3)+(q1.qt2*q2.qt0)+(q1.qt3*q2.qt1);
  q.qt3=(q1.qt0*q2.qt3)+(q1.qt1*q2.qt2)-(q1.qt2*q2.qt1)+(q1.qt3*q2.qt0);
  return q;
}
void qcalc_des(float psi_des,float theta_des,float phi_des,VectorX &qdes){
  float Cos_phi=cos(phi_des*0.5);
  float Cos_theta=cos(theta_des*0.5);
  float Cos_psi=cos(psi_des*0.5);
  float Sin_phi=sin(phi_des*0.5);
  float Sin_theta=sin(theta_des*0.5);
  float Sin_psi=sin(psi_des*0.5);

  qdes.qt0=Cos_phi*Cos_theta*Cos_psi+Sin_phi*Sin_theta*Sin_psi;
  qdes.qt1=Sin_phi*Cos_theta*Cos_psi-Cos_phi*Sin_theta*Sin_psi;
  qdes.qt2=Cos_phi*Sin_theta*Cos_psi+Sin_phi*Cos_theta*Sin_psi;
  qdes.qt3=Cos_phi*Cos_theta*Sin_psi-Sin_phi*Sin_theta*Cos_psi;
  if (qdes.qt0<0){
    qdes.qt0=-1*qdes.qt0;
    qdes.qt1=-1*qdes.qt1;
    qdes.qt2=-1*qdes.qt2;
    qdes.qt3=-1*qdes.qt3;
  }
}


float errork2=0,errork3=0,errork4=0,uik2=0,uik3=0,uik4=0,ui_p=0,ui_q=0,ui_r=0,e_p=0,e_q=0,e_r=0,ui2=0,ui3=0,gx_des=0,gy_des=0;
float E_x,E_y,E_z,E_gx,E_gy,E_gz,a_error,a_sin,E_max;
float altura_b=0;

VectorX qu_m,qu_n;
VectorX n;
float U1_z=0, Z_des=0, E_alt=0,V_des=0, uiZ=0, E_vel_ant=0, gz_ant=0,gx_ant=0,gy_ant=0, Z_ant=0,gx_error=0,gy_error=0,gx_d=0,gx_dant=0,gy_d=0,gy_dant=0,vel_dP_ant=0;
int alt_cont=0;
void q_rate(void){
  float Mx=0,My=0,Mz=0,uip=0,uiq=0,uir=0;
  E_gx=gx_des-gx;
  E_gy=gy_des-gy;
  E_gz=(gz_des-gz)*-1;
  if (1000*dt_r==0){
    dt_r=dt_y;
  }
  else{
    dt_r=(micros()-time_r)*1e-6;
  }  
  time_r=micros();
  uip=ui_p+KI_rx*dt_r*E_gx;
  uip=max(-1,min(1,uip));
  uiq=ui_q+KI_ry*dt_r*E_gy;
  uiq=max(-1,min(1,uiq));
  uir=ui_r+KI_rz*dt_r*E_gz;
  uir=max(-1,min(1,uir));
  gx_d=KD_rx*(E_gx-e_p)*(1/dt_r);
  gy_d=KD_ry*(E_gy-e_q)*(1/dt_r);
  LowpassFilter(gx_d,gx_dant,dt_r,0.0088);
  LowpassFilter(gy_d,gy_dant,dt_r,0.0088);
  gx_dant=gx_d;
  gy_dant=gy_d;
  U2=KP_rx*E_gx+uip+gx_d;
  U3=KP_ry*E_gy+uiq+gy_d;
  U4=KP_rz*E_gz+uir;
  ui_p=uip;
  ui_q=uiq;
  ui_r=uir;
  e_p=E_gx;
  e_q=E_gy;
  e_r=E_gz;
  if (ALT_HOLD==1){
    if (dt_v==0){
      dt_v=dt;
    }
    else{ 
      dt_v=(micros()-time_v)*1e-6;
    } 
    time_v=micros();

    float E_vel=V_des-vel_estimada;
    float vel_dP=(E_vel-E_vel_ant)*(1/dt_v);
    LowpassFilter(vel_dP,vel_dP_ant,dt_v,0.0088);
    vel_dP_ant=vel_dP;
    uiZ=uiZ+0.3*E_vel*dt_v;
    uiZ=min(400,max(-400,uiZ));
    U1_z=(uiZ+E_vel*2.9+0.0015*vel_dP)*0.01;
    E_vel_ant=E_vel; 
  }
  else {
    U1_z=0;
    uiZ=0;
    E_vel_ant=0;
    dt_v=0;
  }
  if (E_max<(50)){
    U1=(U1_n+U1_z)/(qur.qt0*qur.qt0-qur.qt1*qur.qt1-qur.qt2*qur.qt2+qur.qt3*qur.qt3);
  }
  else{
    U1=(U1_n+U1_z);
  }
  U1=max(-21,min(21,U1));
  r= FM.calcRotX(U1,U2,U3,U4);
  VM=FM.calcPWM(r.Omega_1,r.Omega_2,r.Omega_3,r.Omega_4);  
  esc1.writeMicroseconds(VM.PWM1);
  esc2.writeMicroseconds(VM.PWM2);
  esc3.writeMicroseconds(VM.PWM3);
  esc4.writeMicroseconds(VM.PWM4);
}

static uint16_t CH1_in;
static uint16_t CH2_in;
static uint16_t CH3_in;
static uint16_t CH4_in;
static uint16_t CH5_in;
static uint16_t CH6_in;
float Volt;
void q_control(void){
  qu_m=q_conjug(qur);
  n=q_mult(qu_m,qu_des);

  if (n.qt0<0){
    n.qt1=-1*n.qt1;
    n.qt2=-1*n.qt2;
    n.qt3=-1*n.qt3;
    n.qt0=-1*n.qt0;
  }
  if (n.qt0>1){
    n.qt0=1;
  }
  a_error=2*acos(n.qt0);
  a_sin=sin(a_error*0.5);
  if (a_sin==0){
    a_sin=1;
  }
  a_sin=1/a_sin;
  E_max=abs(RAD_DEG*a_error);
  E_x=(n.qt1*(a_error*a_sin));
  E_y=(n.qt2*(a_error*a_sin));
  E_z=(Yaw_des-yaw_cf);
  if (E_max>50){
  E_z=0;
  }
  if (E_z>PI){
    E_z-=2*PI;
  }
  else if (E_z<-PI){
    E_z+=2*PI;
  }                

  if (ALT_HOLD==1){
    E_alt=Z_des-alt_estimada;
      V_des=min(max_vspeed,max(-max_vspeed,(E_alt*1)));
  }
  else {
    V_des=0;
  }

  gx_des= (KP_sx*E_x);
  gy_des= (KP_sy*E_y);
  gz_des= (KP_sz*E_z);    
  gz_des=min(PI/2,max(gz_des,-PI/2));
}

int bar_cont=0,control_cont=0;
int c_bat=0;
void RC(void)
{
  static uint8_t bUpdateFlags;

  if(bUpdateFlagsShared)
  {
    noInterrupts();

    bUpdateFlags = bUpdateFlagsShared;

    if(bUpdateFlags & CH1_FLAG)
    {
      CH1_mf.in(CH1_FSHARED);
      CH1_in=CH1_mf.out();
    }
    if(bUpdateFlags & CH2_FLAG)
    {
      CH2_mf.in(CH2_FSHARED);
      CH2_in =CH2_mf.out();

    }
    if(bUpdateFlags & CH3_FLAG)
    {
      CH3_in = CH3_mf.in(CH3_FSHARED);
      CH3_in=CH3_mf.out();
    }
    if(bUpdateFlags & CH4_FLAG)
    {
      CH4_mf.in(CH4_FSHARED);
      CH4_in = CH4_mf.out();
    }
    if(bUpdateFlags & CH5_FLAG)
    {
      CH5_mf.in(CH5_FSHARED);
      CH5_in = CH5_mf.out();
    }
    if(bUpdateFlags & CH6_FLAG)
    {
      CH6_mf.in(CH6_FSHARED);
      CH6_in = CH6_mf.out();
    }


    bUpdateFlagsShared = 0;    
    interrupts();
  }

  if ((CH1_in>1430 && CH1_in<1630) || CH1_in==0){
    CH1_in=1530;
  }
  if ((CH2_in>1430 && CH2_in<1630) || CH2_in==0 ){
    CH2_in=1530;
  }
  if ((CH3_in<1050)|| CH3_in==0){
    CH3_in=1050;
  }
  if (CH3_in>1850){
    CH3_in=1850;
  }  
  if (CH1_in<1050){
    CH1_in=1050;
  }
  if (CH1_in>1990){
    CH1_in=1990;
  }  
  if (CH2_in<1050){
    CH2_in=1050;
  }
  if (CH2_in>1990){
    CH2_in=1990;
  }
  if (CH4_in<1050){
    CH4_in=1050;
  }
  if (CH4_in>1990){
    CH4_in=1990;
  }  

  if ((CH4_in>1350 && CH4_in<1650)|| CH4_in==0){
    CH4_in=1518;
  }
  if ((CH5_in>1700)&& ini==120){
    ALT_HOLD=1;
  }
  if ( CH5_in>0 && CH5_in<1700 ){
    ALT_HOLD=0;
  }
  if ((CH6_in>1700)&& ini==120){
    POS_HOLD=1;
  }
  if ( CH6_in>0 && CH6_in<1700 ){
    POS_HOLD=0;
  }
  if (CH3_in==1050 && CH1_in==1990 && CH2_in==1990 && U1_n>0){
    ALT_HOLD=1;
    POS_HOLD=1;
    lat_des=lat_ref;
    lon_des=lon_ref;
    if ((alt_estimada-Z_des)<15 && Z_des==15){
      U1_n=0;
      U1_z=0;
    }
    else{
      U1_n=mass*9.807;
    }
    if (distance_gps<1 && SAFE_RETURN==1){
      Z_des=(-15+alt_estimada); 
    }
    SAFE_RETURN=1;
  }
  else{
    SAFE_RETURN=0;
  }
  if (CH3_in==1050 && CH4_in==1990){
    ST=1;
    digitalWrite(13,LOW);
  }
  if (CH3_in==1050 && CH4_in==1050){
    ST=0;
    digitalWrite(13,HIGH);
  }
  if (ST==0){    
    U1_n=0;
  }
  if (ST==1){

    if (CH3_in>1050 && ALT_HOLD==0 && SAFE_RETURN==0){
      int diff=CH3_in-903;
      U1_n=((float)(diff)*(float)(diff))*2.5243e-5;
    } 
    else if (ALT_HOLD==1 && SAFE_RETURN==0){
      U1_n=mass*9.807;
    } 
    if (ALT_HOLD==0){
      Z_des=alt_estimada;
    } 
    if (POS_HOLD==0){
      lat_des=lat_gps;
      lon_des=lon_gps;
    }
    if (CH3_in==1050){
      U1_n=0;
    }
  }
  else {
    Yaw_des=yaw_cf;
  }
  bUpdateFlags = 0;
}
float C_angle=0.08;
void Position_Control(void){
  if (((POS_HOLD==1 && CH1_in==1530) && (CH2_in==1530 && CH4_in==1518)) || SAFE_RETURN==1){
     if  (Check_GPS==1){
    x_gps=cos(course_gps)*distance_gps;
    y_gps=sin(course_gps)*distance_gps;
    Vx_gps=(-1*cos(course_gps)*V_gps);
    Vy_gps=(-1*sin(course_gps)*V_gps);
    float c_yaw_cf=cos(yaw_cf);
    float s_yaw_cf=sin(yaw_cf);
    float Vx_BF=Vx_gps*c_yaw_cf+Vy_gps*s_yaw_cf;
    float X_BF=x_gps*c_yaw_cf+y_gps*s_yaw_cf;
    float Y_BF=y_gps*c_yaw_cf-x_gps*s_yaw_cf;
    float Vy_BF=Vy_gps*c_yaw_cf-Vx_gps*s_yaw_cf;
    x_int=x_int+0.0012*X_BF*0.02;
    y_int=x_int+0.0012*Y_BF*0.02;
    pitch_des=max(-0.1,min(0.1,0.018*(X_BF)+x_int-0.000135*V_x+Vx_BF*0.0135));
    roll_des=max(-0.1,min(0.1,0.018*(Y_BF)+y_int+0.000135*V_y+Vy_BF*0.0135));
     }
    else{
     pitch_des=0;
     roll_des=0;
     x_int=0;
     y_int=0;
    }
  }
  else{   
    roll_des=(((float)CH1_in-1050)*(0.03125)-15)*DEG_RAD;
    pitch_des=(((float)CH2_in-1050)*(0.03125)-15)*DEG_RAD;
    lat_des=lat_gps;
    lon_des=lon_gps;


  }
  if (CH4_in!=1518){
    Yaw_des=yaw_cf;
    gz_des=(((float)(CH4_in-1050)*(0.1388888889))-65);
    gz_des=(float)(round((int)-1*gz_des))*DEG_RAD;      
  }
} 
void V_dinamica(void){
  if (dt==0){
    dt=0.01;
  } 
  else{
    dt=(micros()-time1)*1e-6;
  }
  time1=micros();

  na_tierra.Zaxis=(aaWorld.z*1.2207e-4);
  na_tierra.Xaxis=1.2207e-4*(aaWorld.x*cos(-Yaw)-aaWorld.y*sin(-Yaw));
  na_tierra.Yaxis=1.2207e-4*(aaWorld.y*cos(-Yaw)+aaWorld.x*sin(-Yaw));
  LowpassFilter(na_tierra.Zaxis,na_tierra_ant.Zaxis,dt,0.00318);
  LowpassFilter(na_tierra.Yaxis,na_tierra_ant.Yaxis,dt,0.00318);
  LowpassFilter(na_tierra.Xaxis,na_tierra_ant.Xaxis,dt,0.00318);
  na_tierra_ant.Zaxis=na_tierra.Zaxis;
  na_tierra_ant.Xaxis=na_tierra.Xaxis;
  na_tierra_ant.Yaxis=na_tierra.Yaxis;
  if (abs(na_tierra.Xaxis*1000)<30){
    na_tierra.Xaxis=0;
  }
  if (abs(na_tierra.Yaxis*1000)<30){
    na_tierra.Yaxis=0;
  }
  if (abs(na_tierra.Zaxis*1000)<10){
    na_tierra.Zaxis=0;
  }  


  accX_smooth.update((na_tierra.Xaxis)*980.7,dt);
  accY_smooth.update((na_tierra.Yaxis)*980.7,dt);
  accZ_smooth.update((na_tierra.Zaxis)*980.7,dt);
  V_x=accX_smooth.getOutput();
  V_y=accY_smooth.getOutput(); 
  vel_estimada=(accZ_smooth.getOutput());
}
void Baro(void){
  uint32_t rawPress=baro.rawPressure(MS561101BA_OSR_4096);
  if (rawPress){
    rawPress_t=rawPress;
  }
  press = baro.getPressure(MS561101BA_OSR_4096,rawPress_t);
     altura_b= baro.getAltitude(press,refPress,temp);
  if (!(isinf(altura_b)) && !(isnan(altura_b)) && (press>10)){
    altura=altura_b;
    if (ST==0){
      ref_alt=altura;
      alt_estimada=0;     
    }
  }

}
void Mg_read(void){


  compass.getHeading(&_mx, &_my, &_mz);
  mx=0.0039*_mx-2.9821e-4*_my+1.2695e-4*_mz+0.3530;
  my=-2.9821e-4*_mx+0.0036*_my+1.0029e-4*_mz+0.4711;
  mz=1.2695e-4*_mx+1.0029e-4*_my+0.0043*_mz-0.1625;          
  float  mag_norm=1/sqrt((mx*mx)+(my*my)+(mz*mz));
  if (mag_norm!=0){
    mx=mx*mag_norm;
    my=my*mag_norm;
    mz=mz*mag_norm;
  }              

}
void Reset(void){
  errork2=0;
  errork3=0;
  ui2=0;
  ui3=0;
  ui_p=0;
  ui_q=0;
  ui_r=0;
  e_p=0;
  e_q=0;
  e_r=0;
  uiZ=0;
  U1=0;
  U2=0;
  U3=0;
  U4=0;
  dt_r=0;
  dt_v=0;
  E_x=0;
  E_y=0;
  E_max=0;
  x_int=0;
  y_int=0;
  distance_gps=0;
  course_gps=0;
  control_cont=0;
  if (VM.PWM1>1050){
    VM.PWM1=VM.PWM1-15;
  } 
  else{
    VM.PWM1=1050;
  }
  if (VM.PWM2>1050){
    VM.PWM2=VM.PWM2-15;
  } 
  else{
    VM.PWM2=1050;
  }
  if (VM.PWM3>1050){
    VM.PWM3=VM.PWM3-15;
  } 
  else{
    VM.PWM3=1050;
  }
  if (VM.PWM4>1050){
    VM.PWM4=VM.PWM4-15;
  } 
  else{
    VM.PWM4=1050;
  }


  esc1.writeMicroseconds(VM.PWM1);
  esc2.writeMicroseconds(VM.PWM2);
  esc3.writeMicroseconds(VM.PWM3);
  esc4.writeMicroseconds(VM.PWM4);
}
void Telemetria(void){
  Serial3.print((int)(ypr[1]*100));
  Serial3.print(",");
  Serial3.print((int)(ypr[2]*100));
  Serial3.print(",");
  Serial3.print((int)(yaw_cf*100));
  Serial3.print(",");
  Serial3.print(Check_GPS);
  Serial3.print(",");
  Serial3.println((int)alt_estimada);
}
void _50hz_second(void){
  RC();
  if (ST==1 && CH3_in>1050){
    if (E_max<(75)){
      qcalc_des(Yaw,pitch_des,roll_des,qu_des);
    } 
    else{
      qcalc_des(2*atan2(qur.qt3,qur.qt0),0,0,qu_des);
    } 
    q_control();
  }
  s_cont++;
  if (s_cont>=8){
    Telemetria();
    s_cont=0;
  }

}
void _50hz_first(void){
  Mg_read();
  if (fifoin==0 || ALT_HOLD==1 || POS_HOLD==1){
    gps_cont++;    
    Baro();
    if (gps_cont>=6){
      GPS();

      if (ST==1){
        distance_gps=gps.distance_between(lat_gps,lon_gps,lat_des,lon_des);
        course_gps=gps.course_to(lat_gps,lon_gps,lat_des,lon_des);
      }
      gps_cont=0;
    }
  }
  V_dinamica();
  Position_Control();   
}

void loop()
{

  if (ini<120){
    if (ini==0){
      delay(2000);
      setup(); 
      calibrateGyro(120);
    }
    ini=ini+1;    
  }
  else if (c==0) {
    c=1;
  }
  c_dinamica();
  if (p_cont==0){
    _50hz_first();
    p_cont++;
  }
  else{
    _50hz_second();
    p_cont=0;
  }
  if (ST==1 && CH3_in>1050){
    q_rate();
  }
  else{
    Reset();
  }

}

void calc_CH1()
{
  if(PINK & B00000010)
  { 
    CH1_start = micros();
  }
  else
  {
    CH1_FSHARED = (uint16_t)(micros() - CH1_start);
    bUpdateFlagsShared |= CH1_FLAG;
  }
};
void calc_CH2()
{
  if(PINK & B00000100)
  { 
    CH2_start = micros();
  }
  else
  {
    CH2_FSHARED = (uint16_t)(micros() - CH2_start);
    bUpdateFlagsShared |= CH2_FLAG;
  }
};
void calc_CH3()
{
  if(PINK & B00000001)
  { 
    CH3_start = micros();
  }
  else
  {
    CH3_FSHARED = (uint16_t)(micros() - CH3_start);
    bUpdateFlagsShared |= CH3_FLAG;
  }
};
void calc_CH4()
{
  if(PINK & B00001000)
  { 
    CH4_start = micros();
  }
  else
  {
    CH4_FSHARED = (uint16_t)(micros() - CH4_start);
    bUpdateFlagsShared |= CH4_FLAG;
  }
};
void calc_CH5()
{
  if(PINK & B00010000)
  { 
    CH5_start = micros();

  }
  else
  {
    CH5_FSHARED = (uint16_t)(micros() - CH5_start);
    bUpdateFlagsShared |= CH5_FLAG;
  }
};
void calc_CH6()
{
  if(PINK & B00100000)
  { 
    CH6_start = micros();

  }
  else
  {
    CH6_FSHARED = (uint16_t)(micros() - CH6_start);
    bUpdateFlagsShared |= CH6_FLAG;
  }
};

int sign(float x){
  if (x>=0){
    return 1;
  }
  else {
    return -1;
  }
}




