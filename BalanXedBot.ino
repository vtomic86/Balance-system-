
/* Este software se puede distribuir y modificar bajo los términos de 
 * la Licencia Pública General GNU versión 2 de la GPL (2) publicada 
 * por la Free Software Foundation y que aparecen en el archivo GPL 2.txt 
 * https://www.gnu.org/licenses/gpl-2.0.txt 
 * Tenga en cuenta que GPL2 Sección 2 [b] requiere que los trabajos basados 
 * en este software también deban estar disponibles al 
 * público en los términos de la GPL2 ("Copyleft").
 * 
 * Denle un vistazo a el trabajo de Kristian Lauszus y Larry McGovern, 
 * inspiraciones del presente codigo
 * 
 * XedBalance 1.13:
 * microprocesador atmega328p
 * motor driver l298n
 * modulo bluethooth HC-05
 * bateria 5v 700mA, bateria 12v 2500mA
 * Giro/Acel MPU6050
 * motoreductor 1:50~ motor 12v
 * encoders opticos
 * 
 * Contacto: http://elxedelectronics.blogspot.mx/   xed.cel.xed@gmail.com 
*/

#include<Wire.h>
#include <PWM.h>
#include <digitalWriteFast.h>

 #define CALCTIME_SERIALOUT  // para ver el tiempo de calculo del loop en el monitor serial

// Pins

#define e_InterEncoderDerecha 0
#define e_EncoderDerPinA 2
#define e_EncoderDerPinB 4

#define e_InterEncoderIzq 1
#define e_EncoderIzqPinA 3
#define e_EncoderIzqPinB 5


#define motor1D_B 8
#define motor1D_A 7
#define motor1D_PWM 9

#define motor2I_PWM 10
#define motor2I_A 11
#define motor2I_B 12


// Cuadratura de Encoders
// encoder izquierdo
#define encoderIzquierdoReversa
volatile bool _IzqEncoderBSet;
volatile long _ticksIzqEncoder = 0;

//  encoder derecho
volatile bool _DerEncoderBSet;
volatile long _ticksDerEncoder = 0;

#define DEG_PER_COUNT 0.50f  // 210 conteos por revolucion

#define DT 5000 // 
#define FHz 200 // 

/*
 *  Ganancias de control
 *  todas seran diferentes para cada robot 
 *  y se tienen que afinar
 
*/
#define Kp_Rotacion 0.5f
#define PWMFREQ 10000
float K_p = 6.85;
float K_i = 110.0;
float K_d = 0.17;
float delaytime = 4.25;

//// ganancias de rueda, son importantes y se deben afinar

#define gananciaPosRueda 0.020f
#define gananciaRatioRueda 0.035f

////////////////////////////////filtro de Kalman///////////////////////////
uint32_t LastTime = 0;
bool horizontalPos = true; // Usado para indicar si el robot esta en horizontal

float MotorFactorScala = 0.01f;
bool RecargarBateria = 0; // para un futuro sensor de voltaje en la bateria
 
// Gyro Address
const int MPU = 0x68; // direccion I2C de MPU-6050
uint8_t i2cBuffer[14];

float AnguloA, BiasEst;

float gyroXzero;
float PitchCalOffset;
float IntState = 0;  // estado integral

float voltageFilt = 0;

static uint16_t AceleracionA=1500, DireccionA=1500;

  //////////////leds

int ledPin1 = 6;
int ledPin2 = 13;


  /////////////////
int estado; ///para el bluetooth
  /////////////////

////////////////////////////////////////////////////////////
///////////LOOP SETUP///////////////////////////////////////
////////////////////////////////////////////////////////////  


void setup(void)
{
#ifdef CALCTIME_SERIALOUT ///calculo de tiempo del loop
  Serial.begin(115200);
  Serial.println();
#endif

  // PWM Setup
  InitTimersSafe();  // Inicializamos nueva frecuencia PWM
  SetPinFrequencySafe(motor2I_PWM, PWMFREQ);
  SetPinFrequencySafe(motor1D_PWM, PWMFREQ);
  //leds
   SetPinFrequencySafe(ledPin2, PWMFREQ);
   SetPinFrequencySafe(ledPin1, PWMFREQ);

  // Cuadratura encoder 
  // encoder izq
  pinMode(e_EncoderIzqPinA, INPUT);      
  digitalWrite(e_EncoderIzqPinA, LOW);  
  pinMode(e_EncoderIzqPinB, INPUT);      
  digitalWrite(e_EncoderIzqPinB, LOW);  
  attachInterrupt(e_InterEncoderIzq, motorIzqInterA, RISING);

  // encoder der
  pinMode(e_EncoderDerPinA, INPUT);      
  digitalWrite(e_EncoderDerPinA, LOW); 
  pinMode(e_EncoderDerPinB, INPUT);      
  digitalWrite(e_EncoderDerPinB, LOW);  
  attachInterrupt(e_InterEncoderDerecha, motorDerInterA, RISING);

  // Setup de Giroscopio http://blog.tkjelectronics.dk/2012/09/a-practical-approach-to-kalman-filter-and-how-to-implement-it/
  Wire.begin();
  Wire.setClock(400000UL); // Set I2C frequency to 400kHz

  while (i2cWrite(0x6B, 0x80, true)); // Reset device, this resets all internal registers to their default values
  do {
    while (i2cRead(0x6B, i2cBuffer, 1));
  } while (i2cBuffer[0] & 0x80); // Wait for the bit to clear
  delay(5);
  while (i2cWrite(0x6B, 0x09, true)); // PLL with X axis gyroscope reference, disable temperature sensor and disable sleep mode

  i2cBuffer[0] = 4; // Set the sample rate to 200Hz = 1kHz/(1+4)
  i2cBuffer[1] = 0x03; // Disable FSYNC and set 44 Hz Acc filtering, 42 Hz Gyro filtering, 1 KHz sampling
  i2cBuffer[2] = 0x00; // Set Gyro Full Scale Range to ±250deg/s
  i2cBuffer[3] = 0x00; // Set Accelerometer Full Scale Range to ±2g
  while (i2cWrite(0x19, i2cBuffer, 4, true)); // Write to all four registers at once
  delay(100); // Wait for the sensor to get ready
//////////////////////////////////setup de leds/////////////////////////////////////
pinMode(ledPin1, OUTPUT);
pinMode(ledPin2, OUTPUT);

}


//////////////////////////////////////////////////////////////////////////////////////
////////////////////////////LOOP PRINCIPAL////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////

void loop(void){
/////////////////variables////////////


  int i;
  static unsigned long int timer, encoderTimer, voltajeTimer, voltajeTimerOut;
  static int glitchAcelPersistente, glitchDirPersistente;
  static float Error, ultimoError, velocidadRueda;
  static double posicionRueda, lastposicionRueda, anguloRotacion;
  static unsigned long int timerVelocRueda;
  static double PosCmd, rotacionCmd;
  static float AOCmd;
  static uint16_t AceleracionAGood, DireccionAGood; 
  float AceleraF, DireccF;
  static long ticksIzqEncoder, ticksDerEncoder;
  float TorqueCMD;
  static float VoltearTorque;
  int Ciclo_I = 0, Ciclo_D = 0, brilloLed = 0;
  float Kp_fb = 0, Ki_fb = 0, Kd_fb = 0;
  float accAngle; // Resultados  raw de acelerometro
  float gyroRate;
  float SetpointA;
  static unsigned long int CalcTime, MaxCalcTime, AveCalcTime, TimeCounter;
  static int NumSamples;
  
  
  //  Gyro Datos
  while (i2cRead(0x3B, i2cBuffer, 14));
  int16_t AcY = ((i2cBuffer[2] << 8) | i2cBuffer[3]);
  int16_t AcZ = ((i2cBuffer[4] << 8) | i2cBuffer[5]);
  int16_t GyX = ((i2cBuffer[8] << 8) | i2cBuffer[9]);

  accAngle = atan2((float)AcY, (float)AcZ) * RAD_TO_DEG - PitchCalOffset;
  gyroRate = (float)GyX / 131.0f - gyroXzero; // Convertir a deg/s

  KalmanFilter(accAngle, gyroRate);

  ///////////////////// Calcular posicion y velocidad a 10 Hz
  
  if (millis() - timerVelocRueda > 100) {
    posicionRueda = 0.5f * (_ticksIzqEncoder + _ticksDerEncoder) * DEG_PER_COUNT;
    velocidadRueda = 10.0f * (posicionRueda - lastposicionRueda);
    anguloRotacion = 0.5f * (_ticksIzqEncoder - _ticksDerEncoder) * DEG_PER_COUNT * 0.486f; 
                                                                                            
                                                                                            
    lastposicionRueda = posicionRueda;
    timerVelocRueda = millis();
  }
  /////////////////////////////// lee el bluetooth y almacena en variable
  if(Serial.available()>0){        
    estado = Serial.read();
  }
     if(estado=='s'){  //adelante
       AceleracionA=1750;
      }
     if(estado=='w'){  //atras
      AceleracionA=1200;
      } 

    if(estado=='e'){ //vuelta
      AceleracionA=1200;
      DireccionA=1300;
     }
    if(estado=='q'){  //vuelta2
      AceleracionA=1200;
      DireccionA=1700;
     }
    

    if(estado=='d'){  //lado1
      DireccionA=1250;
     }
     
    if(estado=='a'){  //lado2
      DireccionA=1750;
     }
    if(estado=='z'){  //detener
    AceleracionA=1500;
    DireccionA=1500;
   }
  /* esta parte fue usada para calibrar los valores kp, ki, kd, y delaytime
   *  
   *  if(estado=='1'){
      K_p = (K_p)+0.01;
     }
 if(estado=='2'){
      K_p = (K_p)-0.01;
     }

      if(estado=='3'){
      K_i = (K_i)+0.05;
     }
 if(estado=='4'){
      K_i = (K_i)-0.05;
     }
     
      if(estado=='5'){
      delaytime = (delaytime)+0.001;
     }
  if(estado=='6'){
      delaytime = (delaytime)-0.001;
     }*/
  
//////////si el angulo en menor a -30 o mayor a 30 los motores se apagan y
//        vuelven si el angulo es mayor a -5 y menor a 5

  if ((horizontalPos && (AnguloA < -5 || AnguloA > 5)) || (~horizontalPos && (AnguloA < -30 || AnguloA > 30))) {
    horizontalPos = true;
    digitalWriteFast(motor2I_A, LOW);
    digitalWriteFast(motor2I_B, LOW);
    digitalWriteFast(motor1D_A, LOW);
    digitalWriteFast(motor1D_B, LOW);
    IntState = 0;
    _ticksIzqEncoder = 0; ticksIzqEncoder = 0;
    _ticksDerEncoder = 0; ticksDerEncoder = 0;
  }
  else {
    horizontalPos = false;
    

    ////////////el setpoint es dinamico y cambia en base a la perturbacion 
    
    if (AceleracionA > 800 && AceleracionA < 2200 && !RecargarBateria) {  // rango valido
      if ((abs(AceleracionA - AceleracionAGood) < 200) || (glitchAcelPersistente > 20)) { 
        AceleracionAGood = AceleracionA;
        glitchAcelPersistente = 0;
      }
      else {
        glitchAcelPersistente++;
      }
      AceleraF = (float)AceleracionAGood;
      AOCmd = (AceleraF - 1540.0f) / 75.0f; 
      AOCmd = constrain(AOCmd, -7.0f, 7.0f);
    }
    else {
      glitchAcelPersistente++;
      if (glitchAcelPersistente > 20) {
        glitchAcelPersistente = 0;
      }
      AOCmd = 0;
    }
    SetpointA = AOCmd;
    if (abs(AOCmd) > 2.0f) {
      PosCmd = posicionRueda + 1.0f * velocidadRueda;
      SetpointA -=  velocidadRueda * gananciaRatioRueda;
    }
    else {    /////el setpoint es dinamico y cambia en base a la perturbacion 
      SetpointA -= (posicionRueda - PosCmd) * gananciaPosRueda  + velocidadRueda * gananciaRatioRueda;
    }
    SetpointA = constrain(SetpointA, -10.0f, 10.0f);

///////////////////////////PID////////////////////////////////////////
    ultimoError = Error;
    Error = SetpointA - AnguloA;
    IntState = IntState + Error / FHz;
    IntState = constrain(IntState, -5.0f, 5.0f);

    Kp_fb = K_p * Error;
    
    Ki_fb = K_i * IntState;
    
    Kd_fb = -K_d * gyroRate;

    TorqueCMD = Kp_fb + Ki_fb + Kd_fb;
    TorqueCMD = constrain(TorqueCMD, -200, 200);

    ////// filtro de direccion y aceleracion
    
    if (DireccionA > 800 && DireccionA < 2200 && !RecargarBateria) {  // rango valido
      if ((abs(DireccionA - DireccionAGood) < 200) || (glitchDirPersistente > 20)) { 
        DireccionAGood = DireccionA;
        glitchDirPersistente = 0;
      }
      else {
        glitchDirPersistente++;
      }
      DireccF = DireccionAGood;
      VoltearTorque = (DireccF - 1500.0f) / 20.0f;  
      VoltearTorque = constrain(VoltearTorque, -25.0f, 25.0f);
      if (abs(VoltearTorque) < 5.0f && abs(AOCmd) < 2.0f) {
        VoltearTorque += Kp_Rotacion * (rotacionCmd - anguloRotacion);  // mantener angulo actual
      }
      else {
        rotacionCmd = anguloRotacion;
      }
    }
    else {
      glitchDirPersistente++;
      if (glitchDirPersistente > 20) {
        glitchDirPersistente = 0;
      }
      VoltearTorque = 0;
    }



    Ciclo_I = 255 * ((TorqueCMD - VoltearTorque) * MotorFactorScala);
    Ciclo_D = 255 * ((TorqueCMD + VoltearTorque) * MotorFactorScala);

    if (Ciclo_I < 0) {
      digitalWriteFast(motor2I_A, LOW);
      digitalWriteFast(motor2I_B, HIGH);
      Ciclo_I = -Ciclo_I;
    }
    else {
      digitalWriteFast(motor2I_A, HIGH);
      digitalWriteFast(motor2I_B, LOW);
    }

    if (Ciclo_D < 0) {
      digitalWriteFast(motor1D_A, LOW);
      digitalWriteFast(motor1D_B, HIGH);
      Ciclo_D = -Ciclo_D;
    }
    else {
      digitalWriteFast(motor1D_A, HIGH);
      digitalWriteFast(motor1D_B, LOW);
    }
    if (Ciclo_I > 255) {
      Ciclo_I = 255;
    }
    if (Ciclo_D > 255) {
      Ciclo_D = 255;
    }
  }

////////////imprimir de datos en monitor serial, descomentar para visualizar, ultimo caracter debe ser "n"

Serial.print("Angulo;"); Serial.print(AnguloA); Serial.print("\t");        ///angulo
//Serial.print("E1;"); Serial.print(_ticksIzqEncoder); Serial.print("\t");   ///encoders
//Serial.print("E2;"); Serial.print(_ticksDerEncoder); Serial.print("\t");
Serial.print("SalidaPWM; "); Serial.print(TorqueCMD); Serial.print("\t");  ///salida
//Serial.print("Ciclo_D; "); Serial.print(Ciclo_D); Serial.print("\t");  ///salida derecha pwm
//Serial.print("Ciclo_I; "); Serial.print(Ciclo_I); Serial.print("\t");  ///salida izq pwm
Serial.print("Setpoint; "); Serial.print(SetpointA); Serial.print("\n");  ///setpoint
//Serial.print("velocidadRueda; "); Serial.print(velocidadRueda); Serial.print("\n");  ///velocidad rueda


  pwmWrite(motor2I_PWM, Ciclo_I);
  pwmWrite(motor1D_PWM, Ciclo_D);
//////////////////////////////////////////tiempo de calculo en msec//////////////////////
#ifdef CALCTIME_SERIALOUT
  MaxCalcTime, AveCalcTime, TimeCounter, NumSamples;
  CalcTime = micros() - LastTime;
  if (MaxCalcTime < CalcTime) {
    MaxCalcTime = CalcTime;
  }
  AveCalcTime += CalcTime;
  NumSamples++;
  if ((millis() - TimeCounter) > 1000) {
    TimeCounter = millis();
    AveCalcTime /= NumSamples;
    NumSamples = 0;
    Serial.print("Tiempo promedio de calculo : ");
    Serial.print((float)AveCalcTime / 1000.0f);
    Serial.print(" msec,  ");
    Serial.print("Tiempo Maximo de Calculo : ");
    Serial.print((float)MaxCalcTime / 1000.0f);
    Serial.println(" msec");
    MaxCalcTime = 0;
    AveCalcTime = 0;
  }
#endif

  while (micros() - LastTime < DT);
  LastTime = micros();


  //////////////////leds////////////////////////////////
  
  
  if (AnguloA > SetpointA){
    digitalWrite(ledPin1, HIGH);
  }
      else{
           digitalWrite(ledPin1, LOW);
          }


  if (AnguloA < SetpointA){
  digitalWrite(ledPin2, HIGH);
  }
      else{
           digitalWrite(ledPin2, LOW);
          }

  
  ////////////////// delaytime tambien se debe cambiar dependiendo cada robot

delay(delaytime);

}



