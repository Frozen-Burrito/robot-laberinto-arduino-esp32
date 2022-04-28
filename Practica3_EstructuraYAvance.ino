#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>

#define PWM1_CH1  0
#define PWM1_CH2  0
#define PWM1_RES  8
#define PWM1_FREQ 1000

#define VALOR_MAX_COLOR 5000

// Declaracion del sensor MPU6050 con libreria de Adafruit.
Adafruit_MPU6050 mpu6050;
const bool usarMPU6050 = false;

// Pin de activacion / desactivacion del movimiento.
const int PIN_EN_MOV = 24;

// Descriptores para los posibles estados de los motores.
const int MOTOR_DETENIDO = 0;
const int MOTOR_AVANZANDO = 1;
const int MOTOR_RETROCEDIENDO = 2;

// Es verdadero si el Arduino tendra conexion USB a una computadora.
const bool usarSerial = true;

const int TOLERANCIA_ROT_Z = 1;

// Estructura de conveniencia para almacenar los numeros de los 3 pines de un motor.
struct pines_motor_t 
{
  const int pin1;
  const int pin2;
  const int pinEn;
  const int canalPWM;
};

typedef struct {
  const int s0;
  const int s1;
  const int s2;
  const int s3;
  const int out;
} tcs3200_t;

typedef struct {
  const int r;
  const int g;
  const int b;
} rgb_t;

typedef enum {
  BLANCO,
  ROJO,
  AMARILLO,
  VERDE,
  AZUL,
  NEGRO
} color_t;

pines_motor_t motor1 = { 2, 3, 9, PWM1_CH1 }; // Numeros de pines para el motor izquierdo.
pines_motor_t motor2 = { 4, 5, 10, PWM1_CH2 }; // Numeros de pines para el motor derecho.

tcs3200_t sensor_color = { 27, 26, 32, 33, 23 }; // Numeros de pines del sensor TCS 3200.

void setupMotor(const pines_motor_t&);
// Declaracion de la funcion cambiarMovimientoMotor.
void cambiarMovimientoMotor(const int, const pines_motor_t&, const int velocidad = 0);

void setup_tcs3200(const tcs3200_t&);
rgb_t medir_color(const tcs3200_t&);

void configurarMPU6050();

void setup() 
{
  // Solo inicializar Serial si el Arduino va a tener conexion USB y se desea utilizar Serial.
  if (usarSerial)
  {
    Serial.begin(9600);
    while (!Serial) 
    {
    }
  
    Serial.println("Serial listo");
  }
  
  // Configurar pines de control de motores.
  setupMotor(motor1);
  setupMotor(motor2);

  setup_tcs3200(sensor_color);

  // Pin de activacion como entrada.
  pinMode(PIN_EN_MOV, INPUT);

  Serial.println("Motores inicializados");

  if (usarMPU6050 && !mpu6050.begin())
  {
    Serial.println("Error de inicializacion del MPU6050: verifica que este conectado");

    // Configurar el MPU6050.
    configurarMPU6050();
  }

  Serial.println("MPU6050 inicializado");

  delay(1000);
}

void loop() 
{
  // Revisar si se desean activar los motores
  int movimientoActivado = LOW;
  if (usarMPU6050)
  {
    // Obtener valores del MPU6050.
    sensors_event_t a, g, t;
    mpu6050.getEvent(&a, &g, &t);

    if (usarSerial)
    {
      // Imprimir valores de acelerometro, giroscopio y temperatura.
      String valorAcc = "(" + String(a.acceleration.x) + "," + String(a.acceleration.y) + ", " + String(a.acceleration.z) + ")";
      Serial.println("Aceleración (x,y,z): " + valorAcc + " m/s^2");

      String valorGyro = "(" + String(g.gyro.x) + "," + String(g.gyro.y) + ", " + String(g.gyro.z) + ")";
      Serial.println("Rotacion (x,y,z): " + valorGyro + " rad/s");

      Serial.println("Temperatura: " + String(t.temperature) + " °C");
    }
  }

  rgb_t color_detectado = medir_color(sensor_color);

  Serial.println("Color: (R = " + String(color_detectado.r) + ", G = " + String(color_detectado.g) + ", B = " + String(color_detectado.b) + ")");

  delay(1000);

  if (movimientoActivado == HIGH)
  {
    // Avanzar por tres segundos en linea recta.
    cambiarMovimientoMotor(MOTOR_AVANZANDO, motor1, 150);
    cambiarMovimientoMotor(MOTOR_AVANZANDO, motor2, 150);
    delay(3000);

    // Detener motores por un segundo.
    cambiarMovimientoMotor(MOTOR_DETENIDO, motor1);
    cambiarMovimientoMotor(MOTOR_DETENIDO, motor2);
    delay(1000);

    // Retroceder por tres segundos en linea recta.
    cambiarMovimientoMotor(MOTOR_RETROCEDIENDO, motor1, 150);
    cambiarMovimientoMotor(MOTOR_RETROCEDIENDO, motor2, 150);
    delay(3000);

    // Detener motores por un segundo.
    cambiarMovimientoMotor(MOTOR_DETENIDO, motor1);
    cambiarMovimientoMotor(MOTOR_DETENIDO, motor2);
  }
}

void setup_tcs3200(const tcs3200_t& sensor)
{
  pinMode(sensor.s0, OUTPUT);
  pinMode(sensor.s1, OUTPUT);
  pinMode(sensor.s2, OUTPUT);
  pinMode(sensor.s3, OUTPUT);

  // Usar escalado de filtro del 100%.
  digitalWrite(sensor.s0, HIGH);
  digitalWrite(sensor.s1, LOW);

  pinMode(sensor.out, INPUT);
}

rgb_t medir_color(const tcs3200_t& sensor)
{
  // Activar el filtro rojo.
  digitalWrite(sensor.s2, LOW);
  digitalWrite(sensor.s3, LOW);

  const int freq_rojo = pulseIn(sensor.out, LOW);

  if (freq_rojo == 0) Serial.println("Timeout de medicion del canal rojo.");
  
  delay(25);
  
  // Activar el filtro verde.
  digitalWrite(sensor.s2, HIGH);
  digitalWrite(sensor.s3, HIGH);

  const int freq_verde = pulseIn(sensor.out, LOW);
  
  if (freq_verde == 0) Serial.println("Timeout de medicion del canal verde.");

  delay(25);

  // Activar el filtro azul.
  digitalWrite(sensor.s2, LOW);
  digitalWrite(sensor.s3, HIGH);

  const int freq_azul = pulseIn(sensor.out, LOW);

  if (freq_azul == 0) Serial.println("Timeout de medicion del canal azul.");

  // Limitar el valor de color a un rango entre 0 - VALOR_MAX_COLOR, con 0 como 
  // intensidad minima y VALOR_MAX_COLOR como intensidad maxima del color.
  const int rojo_corregido = VALOR_MAX_COLOR - constrain(freq_rojo, 0, VALOR_MAX_COLOR);
  const int verde_corregido = VALOR_MAX_COLOR - constrain(freq_verde, 0, VALOR_MAX_COLOR);
  const int azul_corregido = VALOR_MAX_COLOR - constrain(freq_azul, 0, VALOR_MAX_COLOR);

  // Transformar colores a valores entre 0 y 255.
  rgb_t colorDetectado = { 
    map(rojo_corregido, 0, VALOR_MAX_COLOR, 0, 255), 
    map(verde_corregido, 0, VALOR_MAX_COLOR, 0, 255),
    map(azul_corregido, 0, VALOR_MAX_COLOR, 0, 255) 
  };
  
  return colorDetectado;
}

// Esta funcion configura el rango del acelerometro, el rango del giroscopio y el
// ancho de banda del filtro del sensor MPU6050. Muestra mensajes de confirmacion en
// el monitor serial, si esta activado.
void configurarMPU6050()
{
  mpu6050.setAccelerometerRange(MPU6050_RANGE_8_G);
  mpu6050.setGyroRange(MPU6050_RANGE_500_DEG);
  mpu6050.setFilterBandwidth(MPU6050_BAND_5_HZ);

  if (usarSerial)
  {
    Serial.print("Rango del acelerometro: ");
    switch (mpu6050.getAccelerometerRange()) 
    {
    case MPU6050_RANGE_2_G:
      Serial.println("+-2G");
      break;
    case MPU6050_RANGE_4_G:
      Serial.println("+-4G");
      break;
    case MPU6050_RANGE_8_G:
      Serial.println("+-8G");
      break;
    case MPU6050_RANGE_16_G:
      Serial.println("+-16G");
      break;
    }
    
    Serial.print("Rango del giroscopio: ");
    switch (mpu6050.getGyroRange()) 
    {
    case MPU6050_RANGE_250_DEG:
      Serial.println("+- 250 deg/s");
      break;
    case MPU6050_RANGE_500_DEG:
      Serial.println("+- 500 deg/s");
      break;
    case MPU6050_RANGE_1000_DEG:
      Serial.println("+- 1000 deg/s");
      break;
    case MPU6050_RANGE_2000_DEG:
      Serial.println("+- 2000 deg/s");
      break;
    }
    
    Serial.print("Ancho de banda de filtro: ");
    switch (mpu6050.getFilterBandwidth()) 
    {
    case MPU6050_BAND_260_HZ:
      Serial.println("260 Hz");
      break;
    case MPU6050_BAND_184_HZ:
      Serial.println("184 Hz");
      break;
    case MPU6050_BAND_94_HZ:
      Serial.println("94 Hz");
      break;
    case MPU6050_BAND_44_HZ:
      Serial.println("44 Hz");
      break;
    case MPU6050_BAND_21_HZ:
      Serial.println("21 Hz");
      break;
    case MPU6050_BAND_10_HZ:
      Serial.println("10 Hz");
      break;
    case MPU6050_BAND_5_HZ:
      Serial.println("5 Hz");
      break;
    }
  }
}

void setupMotor(const pines_motor_t& pinesMotor)
{
  ledcAttachPin(pinesMotor.pinEn, pinesMotor.canalPWM);
  ledcSetup(pinesMotor.canalPWM, PWM1_FREQ, PWM1_RES);

  pinMode(pinesMotor.pin1, OUTPUT);
  pinMode(pinesMotor.pin2, OUTPUT);
}

// Definicion de la funcion cambiarMovimientoMotor.
void cambiarMovimientoMotor(const int nuevoEstado, const pines_motor_t& pinesMotor, const int velocidad)
{
  // Evalua el nuevo estado al que se desea cambiar el motor.
  switch (nuevoEstado)
  {
    case MOTOR_DETENIDO:
      // Detener completamente el motor.
      digitalWrite(pinesMotor.pin1, LOW);
      digitalWrite(pinesMotor.pin2, LOW);
      ledcWrite(pinesMotor.canalPWM, 0);
      break;
    case MOTOR_AVANZANDO:
      // Hacer que el motor gire, con cierta velocidad.
      digitalWrite(pinesMotor.pin1, HIGH);
      digitalWrite(pinesMotor.pin2, LOW);
      ledcWrite(pinesMotor.canalPWM, velocidad);
      break;
    case MOTOR_RETROCEDIENDO:
      // Hacer que el motor gire en sentido inverso, con cierta velocidad.
      digitalWrite(pinesMotor.pin1, LOW);
      digitalWrite(pinesMotor.pin2, HIGH);
      ledcWrite(pinesMotor.canalPWM, velocidad);
      break;
    default: // El nuevoEstado recibido no es soportado, no hacer nada.
      return;
  }

  // Imprimir el cambio de estado en el monitor serial.
  Serial.println("Estado del motor (conectado al pin " + String(pinesMotor.pinEn) + ") cambiado a " + String(nuevoEstado) + ", velocidad: " + String(velocidad));
}
