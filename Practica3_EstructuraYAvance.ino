#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>

// Declaracion del sensor MPU6050 con libreria de Adafruit.
const Adafruit_MPU6050 mpu6050;
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
};

pines_motor_t motor1 = { 2, 3, 9 }; // Numeros de pines para el motor izquierdo.
pines_motor_t motor2 = { 4, 5, 10 }; // Numeros de pines para el motor derecho.

// Declaracion de la funcion cambiarMovimientoMotor.
void cambiarMovimientoMotor(const int, const pines_motor_t&, const int velocidad = 0);

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
  
  // Configurar pines de control de motores como salida.
  pinMode(motor1.pin1, OUTPUT);
  pinMode(motor1.pin2, OUTPUT);
  pinMode(motor1.pinEn, OUTPUT);

  pinMode(motor2.pin1, OUTPUT);
  pinMode(motor2.pin2, OUTPUT);
  pinMode(motor2.pinEn, OUTPUT);

  // Pin de activacion como entrada.
  pinMode(PIN_EN_MOV, INPUT);

  Serial.println("Motores inicializados");

  if (usarMPU6050 && !mpu6050.begin())
  {
    Serial.println("Error de inicializacion del MPU6050: verifica que este conectado");
  }

  Serial.println("MPU6050 inicializado");

  delay(1000);
}

void loop() 
{
  // Revisar si se desean activar los motores
  int movimientoActivado = HIGH;
  if (usarMPU6050)
  {
    // Configurar el MPU6050.
    configurarMPU6050();

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
    delay(1000);
  }
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
      analogWrite(pinesMotor.pinEn, 0);
      break;
    case MOTOR_AVANZANDO:
      // Hacer que el motor gire, con cierta velocidad.
      digitalWrite(pinesMotor.pin1, HIGH);
      digitalWrite(pinesMotor.pin2, LOW);
      analogWrite(pinesMotor.pinEn, velocidad);
      break;
    case MOTOR_RETROCEDIENDO:
      // Hacer que el motor gire en sentido inverso, con cierta velocidad.
      digitalWrite(pinesMotor.pin1, LOW);
      digitalWrite(pinesMotor.pin2, HIGH);
      analogWrite(pinesMotor.pinEn, velocidad);
      break;
    default: // El nuevoEstado recibido no es soportado, no hacer nada.
      return;
  }

  // Imprimir el cambio de estado en el monitor serial.
  Serial.println("Estado del motor (conectado al pin " + String(pinesMotor.pinEn) + ") cambiado a " + String(nuevoEstado) + ", velocidad: " + String(velocidad));
}
