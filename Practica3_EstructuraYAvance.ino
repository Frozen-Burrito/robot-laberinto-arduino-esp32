#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>

#define PWM1_CH1  0
#define PWM1_CH2  0
#define PWM1_RES  8
#define PWM1_FREQ 1000

#define VALOR_MAX_COLOR 5000

#define DISTANCIA_MAX_CM 500

// Declaracion del sensor MPU6050 con libreria de Adafruit.
Adafruit_MPU6050 mpu6050;
const bool usarMPU6050 = false;

// Pin de activacion / desactivacion del movimiento.
const int PIN_EN_MOV = 24;

// Es verdadero si el Arduino tendra conexion USB a una computadora.
const bool usarSerial = true;

const int TOLERANCIA_ROT_Z = 1;

///////////////////////////////////////////////////////////////////////
//                        Estructuras                                //
///////////////////////////////////////////////////////////////////////

// Posibles estados de movimiento del robot.
typedef enum {
  ROBOT_DETENIDO,
  ROBOT_AVANZANDO,
  ROBOT_GIRANDO_90_DER,
  ROBOT_GIRANDO_90_IZQ,
} estado_robot_t;

// Posibles estados de un motor.
typedef enum {
  MOTOR_DETENIDO,
  MOTOR_AVANZANDO,
  MOTOR_RETROCEDIENDO,
} estado_motor_t;

// Estructura de conveniencia para almacenar los numeros de los 3 pines de un motor.
struct pines_motor_t 
{
  const int pin1;
  const int pin2;
  const int pinEn;
  const int canalPWM;
};

// Almacena los pines asignados a un sensor TCS3200.
typedef struct {
  const int s0;
  const int s1;
  const int s2;
  const int s3;
  const int out;
} tcs3200_t;

// Representa un color RGB, usualmente con valores entre 0 y 255.
typedef struct {
  const int r;
  const int g;
  const int b;
} rgb_t;

// Colores discretos para la ejecución de acciones según percepción.
typedef enum {
  BLANCO,
  ROJO,
  AMARILLO,
  AZUL,
  VERDE,
  NEGRO
} color_t;

///////////////////////////////////////////////////////////////////////
//                        Componentes                                //
///////////////////////////////////////////////////////////////////////
pines_motor_t motor1 = { 2, 3, 9, PWM1_CH1 }; // Numeros de pines para el motor izquierdo.
pines_motor_t motor2 = { 4, 5, 10, PWM1_CH2 }; // Numeros de pines para el motor derecho.

tcs3200_t sensor_color = { 27, 26, 32, 33, 23 }; // Numeros de pines del sensor TCS 3200.

estado_robot_t estado_actual = ROBOT_DETENIDO;

///////////////////////////////////////////////////////////////////////
//               Declaración de Funciones                            //
///////////////////////////////////////////////////////////////////////
void setupMotor(const pines_motor_t&);
// Declaracion de la funcion cambiarMovimientoMotor.
void cambiarMovimientoMotor(const pines_motor_t&, const estado_motor_t&, const int velocidad = 0);

void setup_tcs3200(const tcs3200_t&);
rgb_t medir_color(const tcs3200_t&);
color_t colorDiscretoDesdeRGB(const rgb_t&);

// Determinar un nuevo movimiento, segun el estado actual de robot y su percepcion.
// Retorna el nuevo estado del robot.
estado_robot_t nuevoMovimiento(const color_t&, float, const estado_robot_t&);

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

  // Percepcion de color.
  rgb_t medidaColorRGB = medir_color(sensor_color);

  Serial.println("Color: (R = " + String(medidaColorRGB.r) + ", G = " + String(medidaColorRGB.g) + ", B = " + String(medidaColorRGB.b) + ")");

  color_t colorPercibido = colorDiscretoDesdeRGB(medidaColorRGB);

  Serial.println("Color percibido: " + String(colorPercibido));

  // Sensor ultrasonico.
  float distancia = DISTANCIA_MAX_CM;

  // Actualizar estado actual con la percepcion.
  estado_actual = nuevoMovimiento(colorPercibido, distancia, estado_actual);

  delay(1000);

  // Revisar si se desean activar los motores
  int movimientoActivado = LOW;
  if (movimientoActivado == HIGH)
  {
    // Avanzar por tres segundos en linea recta.
    cambiarMovimientoMotor(motor1, MOTOR_AVANZANDO, 150);
    cambiarMovimientoMotor(motor2, MOTOR_AVANZANDO, 150);
    delay(3000);

    // Detener motores por un segundo.
    cambiarMovimientoMotor(motor1, MOTOR_DETENIDO);
    cambiarMovimientoMotor(motor2, MOTOR_DETENIDO);
    delay(1000);

    // Retroceder por tres segundos en linea recta.
    cambiarMovimientoMotor(motor1, MOTOR_RETROCEDIENDO, 150);
    cambiarMovimientoMotor(motor2, MOTOR_RETROCEDIENDO, 150);
    delay(3000);

    // Detener motores por un segundo.
    cambiarMovimientoMotor(motor1, MOTOR_DETENIDO);
    cambiarMovimientoMotor(motor2, MOTOR_DETENIDO);
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

color_t colorDiscretoDesdeRGB(const rgb_t& rgb)
{
  const int T = 5; // Tolerancia para percepcion de color.
  const int T_RB = 30;
  const int NIV_MEDIO = 220;
  const int NIV_COLOR_BRILLANTE = 240;
  const int NIV_MAX_COLOR_OSCURO = 160;
  
  const int diffGR = abs(rgb.g - rgb.r);
  const int diffGB = abs(rgb.g - rgb.b);
  const int diffRB = abs(rgb.r - rgb.b);
  
  if(rgb.g >= NIV_COLOR_BRILLANTE && diffGR < T && diffGB < T) 
  {
    return BLANCO; // 0
    
  } else if (rgb.r > NIV_MEDIO && diffRB < T_RB && rgb.g < rgb.b)
  {
    return ROJO; // 1
    
  } else if (rgb.r >= NIV_COLOR_BRILLANTE && diffGR < 20 && diffGB >= T) 
  {
    return AMARILLO; // 2
    
  } else if (rgb.b > NIV_MEDIO && rgb.g > NIV_MAX_COLOR_OSCURO && diffRB > T_RB + 5) 
  {
    return AZUL; // 3
  
  } else if (rgb.g > NIV_MEDIO - 10 && diffRB < (T * 3))
  {
    return VERDE; // 4
    
  } else {
    
    return NEGRO; // 5
    
  } 
//  else if(rojo < azul && verde > azul && rojo < 200){
//
//    lcd.print("Color Rojo");
//    
//  } else if(rojo > verde && azul > verde && verde > 50){
//
//    lcd.print("Color Verde");
//    
//  } else if(azul < rojo && azul < verde && verde < rojo){
//
//    lcd.print("Color Azul");
//
//  } else if(azul > rojo && azul > verde && verde > rojo) {
//
//    lcd.print("Color Amarillo");
//    
//  }
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

estado_robot_t nuevoMovimiento(const color_t& color, float distancia, const estado_robot_t& estadoRobot)
{
  // Segun el color:
  // BLANCO = Avanzar
  // NEGRO = Detenerse, girar para mantenerse en el espacio.
  // ROJO = 90 a la derecha
  // AMARILLO = 90 a la izquierda

  // Segun distancia
  // Si distancia < DIST_MIN_EVITAR_COLISION = evitar obstaculo

  // Si el nuevo estado es igual al estado actual y ninguna de las condiciones anteriores
  // se cumplen, seguir haciendo lo que esta haciendo.
}

void setupMotor(const pines_motor_t& pinesMotor)
{
  ledcAttachPin(pinesMotor.pinEn, pinesMotor.canalPWM);
  ledcSetup(pinesMotor.canalPWM, PWM1_FREQ, PWM1_RES);

  pinMode(pinesMotor.pin1, OUTPUT);
  pinMode(pinesMotor.pin2, OUTPUT);
}

// Definicion de la funcion cambiarMovimientoMotor.
void cambiarMovimientoMotor(const pines_motor_t& motor, const estado_motor_t& nuevoEstado, const int velocidad)
{
  // Evalua el nuevo estado al que se desea cambiar el motor.
  switch (nuevoEstado)
  {
    case MOTOR_DETENIDO:
      // Detener completamente el motor.
      digitalWrite(motor.pin1, LOW);
      digitalWrite(motor.pin2, LOW);
      ledcWrite(motor.canalPWM, 0);
      break;
    case MOTOR_AVANZANDO:
      // Hacer que el motor gire, con cierta velocidad.
      digitalWrite(motor.pin1, HIGH);
      digitalWrite(motor.pin2, LOW);
      ledcWrite(motor.canalPWM, velocidad);
      break;
    case MOTOR_RETROCEDIENDO:
      // Hacer que el motor gire en sentido inverso, con cierta velocidad.
      digitalWrite(motor.pin1, LOW);
      digitalWrite(motor.pin2, HIGH);
      ledcWrite(motor.canalPWM, velocidad);
      break;
    default: // El nuevoEstado recibido no es soportado, no hacer nada.
      Serial.println("ADVERTENCIA: Cambio de estado de motor no soportado.");
      return;
  }

  // Imprimir el cambio de estado en el monitor serial.
  Serial.println("Estado del motor (conectado al pin " + String(motor.pinEn) + ") cambiado a " + String(nuevoEstado) + ", velocidad: " + String(velocidad));
}
