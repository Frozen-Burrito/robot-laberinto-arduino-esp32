// Pin de activacion / desactivacion del movimiento.
const int PIN_EN_MOV = 24;

// Descriptores para los posibles estados de los motores.
const int MOTOR_DETENIDO = 0;
const int MOTOR_AVANZANDO = 1;
const int MOTOR_RETROCEDIENDO = 2;

// Es verdadero si el Arduino tendra conexion USB a una computadora.
const bool usarSerial = true;

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

void setup() {

  // Configurar pines de control de motores como salida.
  pinMode(motor1.pin1, OUTPUT);
  pinMode(motor1.pin2, OUTPUT);
  pinMode(motor1.pinEn, OUTPUT);

  pinMode(motor2.pin1, OUTPUT);
  pinMode(motor2.pin2, OUTPUT);
  pinMode(motor2.pinEn, OUTPUT);

  // Pin de activacion como entrada.
  pinMode(PIN_EN_MOV, INPUT);

  // Solo inicializar Serial si el Arduino va a tener conexion USB y se desea utilizar Serial.
  if (usarSerial)
  {
    Serial.begin(9600);
    while (!Serial) 
    {
    }
  
    Serial.println("Serial listo");
  }

  delay(1000);
}

void loop() 
{
  // Obtener valor 
  int movimientoActivado = digitalRead(PIN_EN_MOV);

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

  /*
  if (btnAvanzar == HIGH && estadoMotor != 1) 
  {
    estadoMotor = 1;
    
    digitalWrite(ENABLE_MOTOR1, HIGH);
    digitalWrite(PIN1_MOTOR1, HIGH);
    digitalWrite(PIN2_MOTOR1, LOW);

    lcd.clear();
    lcd.print("Avanzando");
    
    Serial.println("Avanzando");
    
  } else if (btnRetroceder == HIGH && estadoMotor != 2) 
  {
    estadoMotor = 2;
     
    digitalWrite(ENABLE_MOTOR1, HIGH);
    digitalWrite(PIN1_MOTOR1, LOW);
    digitalWrite(PIN2_MOTOR1, HIGH);

    lcd.clear();
    lcd.print("Retrocediendo");
    Serial.println("Retrocediendo");
    
  } else if (btnDetener == HIGH && estadoMotor != 0) 
  {
    estadoMotor = 0;
    
    digitalWrite(ENABLE_MOTOR1, LOW);
    digitalWrite(PIN1_MOTOR1, LOW);
    digitalWrite(PIN2_MOTOR1, LOW);

    lcd.clear();
    lcd.print("Detenido");
    Serial.println("Detenido");
  }
  */
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
