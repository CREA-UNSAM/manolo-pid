// ------------------------- CONFIGURACIÓN ---------------------------
// Lógica del sensor (1 = línea negra, 0 = línea blanca)
#define LINE_LOGIC 0

// Debug para testear los sensores y valores de los motores
#define DEBUG 0

// Límites de PWM para los motores
#define MOTOR_MAX_PWM 255
#define MOTOR_MIN_PWM -255
#define MOTOR_RIGHT_OFFSET 0  // Compensación para motor derecho

// Velocidad base y ajustes
#define BASE_SPEED 255
#define RUN_INTERVAL 20     // ms
#define BLINK_INTERVAL 490  // ms

// Pines de hardware
enum Pins {
  // Motores
  MOTOR_L_PWM = 3,  // D3 | Digital 3 | GPIO 5
  MOTOR_L_IN1 = 4,  // D4 | Digital 4 | GPIO 6
  MOTOR_L_IN2 = 2,  // D2 | Digital 2 | GPIO 4
  MOTOR_R_PWM = 5,  // D5 | Digital 5 | GPIO 11
  MOTOR_R_IN1 = 6,  // D7 | Digital 7 | GPIO 13
  MOTOR_R_IN2 = 7,  // D6 | Digital 6 | GPIO 12

  // Sensores
  SENSOR_D0 = 11,
  SENSOR_A1 = A5,
  SENSOR_A2 = A4,
  SENSOR_A3 = A3,
  SENSOR_A4 = A2,
  SENSOR_A5 = A1,
  SENSOR_A6 = A0,
  SENSOR_D7 = 12,

  LED_PIN = 8,    // D8 | Digital 8 | GPIO 14
  BUTTON_PIN = 9  // D9 | Digital 9 | GPIO 15
};

// ----------------------- ESTRUCTURAS DE DATOS ------------------------
struct Motor {
  uint8_t pwm_pin;
  uint8_t in1_pin;
  uint8_t in2_pin;
  int16_t speed;
};

struct PIDController {
  double Kp = 90;
  double Ki = 0.0001;
  double Kd = 80;
  double setpoint = 0.0;
  double integral = 0.0;
  double prev_error = 0.0;
};

// ------------------------ VARIABLES GLOBALES -------------------------
Motor left_motor = { MOTOR_L_PWM, MOTOR_L_IN1, MOTOR_L_IN2, 0 };
Motor right_motor = { MOTOR_R_PWM, MOTOR_R_IN1, MOTOR_R_IN2, 0 };
PIDController pid;

const int sensor_weights[8] = { 0, 100, 60, 25, 25, 60, 100, 0 };
uint16_t sensor_values[8] = { 0 };

bool system_active = false;
bool led_state = false;
uint32_t last_run_time = 0;
uint32_t last_blink_time = 0;

// ------------------------- PROTOTIPOS --------------------------------
void readSensors();
double calculateLinePosition();
double updatePID(double position);
void setMotorSpeed(Motor &motor, int16_t speed);
void handleSerialInput();
void toggleSystemState();

// -------------------------- CONFIGURACIÓN ----------------------------
void setup() {

  Serial.begin(9600);

  // Inicializar pines
  pinMode(LED_PIN, OUTPUT);
  pinMode(BUTTON_PIN, INPUT_PULLUP);

  // Configurar pines de motores
  const uint8_t motor_pins[] = {
    MOTOR_L_PWM, MOTOR_L_IN1, MOTOR_L_IN2,
    MOTOR_R_PWM, MOTOR_R_IN1, MOTOR_R_IN2
  };

  for (uint8_t pin : motor_pins) pinMode(pin, OUTPUT);
}

// --------------------------- BUCLE PRINCIPAL --------------------------
void loop() {
  uint32_t current_time = millis();
  if (DEBUG) {
    readSensors();
    double position = calculateLinePosition();
    double Output = updatePID(position);
    Serial.print("PID Actualizado: ");
    Serial.print("Kp=");
    Serial.print(pid.Kp);
    Serial.print(" Ki=");
    Serial.print(pid.Ki);
    Serial.print(" Kd=");
    Serial.print(pid.Kd);
    Serial.print(" ||");
    Serial.print(" Speed_L=");
    Serial.print(constrain(BASE_SPEED + Output, MOTOR_MIN_PWM, MOTOR_MAX_PWM));
    Serial.print(" Speed_R=");
    Serial.print(constrain(BASE_SPEED - Output + MOTOR_RIGHT_OFFSET, MOTOR_MIN_PWM, MOTOR_MAX_PWM));
    Serial.print(" ||");
    Serial.print(" position=");
    Serial.println(position);
    delay(500);

  } else {
    /*
    // Si hay datos desde la PC (por Bluetooth)
      if (Serial.available()) {
    String comando = Serial.readStringUntil('\n');
    comando.trim();

    Serial.print("Recibido: ");
    Serial.println(comando);

    // Comandos para controlar LED y variable
    if (comando.startsWith("kp=")) {
      pid.Kp = comando.substring(3).toDouble();
      Serial.print("KP cambiado a ");
      Serial.println(pid.Kp);
    }
    else if (comando.startsWith("kd=")) {
      pid.Kd = comando.substring(3).toDouble();
      Serial.print("KD cambiada a ");
      Serial.println(pid.Kd);
    }
    else {
      Serial.println("Comando no reconocido");
    }
  }
  */
    // Control del sistema
    if (system_active) {
      if (current_time - last_run_time >= RUN_INTERVAL) {
        readSensors();
        double position = calculateLinePosition();
        double Output = updatePID(position);
        // Aplicar velocidades
        setMotorSpeed(left_motor, BASE_SPEED + Output);
        setMotorSpeed(right_motor, BASE_SPEED - Output + MOTOR_RIGHT_OFFSET);

        last_run_time = current_time;
        digitalWrite(LED_PIN, HIGH);
      }
    } else {
      if (current_time - last_blink_time >= BLINK_INTERVAL) {
        led_state = !led_state;
        digitalWrite(LED_PIN, led_state);
        last_blink_time = current_time;
      }
      setMotorSpeed(left_motor, 0);
      setMotorSpeed(right_motor, 0);
    }
  }

  // Entrada serie y botón
  //handleSerialInput();
  if (!digitalRead(BUTTON_PIN)) toggleSystemState();
}

// ------------------------- FUNCIONES PRINCIPALES ----------------------
void readSensors() {
  // Leer sensores digitales
  sensor_values[0] = (LINE_LOGIC != digitalRead(SENSOR_D0));
  sensor_values[7] = (LINE_LOGIC != digitalRead(SENSOR_D7));

  // Leer sensores analógicos
  const uint8_t analog_pins[] = { SENSOR_A1, SENSOR_A2, SENSOR_A3,
                                  SENSOR_A4, SENSOR_A5, SENSOR_A6 };

  for (uint8_t i = 0; i < 6; i++) {
    uint16_t value = analogRead(analog_pins[i]);
    if (LINE_LOGIC) value = 1023 - value;
    sensor_values[i + 1] = (value > 512) ? 1 : 0;
  }
}

  double calculateLinePosition() {
    int left_max = 0, right_max = 0;

    // Mitad izquierda (sensores 0-3)
    for (uint8_t i = 0; i < 4; i++) {
      if (sensor_values[i] && sensor_weights[i] > left_max)
        left_max = sensor_weights[i];
    }

    // Mitad derecha (sensores 4-7)
    for (uint8_t i = 4; i < 8; i++) {
      if (sensor_values[i] && sensor_weights[i] > right_max)
        right_max = sensor_weights[i];
    }

    return left_max - right_max;
  }
  
  /*
double calculateLinePosition() {
  double numerator = 0;
  double denominator = 0;
  for (int i = 0; i < 8; i++) {
    numerator += sensor_values[i] * sensor_weights[i];
    denominator += sensor_values[i];
  }
  if (denominator == 0) return 0;
  return numerator / denominator;
}
*/
double updatePID(double position) {
  double error = pid.setpoint - position;
  pid.integral += error;
  //pid.integral = constrain(pid.integral, -1000, 1000);  // anti wind-up

  double derivative = error - pid.prev_error;
  pid.prev_error = error;
  // Cálculo de la corrección PID
  double correction = (pid.Kp * error) + (pid.Ki * pid.integral) + (pid.Kd * derivative);
  // Deadband para evitar correcciones muy pequeñas
  //if (abs(correction) < 500) correction = 0;
  return correction;
}

void setMotorSpeed(Motor &motor, int16_t speed) {
  speed = constrain(speed, MOTOR_MIN_PWM, MOTOR_MAX_PWM);

  if (speed > 0) {
    digitalWrite(motor.in1_pin, HIGH);
    digitalWrite(motor.in2_pin, LOW);
  } else if (speed < 0) {
    digitalWrite(motor.in1_pin, LOW);
    digitalWrite(motor.in2_pin, HIGH);
    speed = -speed;
  } else {
    digitalWrite(motor.in1_pin, LOW);
    digitalWrite(motor.in2_pin, LOW);
  }

  analogWrite(motor.pwm_pin, speed);
}

// ------------------------- FUNCIONES AUXILIARES ----------------------
void toggleSystemState() {
  static uint32_t last_press = 0;
  if (millis() - last_press > 250) {  // Debounce de 250ms
    system_active = !system_active;
    Serial.print("Estado: ");
    Serial.println(system_active ? "ACTIVO" : "INACTIVO");
    last_press = millis();
  }
}

void handleSerialInput() {
  if (Serial.available()) {
    String input = Serial.readStringUntil('\n');
    input.trim();

    if (input.startsWith("Kp")) pid.Kp = input.substring(2).toFloat();
    else if (input.startsWith("Ki")) pid.Ki = input.substring(2).toFloat();
    else if (input.startsWith("Kd")) pid.Kd = input.substring(2).toFloat();

    Serial.print("PID Actualizado: ");
    Serial.print("Kp=");
    Serial.print(pid.Kp);
    Serial.print(" Ki=");
    Serial.print(pid.Ki);
    Serial.print(" Kd=");
    Serial.println(pid.Kd);
  }
}
