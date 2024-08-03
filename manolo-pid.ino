// DEFINES
#define LOGIC 1
#define MOTORS_MAX_PWM_VALUE 255
#define MOTORS_MIN_PWM_VALUE -80
// #define MAXOUTPUT 255
#define SPEED_BASE 100

// PIN DEFINITIONS
const int PIN_LED = 8;    // D8 | Digital 8 | GPIO 14
const int PIN_BUTTON = 9; // D9 | Digital 9 | GPIO 15

// MOTOR RIGHT
const int PIN_MOTOR_R_PWM = 5; // D5 | Digital 5 | GPIO 11
const int PIN_MOTOR_R_1 = 7;   // D6 | Digital 6 | GPIO 12
const int PIN_MOTOR_R_2 = 6;   // D7 | Digital 7 | GPIO 13

// MOTOR LEFT
const int PIN_MOTOR_L_PWM = 3; // D3 | Digital 3 | GPIO 5
const int PIN_MOTOR_L_1 = 4;   // D2 | Digital 2 | GPIO 4
const int PIN_MOTOR_L_2 = 2;   // D4 | Digital 4 | GPIO 6

// SENSOR PINS
const int PIN_SENSOR_0 = 11;
const int PIN_SENSOR_1 = A5;
const int PIN_SENSOR_2 = A4;
const int PIN_SENSOR_3 = A3;
const int PIN_SENSOR_4 = A2;
const int PIN_SENSOR_5 = A1;
const int PIN_SENSOR_6 = A0;
const int PIN_SENSOR_7 = 12;

const int CANT_ANALOG_SENSORS = 6;
const int CANT_DIGITAL_SENSORS = 2;

const int CANT_ALL_SENSORS = CANT_ANALOG_SENSORS + CANT_DIGITAL_SENSORS;

const int PINS_ANALOG_SENSORS[CANT_ANALOG_SENSORS] = {PIN_SENSOR_1, PIN_SENSOR_2, PIN_SENSOR_3, PIN_SENSOR_4, PIN_SENSOR_5, PIN_SENSOR_6};
const int PINS_DIGITAL_SENSORS[CANT_DIGITAL_SENSORS] = {PIN_SENSOR_0, PIN_SENSOR_7};

int analogSensorValues[CANT_ANALOG_SENSORS];
int sensorValues[CANT_ALL_SENSORS];

int motorspeedR;
int motorspeedL;

double Setpoint, Input, Output;
double KP = 0.45, KI = 0.045, KD = 0.3;

float setpoint = 0.0; // Línea central
float error = 0.0;
float previous_error = 0.0;
float integral = 0.0;
float derivative = 0.0;
float correction = 0.0;
int left_motor_speed = 0.0;
int right_motor_speed = 0.0;

const unsigned int blinkDelay = 490;
const unsigned int runDelay = 0;
bool state = false;
bool blink = true;
bool button_state;

unsigned long blinkTimer;
unsigned long runTimer;

int memory = 0;

void setup()
{
  // Initialize serial communication
  Serial.begin(9600);
  Serial.println(" ----------------------------------------------------Start------------------------------------------------------------------ ");
  Serial.println(" ----------------------------------------------------Start------------------------------------------------------------------ ");
  Serial.println(" ----------------------------------------------------Start------------------------------------------------------------------ ");
  Serial.println(" ----------------------------------------------------Start------------------------------------------------------------------ ");
  Serial.println(" ----------------------------------------------------Start------------------------------------------------------------------ ");
  Serial.println(" ----------------------------------------------------Start------------------------------------------------------------------ ");
  Serial.println(" ----------------------------------------------------Start------------------------------------------------------------------ ");
  Serial.println(" ----------------------------------------------------Start------------------------------------------------------------------ ");
  Serial.println(" ----------------------------------------------------Start------------------------------------------------------------------ ");
  Serial.println(" ----------------------------------------------------Start------------------------------------------------------------------ ");
  Serial.println(" ----------------------------------------------------Start------------------------------------------------------------------ ");
  Serial.println(" ----------------------------------------------------Start------------------------------------------------------------------ ");
  Serial.println(" ----------------------------------------------------Start------------------------------------------------------------------ ");
  Serial.println(" ----------------------------------------------------Start------------------------------------------------------------------ ");
  Serial.println(" ----------------------------------------------------Start------------------------------------------------------------------ ");

  // Initialize the LED pin as an output
  pinMode(PIN_LED, OUTPUT);

  // Initialize the button pin as an input
  pinMode(PIN_BUTTON, INPUT_PULLUP);

  // Initialize 6 analog inputs for sensors
  for (int i = 0; i < CANT_ANALOG_SENSORS; i++)
  {
    pinMode(PINS_ANALOG_SENSORS[i], INPUT);
  }

  // Initialize 2 digital inputs for sensors
  for (int i = 0; i < CANT_DIGITAL_SENSORS; i++)
  {
    pinMode(PINS_DIGITAL_SENSORS[i], INPUT);
  }

  // Initialize the 3 outputs for each motor
  pinMode(PIN_MOTOR_L_PWM, OUTPUT);
  pinMode(PIN_MOTOR_L_1, OUTPUT);
  pinMode(PIN_MOTOR_L_2, OUTPUT);

  pinMode(PIN_MOTOR_R_PWM, OUTPUT);
  pinMode(PIN_MOTOR_R_1, OUTPUT);
  pinMode(PIN_MOTOR_R_2, OUTPUT);
}

void run(unsigned long currentTime)
{
  if ((currentTime - runTimer) >= runDelay)
  {
    for (int i = 0; i < CANT_ANALOG_SENSORS; i++)
    {
      analogSensorValues[i] = analogRead(PINS_ANALOG_SENSORS[i]);

      // Invert the reading if the line is white on black, 1 is black line
      if (LOGIC != 1)
      {
        analogSensorValues[i] = 1023 - analogSensorValues[i];
      }
    }

    sensorValues[0] = !digitalRead(PINS_DIGITAL_SENSORS[0]);

    //----analog to digital conversion
    for (int i = 0; i < CANT_ANALOG_SENSORS; i++)
    {
      sensorValues[i + 1] = analogSensorValues[i] > 512 ? 0 : 1;
    }

    sensorValues[CANT_ALL_SENSORS - 1] = !digitalRead(PINS_DIGITAL_SENSORS[1]);

    int weights[] = {24, 16, 9, 4, 4, 9, 16, 24};

    int maxLeftDetections = 0;
    for (int i = 0; i < CANT_ALL_SENSORS / 2; i++)
    {
      if (sensorValues[i] == 1 && weights[i] > maxLeftDetections)
      {
        maxLeftDetections = weights[i];
      }
    }
    Serial.print(maxLeftDetections);
    int maxRightDetections = 0;
    for (int i = CANT_ALL_SENSORS / 2; i < CANT_ALL_SENSORS; i++)
    {
      if (sensorValues[i] == 1 && weights[i] > maxRightDetections)
      {
        maxRightDetections = weights[i];
      }
    }
    Serial.print(maxRightDetections);

    // Calculate the weighted sum of sensor values
    Input = (maxLeftDetections * 10) - (maxRightDetections * 10);

    // Compute PID output
    float position = Input;                                          // Lee la posición de la línea
    error = setpoint - position;                                     // Calcula el error
    integral += error;                                               // Acumula el error para el término integral
    derivative = error - previous_error;                             // Calcula el cambio en el error
    correction = (KP * error) + (KI * integral) + (KD * derivative); // Calcula la corrección
    previous_error = error;                                          // Actualiza el error previo

    Output = correction;

    motorspeedL = constrain(SPEED_BASE + correction, MOTORS_MIN_PWM_VALUE, MOTORS_MAX_PWM_VALUE);
    motorspeedR = constrain((SPEED_BASE - correction) - 18, MOTORS_MIN_PWM_VALUE, MOTORS_MAX_PWM_VALUE - 18);

    // Motor izquierdo
    if (motorspeedL > 0)
    {
      digitalWrite(PIN_MOTOR_L_1, HIGH);
      digitalWrite(PIN_MOTOR_L_2, LOW);
      analogWrite(PIN_MOTOR_L_PWM, motorspeedL);
    }
    else if (motorspeedL < 0)
    {
      digitalWrite(PIN_MOTOR_L_1, LOW);
      digitalWrite(PIN_MOTOR_L_2, HIGH);
      analogWrite(PIN_MOTOR_L_PWM, -motorspeedL); // multiplica por -1 porque el valor es negativo
    }
    else
    {
      digitalWrite(PIN_MOTOR_L_1, LOW);
      digitalWrite(PIN_MOTOR_L_2, LOW);
      analogWrite(PIN_MOTOR_L_PWM, 0);
    }

    // Motor derecho
    if (motorspeedR > 0)
    {
      digitalWrite(PIN_MOTOR_R_1, HIGH);
      digitalWrite(PIN_MOTOR_R_2, LOW);
      analogWrite(PIN_MOTOR_R_PWM, motorspeedR);
    }
    else if (motorspeedR < 0)
    {
      digitalWrite(PIN_MOTOR_R_1, LOW);
      digitalWrite(PIN_MOTOR_R_2, HIGH);
      analogWrite(PIN_MOTOR_R_PWM, -motorspeedR);
    }
    else
    {
      digitalWrite(PIN_MOTOR_R_1, LOW);
      digitalWrite(PIN_MOTOR_R_2, LOW);
      analogWrite(PIN_MOTOR_R_PWM, 0);
    }

    Serial.print(" | ");

    // Monitorización por serial
    Serial.print("SA: ");
    for (int i = 0; i < CANT_ANALOG_SENSORS; i++)
    {
      Serial.print(analogSensorValues[i]);
      Serial.print(" ");
    }
    Serial.print(" | ");
    for (int i = 0; i < CANT_ALL_SENSORS; i++)
    {
      Serial.print(sensorValues[i]);
      Serial.print(" ");
    }

    Serial.print(" | Input: ");
    Serial.print(Input);
    Serial.print(" | Output: ");
    Serial.print(Output);
    Serial.print(" | Motor Speed R: ");
    Serial.print(motorspeedL);
    Serial.print(" | Motor Speed L: ");
    Serial.println(motorspeedR);
    Serial.print(" | Kp: ");
    Serial.print(KP, 2);
    Serial.print(" | Ki: ");
    Serial.print(KI, 2);
    Serial.print(" | Kd: ");
    Serial.print(KD, 2);
    Serial.print(" | ");

    // Check for serial input
    if (Serial.available())
    {
      String input = Serial.readStringUntil('\n');
      input.trim();

      // Parse the input and update PID constants
      if (input.startsWith("Kp"))
      {
        KP = input.substring(3).toFloat();

        Serial.print("Kp updated to: ");
        Serial.println(KP);
      }
      else if (input.startsWith("Ki"))
      {
        KI = input.substring(3).toFloat();

        Serial.print("Ki updated to: ");
        Serial.println(KI);
      }
      else if (input.startsWith("Kd"))
      {
        KD = input.substring(3).toFloat();
        Serial.print("Kd updated to: ");
        Serial.println(KD);
      }
    }

    delay(1); // Ajusta el retardo según sea necesario

    runTimer = currentTime;
    digitalWrite(PIN_LED, HIGH);
  }
}

void stop(unsigned int currentTime)
{
  if ((currentTime - blinkTimer) >= blinkDelay)
  {
    analogWrite(PIN_MOTOR_L_PWM, 0);
    analogWrite(PIN_MOTOR_R_PWM, 0);
    digitalWrite(PIN_LED, blink);
    blink = !blink;
    blinkTimer = currentTime;
    memory = 0;
  }
}

void loop()
{
  unsigned long currentTime = millis();
  if (state)
  {
    run(currentTime);
  }
  else
  {
    stop(currentTime);
  }

  if (button_state && !digitalRead(PIN_BUTTON))
  {
    state = !state;
    Serial.println(state ? "RUN" : "STOP");
  }

  button_state = digitalRead(PIN_BUTTON);
  delay(1);
}
