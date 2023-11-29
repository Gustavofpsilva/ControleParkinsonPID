#include <Wire.h>
#include <Servo.h>
#include <MPU6050.h>

MPU6050 mpu;

Servo servoMotor;

const int servoPin = 9;  // Pino de controle do servo

// Parâmetros PID
double setpoint = 0;   // Ângulo desejado
double input, output;
double Kp = 2.5;        // Ganho proporcional
double Ki = 0.1;        // Ganho integral
double Kd = 0.01;       // Ganho derivativo

unsigned long currentTime, previousTime;
double elapsedTime;
double error, lastError;
double cumError, rateError;

void setup() {
  Serial.begin(9600);
  Wire.begin();
  mpu.initialize();

  servoMotor.attach(servoPin);
  servoMotor.write(90); // Posição inicial do servo
  delay(1000); // Aguarda o servo se posicionar

  previousTime = millis();
}

void loop() {
  int16_t ax, ay, az;
  mpu.getAcceleration(&ax, &ay, &az);

  // Calcula o ângulo de inclinação em relação ao eixo X
  input = atan2(ay, az) * (180.0 / M_PI);

  // Calcula o erro
  error = setpoint - input;

  currentTime = millis();
  elapsedTime = (double)(currentTime - previousTime) / 1000;

  // Calcula as componentes PID
  double P = Kp * error;
  cumError += error * elapsedTime;
  double I = Ki * cumError;
  rateError = (error - lastError) / elapsedTime;
  double D = Kd * rateError;

  // Calcula a saída PID
  output = P + I + D;

  // Ajusta a posição do servo com base na saída PID
  int servoPosition = constrain(90 + output, 0, 180);
  servoMotor.write(servoPosition);

  // Exibe as leituras para fins de depuração
  Serial.print("Input: ");
  Serial.println(input);
  Serial.print("Output: ");
  Serial.println(output);
  Serial.print("Servo Position: ");
  Serial.println(servoPosition);

  // Atualiza variáveis para a próxima iteração
  lastError = error;
  previousTime = currentTime;

  delay(100); // Aguarda um curto período para evitar leituras muito rápidas
}
