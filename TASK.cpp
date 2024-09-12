float previousvalue = 0;

class PIDcontroller {
    private:
    float Kp, Ki, Kd;
    float setpoint;
    float previousError;
    unsigned long previousTime;
    
  public:
    PIDcontroller(float p, float i, float d) {
      Kp = p;
      Ki = i;
      Kd = d;
      previousError = 0;
      previousTime = millis();
    }

    void setpoint(float sp) {
      setpoint = sp;
    }

    float compute(float currentSpeed) {

      unsigned long currentTime = millis();
      float time_interval = (currentTime - previousTime) / 1000.0;  // Time in seconds
      previousTime = currentTime;

      float error = setpoint - currentSpeed;
      
      float integral = 0;
      integral += error * time_interval; // calculating the integration approximatly as area of rectangle
      float derivative = (error - previousError) / time_interval;  // calculate derivative as slope

      previousError = error;
      
      // PID output calculation formula
      float output = Kp * error + Ki * integral + Kd * derivative;
      
      return output;
    }
};


PIDcontroller pid(2.0, 0.5, 1.0); //kp , ki , kd values


const float alpha = 0.1; // Smoothing factor 

float exponential_smoothing(float data) {
    // exponential filter formula
      float smoothedValue = alpha * data + (1 - alpha) * previousvalue;
      return smoothedValue;
    }



const int motorPin = 9;    // Motor control pin
const int velocityCalcEncoderPin = A0;  // Motor speed sensor pin or encoder or any thing that could return speed 

void setup() {
    
    pinMode(motorPin, OUTPUT);
    Serial.begin(9600);
    pid.setpoint(100); // setpoint of the PID
}

void loop() {
    // Read raw sensor value
    int speed = analogRead(velocityCalcEncoderPin);
    
    // Apply exponential smoothing to sensor value
    float smoothedSpeed = exponential_smoothing(speed);
    previousvalue = smoothedSpeed

    // Compute PID output
    float controlSignal = pid.compute(smoothedSpeed);
    
    // This signal needs to be converted to PWM first before sending it to the motor 
    // or scale it to be in range of 255
    //then you pass it 
    //   vvvvvvvv
    analogWrite(motorPin, controlSignal);
    
    // Print values for debugging
    Serial.print("Raw Speed: ");
    Serial.print(speed);
    Serial.print(" Smoothed Speed: ");
    Serial.print(smoothedSpeed);
    Serial.print(" Control Signal: ");
    Serial.println(controlSignal);

    delay(100); // Adjust delay as needed
}
