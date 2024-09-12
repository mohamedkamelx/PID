# Arduino PID Motor Speed Control with Exponential Smoothing

This code implements a PID controller to control the speed of a motor.the PID algorithm is used to adjust the control signal based on the error between the target and the actual speed.
a exponential filter to smooth the signal .

## Components

- **PID Controller**: A feedback control system that adjusts the motor speed to minimize the error between the setpoint (target speed) and the actual speed.

    - formula :
      
        $$\text{Output} = K_p \cdot \text{error} + K_i \cdot \int \text{error} \, dt + K_d \cdot \frac{d(\text{error})}{dt}$$


    - #### PID Class

        ```cpp
        class PIDcontroller {
        private:
            float Kp, Ki, Kd;
            float setpoint;
            float previousError;
            float integral;
            unsigned long previousTime;

        public:
            PIDcontroller(float p, float i, float d) {
            Kp = p;
            Ki = i;
            Kd = d;
            previousError = 0;
            integral = 0;
            previousTime = millis();
            }

            void setSetpoint(float sp) {
            setpoint = sp;
            }

            float compute(float currentSpeed) {
            unsigned long currentTime = millis();
            float time_interval = (currentTime - previousTime) / 1000.0;  // Time in seconds
            previousTime = currentTime;

            float error = setpoint - currentSpeed;
            integral += error * time_interval;
            float derivative = (error - previousError) / time_interval;

            previousError = error;
            
            // PID output calculation formula
            float output = Kp * error + Ki * integral + Kd * derivative;
            
            return output;
            }
        };
        ```


- **Exponential filter**: Used to smooth out the input speed value from the sensor, preventing sudden jumps in the control signal.

    - **Exponential filter function**

        ```cpp
        float exponential_smoothing(float data) {
            // exponential filter formula
            float smoothedValue = alpha * data + (1 - alpha) * previousvalue;
            return smoothedValue;
        }
        ```

    - formula :

        $$\text{y[n]} = \alpha \cdot \text{x[n]} + (1 - \alpha) \cdot \text{y[n-1]}$$
