Controller gains tuning is one of the most important parts of controller development. Gains are tuned by different algorithms. one of the popular algorithms is Fuzzy.
This code is written for temperature control using an Adaptive type-II fuzzy - PID controller, specifically a DS18B20 sensor. It's worth mentioning that all components of the controller, have been manually implemented in this code without utilizing any pre-built functions, solely relying on fundamental commands of the C programming language. This controller was simulated in MATLAB and then coded for microcontroller. In other words, this code is the second stage of controller development, in which real DS18B20 sensor data has been employed.

The general architecture of this controller is as follows:<br>
1. **Error Calculation**:
   - The error between the setpoint and the actual temperature is calculated.

2. **Fuzzy Membership Functions (MF)**:
   - Membership functions (MF) are defined for each input variable (error, integral of error, derivative of error) and for each linguistic variable (e.g., low, medium, high).
   - Two sets of membership functions are defined: one for "upper" (MFu) and one for "lower" (MFl) to represent the antecedent (input) and consequent (output) fuzzy sets.

3. **Fuzzy Inference**:
   - For each linguistic term (e.g., "low error", "medium error", "high error"), the degree of membership of the error in that linguistic term is computed using the Gaussian function.
   - These membership values are used to calculate the degree of firing of each rule.

4. **Fuzzy Rule Evaluation**:
   - Fuzzy rules define the relationship between input variables (error, integral of error, derivative of error) and output variables (Kp, Ki, Kd).
   - Each rule has an associated weight (wu_kp, wl_kp, wu_ki, wl_ki, wu_kd, wl_kd) which represents the contribution of that rule to the output.

5. **Defuzzification**:
   - The weighted average of the output values is computed to determine the final values of Kp, Ki, and Kd.

6. **PID Controller Output Calculation**:
   - The PID controller output is computed using the classic formula: output = Kp * error + Ki * integral + Kd * derivative.

7. **Adjustment of Fuzzy Rules**:
   - The weights of the fuzzy rules are adjusted based on the error, its rate of change, and the change in output.<br>

This code is written in C language, but due to the requirements of Arduino's framework, some C++ functions are used in it, but the distinctive features of C++, such as object orientation, are not present in it.<br>
For quastions: <a href="https://mirmikael.github.io/" target="_blank">MirMikael.github.io</a>
