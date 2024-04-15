#  * Driver's ports description for controlling block *

Port of driver information

1  - Black = GND 
2  - Green = INT-SPEED - if ON, Speed setting using internal volume (LOAD/SPEED) in the margin of the driver. if OFF, Set speed using external volume.
3  - Purple = Alarm out - the default signal is OFF, change to ON when the errors occurs.
4  - Orange = Speed out - OUTPUT for calculating the number or rotation of the motor.
5  - Gray = Alarm reset - Reset alarm to the default.
6  - Brown = CW-CCW - set the rotation for the motor - clockwise or counter clockwise.
7  - White = BRK & PULSE_IN ~ RUN and BRAKE - when turned ON, the motor starts. When turned OFF, the motor stop immediately.
8  - Pink = Start/Stop -ON means the motor is ready to rotate. If it turns OFF while motor is running, it stops naturally.
9  - Blue = GND
10 - Yellow = Speed in - The input range of speed settinng is 0-5V proportional control.
11 - Red = 5V for supplying DC power 

# *Notes for the state of the motor while turning ON/OFF Start/stop and Brk_pulse_in*
       _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _
      | Start/Stop | Run/Brake |        Driving status        |
       _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ 
      |     ON     |     ON    | Normal operation             | 
      |     ON     |     OFF   | Stop immediately             |
      |     OFF    |     ON    | Stop naturally without brake |
      |     OFF    |     OFF   | No activation of motor       |
       - - - - - - - - - - - - - - - - - - - - - - - - - - - - 
   => For the third row, we can understand that RUN/BRAKE still be on, it means that BRAKE is not activated.
   So that the vehicle stop naturally based on inertia of the motor and the environment.
  */

  // Special note for Run/Brake connection, port BRK&PULSE_IN need to be connected to ground

  # function for running motor 

  void run_motor(int v){
  digitalWrite(INT_SPEED, LOW);    - this port allow to controll by voltage signal from microcontroller
  digitalWrite(rotation, LOW);     - this port decide the rotation of the motor, we can adjust high or low depend on our desire
  digitalWrite(brk_pulse_in, LOW); - this port decide the state of the brake, when running, it should be LOW
  digitalWrite(start_stop, LOW);   - this port decide the state of the driver, it should be LOW - in START state
  analogWrite(pulse_out,v);        - this port transfer voltage signal from micro-controller to driver to set desire performance for motor
} 