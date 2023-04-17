# Quad motor

This is a simple quad motor controller for arduino boards

## Serial commands

The serial commands are:
'x' - stop all motors
'o <motor> <speed>' - start motor <motor> with pwm speed <speed>
'c <motor> <speed>' - change speed of motor <motor> to <speed>
'p <motor> <position>' - set motor <motor> to position <position>
'r' - reset all motors to 0 position
'e' - print encoder values for all motors
's' - print motor speeds for all motors
'd <p/i/d> <motor> <value>' - set the P I or D value for motor <motor> to <value>


Return values:
OK - command executed
ERR - command not executed
alternatively, the return value can be the value of the command
'e' - return encoder values "<int> <int> <int> <int>"
's' - return motor speeds "<int> <int> <int> <int>"


## Notes

* Multiple pid controllers, one for position and one for speed?
* Save pid values to eeprom?