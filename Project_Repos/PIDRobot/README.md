# PIDRobot

## Bluetooth
https://github.com/Mohamadol/Flutter_Arduino_Bluetooth/

## Commands

| Command  | Description |
|----------|------------|
| `KP=<value>` | Sets the **proportional gain (KP)**. Example: `KP=1.5` |
| `KI=<value>` | Sets the **integral gain (KI)**. Example: `KI=0.01` |
| `KD=<value>` | Sets the **derivative gain (KD)**. Example: `KD=0.5` |
| `SHOW` | Displays the current values of KP, KI, and KD. |

### Using commands 
1. **Open the serial monitor** in the Arduino IDE.
2. Type a command and press **Enter**.
3. Use `SHOW` command to verify changes 

### 
If encoders are not giving signal, code will be stuck in while(1)
Robot will not move motors if no encoders.

