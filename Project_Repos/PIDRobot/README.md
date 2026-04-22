# PIDRobot


## Debug / Tuning Commands

| Command  | Description |
|----------|------------|
| `KP=<value>` | Sets the **proportional gain (KP)**. Example: `KP=1.5` |
| `KI=<value>` | Sets the **integral gain (KI)**. Example: `KI=0.01` |
| `KD=<value>` | Sets the **derivative gain (KD)**. Example: `KD=0.5` |
| `SHOW` | Displays the current values of KP, KI, and KD. |
| `DBG=1` | Enables **debugging mode**, printing extra details. |
| `DBG=0` | Disables **debugging mode**. |

### Using commands 
1. **Open the serial monitor** in the Arduino IDE.
2. Type a command and press **Enter**.
3. Use `SHOW` command to verify changes 

