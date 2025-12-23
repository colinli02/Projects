# Micro-controller Projects

## **Robotic Coin Picker (March 2022 - April 2022)**

- PIC32 micro-controller controlled and battery powered robot coin picker
- Powered by C software for robot wheel control, magnetic arm movement, and boundary perimeter detection of AC perimeter
- Debugged and optimised software and hardware issues

<div class="video-wrapper">
  <iframe width="1280" height="720" src="https://www.youtube.com/embed/euFSMJPZnGc" frameborder="0" allowfullscreen></iframe>
</div>

!!! quote "Links"
    [Github Code Link](https://github.com/colinli02/Projects/blob/main/Project_Repos/Robot_Base_Final/Robot_Base.c)

    [Raw Code Link](https://raw.githubusercontent.com/colinli02/Projects/main/Project_Repos/Robot_Base_Final/Robot_Base.c)

    [Wheel Code Separate](https://github.com/colinli02/Projects/blob/main/Project_Repos/Robot_Base_Final/Test_Codes/wheel.c)

    [Youtube Link](https://youtu.be/euFSMJPZnGc)

??? "Click to show code"
    ```c title="Robot_Base.c"
    --8<-- "https://raw.githubusercontent.com/colinli02/Projects/main/Project_Repos/Robot_Base_Final/Robot_Base.c"
    ```

## **Diode Heartrate Monitor (Feb. 2022 - March 2022)**

This first video shows a similar project of a temperature monitor using similar principles, based on the same code but with different hardware setups.

The circuit works by using the op-amp to amplify the weak signal emitted from the light passing through the tissue containing blood. 
Then this information is processed using the microcontroller circuit and 2 codes to generate a heart rate monitor.

[Temperature Monitor Demo](https://youtu.be/DtizvK82OfQ)

[Heart-rate Monitor Demo](https://youtu.be/MpQabdz20nY)
??? "Click to show Python code"
    ``` py linenums="1" title="heart-mtr_graphing.py"
    --8<-- "https://raw.githubusercontent.com/colinli02/Projects/refs/heads/main/Project_Repos/diode_hr_mtr/heart-mtr_graphing.py"

    ```

??? "Click to show C code"
    ``` c title="heart-mtr_at89.c"
    --8<-- "https://raw.githubusercontent.com/colinli02/Projects/refs/heads/main/Project_Repos/diode_hr_mtr/heart-mtr_at89.c"
    ```

- Developed AT89LP51RC2 micro-controller based photoelectric heart rate monitor
- Wrote software that would read micro-controller using PuTTY and C
- Further graphed data with Python using MatLab functions to graph heart rate

## **AC Voltmeter (March 2022)**

[Youtube Demo](https://youtu.be/pHFQ8-BD4R0)

??? "Click to show code"
    ``` c title="adc_voltmeter.c"
    --8<-- "https://raw.githubusercontent.com/colinli02/Projects/refs/heads/main/Project_Repos/ac_voltmeter/adc_spi_final%20(VOLTMETER).c"
    ```

- Built a software based AC voltmeter using C
- Understood AC circuit concepts to create voltmeter that determined magnitude, phase shift, and phase of AC signals produced by function generator
