# Table of all motors and CAN devices

| Name                     | Model              | ID  | PDH Port |
|--------------------------|--------------------|-----|----------|
| BACK_RIGHT_TURN_MOTOR    | Falcon 500         | 1   | TBD      |
| BACK_LEFT_DRIVE          | Kraken X60         | 2   | TBD      |
| FRONT_RIGHT_TURN_MOTOR   | Falcon 500         | 3   | TBD      |
| BACK_RIGHT_DRIVE_MOTOR   | Kraken X60         | 4   | TBD      |
| BACK_LEFT_TURN_MOTOR     | Falcon 500         | 5   | TBD      |
| FRONT_LEFT_DRIVE_MOTOR   | Kraken X60         | 6   | TBD      |
| FRONT_LEFT_TURN_MOTOR    | Falcon 500         | 8   | TBD      |
| FRONT_RIGHT_DRIVE_MOTOR  | Kraken X60         | 9   | TBD      |
| Pigeon 2 (Device ID 20)  | Pigeon 2           | 20  | TBD      |
| BACK_RIGHT_CANCoder      | CANCoder vers. H   | 21  | TBD      |
| FRONT_RIGHT_CANCoder     | CANCoder vers. H   | 22  | TBD      |
| BACK_LEFT_CANCoder       | CANCoder vers. H   | 23  | TBD      |
| FRONT_LEFT_CANCoder      | CANCoder vers. H   | 24  | TBD      |
| LEFT_PIVOT_1             | Falcon 500         | 31  | TBD      |
| LEFT_PIVOT_2             | Falcon 500         | 32  | TBD      |
| RIGHT_PIVOT_1            | Falcon 500         | 33  | TBD      |
| RIGHT_PIVOT_2            | Falcon 500         | 34  | TBD      |
| ELEVATOR                 | Kraken X60         | 40  | TBD      |
| END_EFFECTOR             | NEO 550            | 41  | TBD      |
| ALGAE_MANIPULATOR        | NEO 550            | 42  | TBD      |
| WRIST_CANCoder           | CANCoder vers. H   | 50  | TBD      |
| WRIST_MOTOR              | Kraken X60         | 51  | TBD      |


## [Kraken Status Lights](https://docs.wcproducts.com/kraken-x60/kraken-x60-+-talonfx/status-lights)
| LED State               | Description                                                                                     |
|-------------------------|-------------------------------------------------------------------------------------------------|
| Alternating Off/Orange  | Talon FX is disabled. Robot controller is missing on the bus or the diagnostic server is not installed. |
| Simultaneous Off/Orange | Talon FX is disabled. Phoenix is running in Robot Controller.                                   |
| Alternating Red/Green   | Talon FX is not licensed. Please license device in Phoenix Tuner.                               |
| Off/Slow Red            | CAN/PWM is not detected.                                                                        |
| Red/Orange              | Damaged Hardware                                                                               |
| Off/Red                 | Limit Switch or Soft Limit triggered.                                                          |
| Green/Orange            | Device is in bootloader.                                                                       |

## [Falcon 500 Status Lights](https://v6.docs.ctr-electronics.com/en/2024/docs/hardware-reference/talonfx/index.html)
| LED State                         | Cause                                                                                   | Possible Fix                                                                                                     |
|-----------------------------------|-----------------------------------------------------------------------------------------|------------------------------------------------------------------------------------------------------------------|
| LEDs Off                          | No Power                                                                                | Provide 12V to Red/Black leads.                                                                                 |
| Blinking Alternating Red          | Talon FX does not have a valid CAN/PWM signal.                                          | Ensure good connections between CANH and CANL (Yellow and Green) & robot controller is on.                      |
| Blinking Alternating Orange       | TalonFX detects CAN but does not see Phoenix running on the robot controller.           | If Phoenix is running on the robot controller, ensure good connection between the controller and this device. Otherwise, deploy a robot program that uses Phoenix. |
| Blinking Simultaneous Orange      | Talon FX has valid CAN signal and is disabled. Phoenix is running in robot controller and Talon FX has good CAN connection to robot controller. | If robot is enabled, ensure a control request is being sent to the Talon FX.                                    |
| **Enabled Codes**                 |                                                                                         |                                                                                                                  |
| Both Solid Orange                 | Talon FX enabled with neutral output.                                                  |                                                                                                                  |
| Blinking Simultaneous Red         | Talon FX driving in reverse. Rate of blink corresponds to duty cycle applied.           |                                                                                                                  |
| Blinking Simultaneous Green       | Talon FX driving forward. Rate of blink corresponds to duty cycle applied.             |                                                                                                                  |
| Offset Alternating Red/Off        | Talon FX limited (hard or soft limit). Direction of offset determines forward/reverse limit. |                                                                                                                  |
| **Special Codes**                 |                                                                                         |                                                                                                                  |
| Offset Orange/Off                 | Talon FX in thermal cutoff.                                                             | Allow Talon FX to cool. Consider configuring Stator Current Limits to reduce heat generation.                   |
| Alternate Red/Green               | Talon FX driven with Pro-only command while unlicensed.                                 | Use non-Pro-only command, or license device for Pro.                                                            |
| Alternate Red/Orange              | Damaged Hardware.                                                                       | Contact CTRE.                                                                                                   |
| Single LED alternates Green/Orange| Talon FX in bootloader.                                                                 | Field-upgrade device in Tuner X.                                                                                |



## [CANCoder Status Light](https://v6.docs.ctr-electronics.com/en/2024/docs/hardware-reference/cancoder/index.html#cancoder)
| LED State               | Cause                                                                                          | Possible Fix                                                                                                                                                           |
|-------------------------|------------------------------------------------------------------------------------------------|----------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| LED Off                | No Power                                                                                       | Provide 12V to Red/Black leads.                                                                                                                                      |
| Slow Bright Red        | CANcoder does not have valid CAN.                                                              | Ensure good connections between CANH and CANL (Yellow and Green) & robot controller is on.                                                                           |
| Rapid Dim Red          | CAN bus never detected since boot, CANcoder now reporting strength of magnet. Magnet is out of range (<25 mT or >135 mT). | Ensure good connections between CANH and CANL (Yellow and Green) & robot controller is on. Additionally, ensure the magnet's center axis is aligned with the defined center of the CANcoder housing and the magnet is in range of the CANcoder. See Section 2.1 of the Hardware User Manual. |
| Rapid Dim Orange       | CAN bus never detected since boot, CANcoder now reporting strength of magnet. Magnet is in range with slightly reduced accuracy (25-45 mT or 75-135 mT). | Ensure good connections between CANH and CANL (Yellow and Green) & robot controller is on. Additionally, ensure the magnet's center axis is aligned with the defined center of the CANcoder housing and the CANcoder is not too close or too far from the magnet. See Section 2.1 of the Hardware User Manual. |
| Rapid Dim Green        | CAN bus never detected since boot, CANcoder now reporting strength of magnet. Magnet is in range. | Ensure good connections between CANH and CANL (Yellow and Green) & robot controller is on.                                                                           |
| Rapid Bright Red       | CAN bus healthy. Magnet is out of range (<25 mT or >135 mT).                                   | Ensure the magnet's center axis is aligned with the defined center of the CANcoder housing and the magnet is in range of the CANcoder. See Section 2.1 of the Hardware User Manual. |
| Rapid Bright Orange    | CAN bus healthy. Magnet is in range with slightly reduced accuracy (25-45 mT or 75-135 mT).    | Ensure the magnet's center axis is aligned with the defined center of the CANcoder housing and the CANcoder is not too close or too far from the magnet. See Section 2.1 of the Hardware User Manual. |
| Rapid Bright Green     | CAN bus healthy. Magnet is in range.                                                           | -                                                                                                                                                                    |
| Alternate Red/Orange   | Damaged Hardware.                                                                              | Contact CTRE.                                                                                                                                                        |
| Alternate Orange/Green | CANcoder in bootloader.                                                                        | Field-upgrade device in Tuner X.                                                                                                                                    |

## [SparkMax Status Lights](https://docs.revrobotics.com/brushless/spark-max/status-led)
| Operating Mode | State                | Color/Pattern               |
|----------------|----------------------|-----------------------------|
| **Idle Mode**  | **Brushed**          |                             |
|                | Brake - No Signal    | Blue Blink                  |
|                | Brake - Valid Signal | Blue Solid                  |
|                | Coast - No Signal    | Yellow Blink                |
|                | Coast - Valid Signal | Yellow Solid                |
| **Brushless**  |                      |                             |
|                | Brake - No Signal    | Cyan Blink                  |
|                | Brake - Valid Signal | Cyan Solid                  |
|                | Coast - No Signal    | Magenta Blink               |
|                | Coast - Valid Signal | Magenta Solid               |
|                | Partial Forward      | Green Blink                 |
|                | Full Forward         | Green Solid                 |
|                | Partial Reverse      | Red Blink                   |
|                | Full Reverse         | Red Solid                   |
|                | Forward Limit        | Green/White Blink           |
|                | Reverse Limit        | Red/White Blink             |

| **Identification, Updating, and Recovery** | Mode                          | Color/Pattern               |
|--------------------------------------------|-------------------------------|-----------------------------|
|                                            | Device Identify               | White/Magenta Fast Blink    |
|                                            | CAN Bootloader - Firmware Updating | White/Yellow Blink (v1.5.0) / Green/Magenta Blink (v1.4.0) |
|                                            | CAN Bootloader - Firmware Retry | White/Blue Blink            |
|                                            | USB DFU (Device Firmware Update) | Dark (LED off)              |
|                                            | Recovery Mode                 | Dark (LED off)              |

| **Fault Conditions** | Fault Condition            | Color/Pattern               |
|-----------------------|----------------------------|-----------------------------|
| 12V Missing          | The motor will not drive if powered only by USB. This blink code warns the user of this condition. | Orange/Blue Slow Blink          |
| Sensor Fault         | This can occur if the sensor type is misconfigured, the sensor cable is not plugged in or damaged, or if a sensor other than the motor sensor is plugged in. | Orange/Magenta Slow Blink       |
| Gate Driver Fault    | A fault reported by the core internal electronic circuitry. If this code persists after power cycling the controller, contact REV. | Orange/Cyan Slow Blink          |
| CAN Fault            | The CAN fault will be shown after the first time the device is plugged into the CAN port and a fault later occurs. Check your CAN wiring if you see this fault. | Orange/Yellow Slow Blink        |
| Corrupt Firmware     | Firmware failed to load. Recover using Recovery Mode. | Dark (LED off)                |
