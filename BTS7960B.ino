// Dual DC Motor control
// Arduino UNO R3
// Cytron MD13S: 6-30V, 13A (30A peak current - 10 sec) - two pcs.
// SPDT Relay(30A): SLA-05VDC-SL-C
// Andrey Zhuravlev
// email: v.azhure@gmail.com
// 2019

// uncomment next line if you use temperature sensor
// #define USE_TEMP_SENSOR

#include <EEPROM.h>
#include "protocol.h"
#include "BTS7960.h"

#ifdef USE_TEMP_SENSOR
#include <OneWire.h>
#endif

#define BYTE_MAX 255

// override Serial Port buffer size
#define SERIAL_TX_BUFFER_SIZE 64
#define SERIAL_RX_BUFFER_SIZE 128

// Pins setting
#define RELAY_PIN 4
#define M1_PWMr_PIN 10
#define M1_PWMl_PIN 9
#define M1_ENr_PIN 8
#define M1_ENl_PIN 7
//#define M2_PWM_PIN 6
//#define M2_DIR_PIN 8
BTS7960 motorController1(M1_ENl_PIN, M1_ENr_PIN, M1_PWMl_PIN, M1_PWMr_PIN);

#ifdef USE_TEMP_SENSOR
#define TEMP_SENSOR_PIN 2
#endif

// Rotary potentiometers
#define ROT_POT1_PIN A0
//#define ROT_POT2_PIN A1
//#define ROT_POTEXT_PIN A2

// Potentiometer full rotary angle in degrees
#define POTFULL_ANGLE 250.0f
#define POT_EXT_FULL_ANGLE 250.0f
#define PWM_TOLERANCE_DEG 5
#define PWM_MAX_ACC_DEG 70
#define RANGE_TOLERANCE 0.1
#define ONE_MINUS_RANGE_TOLERANCE (1.0 - RANGE_TOLERANCE)

// default values
#define PID_FREQUENCY 100 // Hz
#define MAX_PWM 127       // 50%
#define LOW_PWM 64        // Low speed PWM

// Serial port settings
#define SERIAL_PORT_SPEED 57600
// Relay related constant
#define POWER_ON_DELAY_MS 5000
// Device name
#define DEVICE_NAME "AM_MOTION_DEVICE"

// previous values
float _pwm1 = 0.f;
//float _pwm2 = 0.f;

typedef enum DEVICE_STATE
{
    SHUTDOWN,
    STARTUP,
    READY,
    PARKING,
    EMERGENCY_STOP
};

// Current state
volatile DEVICE_STATE _device_state = DEVICE_STATE::SHUTDOWN;

// Potentiometer name
typedef enum POT_NAME
{
    POT1
//    POT2,
//    POT_EXT
};

// Motor name
typedef enum MOTOR_NAME
{
    MOTOR1,
//    MOTOR2,
};

// Motor direction
typedef enum MOTOR_DIRECTION
{
    // Stop
    STOP,
    // Clockwise
    CW,
    // Counter-clockwise
    CCW,
};

#define EEPROM_DATA_VER1 101
#define EEPROM_DATA_VER2 102 // pwm_max added
#define EEPROM_DATA_VER3 103 // PID using flag added
#define EEPROM_DATA_VER4 104 // acceleration / deceleration added
#define EEPROM_DATA_VER EEPROM_DATA_VER4
#define EEPROM_DATA_OFFSET 0

// EEPROM data
typedef struct MotorsStruct
{
    byte version; // data version
    //[kp, ki, kd]
    float pid1[3]; // M1 PID data
    //[kp, ki, kd]
//    float pid2[3];      // M2 PID data
    int pot_offset[2];  // Potentiometer zero offset
    float m1_limits[2]; // Motor 1 limits [min, max] in degrees
//    float m2_limits[2]; // Motor 2 limits [min, max] in degrees
    float frequency;    // PID frequency
    byte pwm_max;       // Max PWM for motors
    bool bUsePID;       // PID using flag
    float dAcc;         // Acceleration delta
    float dDec;         // Deceleration delta
    int pwm_point1[2];  // [deg, pwm]
    int pwm_point2[2];  // [deg, pwm]
};

// Global variables
MotorsStruct _global;
// Motors enable flags
volatile bool m_motorEnabled[2];
// Target position, degrees
volatile float m_position[2];
// Encoder / Potentiometer position, degrees
volatile float m_encoderAngle[2];

#define WDT_SLEEP_CYCLES 30000
// empty cycles counter
volatile long _wdt = 0;

#ifdef USE_TEMP_SENSOR
OneWire ts(TEMP_SENSOR_PIN);
#endif

// Power on settings
void setup()
{
//    setPwmFrequency(M1_PWM_PIN, 8); // About 7.8 kHz
//    setPwmFrequency(M2_PWM_PIN, 8); // About 7.8 kHz

    // Digital outputs
    pinMode(M1_PWMr_PIN, OUTPUT);
    pinMode(M1_PWMl_PIN, OUTPUT);
    pinMode(M1_ENr_PIN, OUTPUT);
    pinMode(M1_ENl_PIN, OUTPUT);
//    pinMode(M2_PWM_PIN, OUTPUT);
//    pinMode(M2_DIR_PIN, OUTPUT);
    pinMode(RELAY_PIN, OUTPUT);

    // Analogue inputs
    pinMode(ROT_POT1_PIN, INPUT);
//    pinMode(ROT_POT2_PIN, INPUT);

    // All motors STOP
    analogWrite(M1_ENr_PIN, LOW);
    digitalWrite(M1_ENl_PIN, LOW);
//    analogWrite(M2_PWM_PIN, LOW);
//    digitalWrite(M2_DIR_PIN, LOW);

    // Target position
    m_position[MOTOR_NAME::MOTOR1] = 0;
//    m_position[MOTOR_NAME::MOTOR2] = 0;

    // Enable motors
//    m_motorEnabled[MOTOR_NAME::MOTOR1] =
//        m_motorEnabled[MOTOR_NAME::MOTOR2] = false;
    m_motorEnabled[MOTOR_NAME::MOTOR1] = false;

    // Current position
    m_encoderAngle[POT_NAME::POT1] = GetAngle(POT_NAME::POT1, POTFULL_ANGLE);
//    m_encoderAngle[POT_NAME::POT2] = GetAngle(POT_NAME::POT2, POTFULL_ANGLE);

    // Load setting from EEPROM
    LoadSettings();

    // Start communication
    Serial.begin(SERIAL_PORT_SPEED);

    _device_state = DEVICE_STATE::READY;
}

#define _DEBUG

// Main loop
void loop()
{
//    ReadTemperature();

    // read actual position
    m_encoderAngle[POT_NAME::POT1] = GetAngle(POT_NAME::POT1, POTFULL_ANGLE);
//    m_encoderAngle[POT_NAME::POT2] = GetAngle(POT_NAME::POT2, POTFULL_ANGLE);

#ifdef _DEBUG
    Serial.print("Pot1:");
    Serial.print(m_encoderAngle[POT_NAME::POT1]);
    Serial.print(",");
//    return;
#endif

    // Check our potentiometers values in range
    if (
      m_encoderAngle[POT_NAME::POT1] < _global.m1_limits[0] || m_encoderAngle[POT_NAME::POT1] > _global.m1_limits[1]
//      ||
//      m_encoderAngle[POT_NAME::POT2] < _global.m2_limits[0] || m_encoderAngle[POT_NAME::POT2] > _global.m2_limits[1]
      )
    {
#ifndef _DEBUG
        EmergencyStop();
        return;
#endif
    }
    else
    {
        _device_state = DEVICE_STATE::READY;
    }

    // clamp motors position software limits
    float pos1 = constrain(m_position[MOTOR_NAME::MOTOR1], _global.m1_limits[0] * ONE_MINUS_RANGE_TOLERANCE, _global.m1_limits[1] * ONE_MINUS_RANGE_TOLERANCE);
//    float pos2 = constrain(m_position[MOTOR_NAME::MOTOR2], _global.m2_limits[0] * ONE_MINUS_RANGE_TOLERANCE, _global.m2_limits[1] * ONE_MINUS_RANGE_TOLERANCE);

#ifdef _DEBUG
    Serial.print("Pos1:");
    Serial.println(pos1);
//    Serial.print(",");
//    Serial.print("Pos2:");
//    Serial.println(pos2);
//    return;
#endif

    float delta1 = pos1 - m_encoderAngle[POT_NAME::POT1];
//    float delta2 = pos2 - m_encoderAngle[POT_NAME::POT2];

    float fdelta1 = fabs(delta1);
//    float fdelta2 = fabs(delta2);

    // Small steps at small speed, large steps at fast speed

    int pwm1 = (int)(fdelta1 < PWM_TOLERANCE_DEG ? mapf(fdelta1, 0, _global.pwm_point1[0], 0, _global.pwm_point1[1]) : mapf(fdelta1, _global.pwm_point1[0], _global.pwm_point2[0], _global.pwm_point1[1], _global.pwm_point2[1]));
//    int pwm2 = (int)(fdelta2 < PWM_TOLERANCE_DEG ? mapf(fdelta2, 0, _global.pwm_point1[0], 0, _global.pwm_point1[1]) : mapf(fdelta2, _global.pwm_point1[0], _global.pwm_point2[0], _global.pwm_point1[1], _global.pwm_point2[1]));

    int sign1 = delta1 < 0 ? -1 : 1;
//    int sign2 = delta2 < 0 ? -1 : 1;

    // acceleration / deceleration delta
    float dt1 = pwm1 - _pwm1;
//    float dt2 = pwm2 - _pwm2;

    // acceleration / deceleration correction
    if (fabs(dt1) > _global.dAcc)
        _pwm1 = _pwm1 + (dt1 < 0 ? -_global.dAcc : _global.dAcc);

//    if (fabs(dt2) > _global.dAcc)
//        _pwm2 = _pwm2 + (dt2 < 0 ? -_global.dAcc : _global.dAcc);

    // store current values of PWM
    pwm1 = (int)_pwm1;
//    pwm2 = (int)_pwm2;

    // clamp PWM values in range [-255..255]
    pwm1 = constrain(pwm1, -BYTE_MAX, BYTE_MAX);
//    pwm2 = constrain(pwm2, -BYTE_MAX, BYTE_MAX);

    if (m_motorEnabled[MOTOR_NAME::MOTOR1] && _wdt < WDT_SLEEP_CYCLES)
    {
        MOTOR_DIRECTION dir = delta1 < 0 ? MOTOR_DIRECTION::CW : MOTOR_DIRECTION::CCW;
        Move(MOTOR_NAME::MOTOR1, dir, abs(pwm1));
    }
    else
    {
        Move(MOTOR_NAME::MOTOR1, MOTOR_DIRECTION::STOP, 0);
    }
/*
    if (m_motorEnabled[MOTOR_NAME::MOTOR2] && _wdt < WDT_SLEEP_CYCLES)
    {
        MOTOR_DIRECTION dir = delta2 < 0 ? MOTOR_DIRECTION::CW : MOTOR_DIRECTION::CCW;
        Move(MOTOR_NAME::MOTOR2, dir, abs(pwm2));
    }
    else
    {
        Move(MOTOR_NAME::MOTOR2, MOTOR_DIRECTION::STOP, 0);
    }
*/
    _wdt++;
} // loop

CMD _tempCmd;
// internal RX buffer
byte _rx_cmd[SERIAL_RX_BUFFER_SIZE];
// RX buffer offset
volatile size_t _rx_cmd_len = 0;

// Serial input process
void serialEvent()
{
    while (Serial.available())
    {
        if (_rx_cmd_len == 0)
        {
            do
            {
                _rx_cmd[0] = Serial.read(); // Seeking for header
            } while (_rx_cmd[0] != CMD_HEADER && Serial.available() > 0);
            if (_rx_cmd != CMD_HEADER)
                _rx_cmd_len = 1;
            continue;
        }

        if (_rx_cmd_len == 1 && Serial.peek() != CMD_LEN) // Already have header, check cmd length
        {
            _rx_cmd_len = 0;
            continue;
        }

        // read bytes from current offset
        _rx_cmd_len += Serial.readBytes(&_rx_cmd[_rx_cmd_len], CMD_LEN - _rx_cmd_len);

        if (_rx_cmd_len == CMD_LEN)
        {
            if (ProceedCmd(_rx_cmd))
            {
                _rx_cmd_len = 0;
            }
            else
            {
                _rx_cmd_len = 0;
            }
        }
    }
}

bool ProceedCmd(byte *pCmd)
{
    _tempCmd = *((CMD *)pCmd);

    if (_tempCmd.header == CMD_HEADER && _tempCmd.datasize == CMD_LEN)
    {
        // calculate CRC8
        byte crc = crc8((byte *)&_tempCmd, CMD_LEN - 1);
        // check CRC is correct
        if (crc == _tempCmd.crc)
        {
            switch (_tempCmd.cmd)
            {
            case CMD_START_UP:
                if (_device_state != DEVICE_STATE::EMERGENCY_STOP)
                {
                    // Set motor target position
                    m_position[MOTOR_NAME::MOTOR1] = 0;
//                    m_position[MOTOR_NAME::MOTOR2] = 0;
                    // Enable motors
                    m_motorEnabled[MOTOR_NAME::MOTOR1] = true;
//                    m_motorEnabled[MOTOR_NAME::MOTOR2] = true;
                    // Power on
                    SwitchRelay(true);
                    delay(POWER_ON_DELAY_MS);
                }
                break;
            case CMD_SHUT_DOWN:
                ShutDown();
                break;
            case CMD_EMERGENCY_MOVE:
            {
                if (_device_state == DEVICE_STATE::EMERGENCY_STOP)
                {
                    // Power on
                    SwitchRelay(true);
                    delay(POWER_ON_DELAY_MS);
                }
                int mot = (int)_tempCmd.data.f1;
                MOTOR_DIRECTION dir = _tempCmd.data.f2 > 0 ? MOTOR_DIRECTION::CW : MOTOR_DIRECTION::CCW;
                int t = (int)constrain(_tempCmd.data.f3, 0, 1000);
                // Start moving
                Move((MOTOR_NAME)mot, dir, LOW_PWM);
                // delay
                delay(t);
                // Stop moving
                Move((MOTOR_NAME)mot, MOTOR_DIRECTION::STOP, 0);
            }
            break;
            case CMD_SET_X:
                m_position[MOTOR_NAME::MOTOR1] = _tempCmd.data.f1;
                break;
            case CMD_SET_Y:
//                m_position[MOTOR_NAME::MOTOR2] = _tempCmd.data.f2;
                break;
            case CMD_SET_XY:
            case CMD_SET_XY_REPLY:
                m_position[MOTOR_NAME::MOTOR1] = _tempCmd.data.f1;
//                m_position[MOTOR_NAME::MOTOR2] = _tempCmd.data.f2;
                if (_tempCmd.cmd == CMD_SET_XY_REPLY)
                {
                    _tempCmd.data.f1 = m_encoderAngle[POT_NAME::POT1];
//                    _tempCmd.data.f2 = m_encoderAngle[POT_NAME::POT2];
                    _tempCmd.crc = crc8((byte *)&_tempCmd, CMD_LEN - 1);
                    Serial.write((byte *)&_tempCmd, CMD_LEN);
                }
                break;
            case CMD_SETPID1:
                _global.pid1[0] = _tempCmd.data.f1;
                _global.pid1[1] = _tempCmd.data.f2;
                _global.pid1[2] = _tempCmd.data.f3;
                break;
//            case CMD_SETPID2:
//                _global.pid2[0] = _tempCmd.data.f1;
//                _global.pid2[1] = _tempCmd.data.f2;
//                _global.pid2[2] = _tempCmd.data.f3;
//                break;
            case CMD_SET_STOREPID:
            {
                StorePidData();
            }
            break;
            case CMD_GETPID1:
            {
                _tempCmd.data.f1 = _global.pid1[0];
                _tempCmd.data.f2 = _global.pid1[1];
                _tempCmd.data.f3 = _global.pid1[2];
                _tempCmd.crc = crc8((byte *)&_tempCmd, CMD_LEN - 1);
                Serial.write((byte *)&_tempCmd, CMD_LEN);
            }
            break;
//            case CMD_GETPID2:
//            {
//                _tempCmd.data.f1 = _global.pid2[0];
//                _tempCmd.data.f2 = _global.pid2[1];
//                _tempCmd.data.f3 = _global.pid2[2];
//                _tempCmd.crc = crc8((byte *)&_tempCmd, CMD_LEN - 1);
//                Serial.write((byte *)&_tempCmd, CMD_LEN);
//            }
//            break;
            case CMD_ENABLE_MOTOR:
            {
                int motor = (int)_tempCmd.data.f1;
                int flag = (int)_tempCmd.data.f2;
                switch (motor)
                {
                case 0:
                    m_motorEnabled[MOTOR_NAME::MOTOR1] = flag != 0;
                    break;
                case 1:
//                    m_motorEnabled[MOTOR_NAME::MOTOR2] = flag != 0;
                    break;
                case 2:
                    m_motorEnabled[MOTOR_NAME::MOTOR1] = flag != 0;
//                    m_motorEnabled[MOTOR_NAME::MOTOR2] = flag != 0;
                    break;
                }
            }
            break;
            case CMD_GET_ENABLE_MOTOR:
            {
                _tempCmd.data.f2 = m_motorEnabled[MOTOR_NAME::MOTOR1] ? BYTE_MAX : 0;
//                _tempCmd.data.f3 = m_motorEnabled[MOTOR_NAME::MOTOR2] ? BYTE_MAX : 0;
                _tempCmd.crc = crc8((byte *)&_tempCmd, CMD_LEN - 1);
                Serial.write((byte *)&_tempCmd, CMD_LEN);
            }
            break;
            case CMD_GET_POT_OFFSETS:
            {
                _tempCmd.data.f1 = _global.pot_offset[POT_NAME::POT1];
//                _tempCmd.data.f2 = _global.pot_offset[POT_NAME::POT2];
                _tempCmd.crc = crc8((byte *)&_tempCmd, CMD_LEN - 1);
                Serial.write((byte *)&_tempCmd, CMD_LEN);
            }
            break;
            case CMD_SET_POT_OFFSETS:
            {
                _global.pot_offset[POT_NAME::POT1] = constrain((int)_tempCmd.data.f1, 0, 1023);
//                _global.pot_offset[POT_NAME::POT2] = constrain((int)_tempCmd.data.f2, 0, 1023);
            }
            break;
            case CMD_GET_PWM:
            {
                _tempCmd.data.f1 = _global.pwm_max;
                _tempCmd.crc = crc8((byte *)&_tempCmd, CMD_LEN - 1);
                Serial.write((byte *)&_tempCmd, CMD_LEN);
            }
            break;
            case CMD_SET_PWM:
            {
                _global.pwm_max = constrain((int)_tempCmd.data.f1, 0, BYTE_MAX);
            }
            break;
            case CMD_SET_ENABLE_PID:
            {
                _global.bUsePID = _tempCmd.data.f1 != 0;
            }
            break;
            case CMD_GET_ENABLE_PID:
            {
                _tempCmd.data.f1 = _global.bUsePID ? BYTE_MAX : 0;
                _tempCmd.crc = crc8((byte *)&_tempCmd, CMD_LEN - 1);
                Serial.write((byte *)&_tempCmd, CMD_LEN);
            }
            break;
            case CMD_SETPID_FREQ:
                _global.frequency = _tempCmd.data.f1;
                break;
            case CMD_GETPID_FREQ:
            {
                _tempCmd.data.f1 = _global.frequency;

                _tempCmd.data.f2 = 0;
                _tempCmd.data.f3 = 0;
                _tempCmd.crc = crc8((byte *)&_tempCmd, CMD_LEN - 1);
                Serial.write((byte *)&_tempCmd, CMD_LEN);
            }
            break;
            case CMD_GET_ACC:
            {
                _tempCmd.data.f1 = _global.dAcc;
                _tempCmd.data.f2 = _global.dDec;
                _tempCmd.crc = crc8((byte *)&_tempCmd, CMD_LEN - 1);
                Serial.write((byte *)&_tempCmd, CMD_LEN);
            }
            break;
            case CMD_SET_ACC:
            {
                _global.dAcc = constrain(_tempCmd.data.f1, 0.1, 10);
                _global.dDec = constrain(_tempCmd.data.f2, 0.1, 10);
            }
            break;
            case CMD_GET_PWM_POINT_A:
            {
                _tempCmd.data.f1 = _global.pwm_point1[0];
//                _tempCmd.data.f2 = _global.pwm_point1[1];
                _tempCmd.crc = crc8((byte *)&_tempCmd, CMD_LEN - 1);
                Serial.write((byte *)&_tempCmd, CMD_LEN);
            }
            break;
            case CMD_SET_PWM_POINT_A:
            {
                _global.pwm_point1[0] = (int)constrain(_tempCmd.data.f1, 1, 45);
                _global.pwm_point1[1] = (int)constrain(_tempCmd.data.f2, 0, BYTE_MAX);
            }
            break;
            case CMD_GET_PWM_POINT_B:
            {
                _tempCmd.data.f1 = _global.pwm_point2[0];
                _tempCmd.data.f2 = _global.pwm_point2[1];
                _tempCmd.crc = crc8((byte *)&_tempCmd, CMD_LEN - 1);
                Serial.write((byte *)&_tempCmd, CMD_LEN);
            }
            break;
            case CMD_SET_PWM_POINT_B:
            {
                _global.pwm_point2[0] = (int)constrain(_tempCmd.data.f1, _global.pwm_point1[0] + 1, BYTE_MAX);
                _global.pwm_point2[1] = (int)constrain(_tempCmd.data.f2, 0, BYTE_MAX);
            }
            break;
#ifdef USE_TEMP_SENSOR			
            case CMD_GET_TEMPERATURE:
            {
                _tempCmd.data.f1 = ReadTemperature();
                _tempCmd.data.f2 = 0;
                _tempCmd.crc = crc8((byte *)&_tempCmd, CMD_LEN - 1);
                Serial.write((byte *)&_tempCmd, CMD_LEN);
            }
            break;
#endif
            case CMD_REQUEST_NAME: // Request name
                Serial.println(DEVICE_NAME);
                break;
            default:
                break;
            }
            _wdt = 0;
            return true;
        }
    }
    return false;
}

// Load settings from EEPROM
void LoadSettings()
{
    // Defaults
    _global.m1_limits[0] = -80.0f; // negative degree
//    _global.m2_limits[0] = -80.0f; // negative degree
    _global.m1_limits[1] = 80.0f;  // positive degree
//    _global.m2_limits[1] = 80.0f;  // positive degree

    _global.pot_offset[POT_NAME::POT1] = 512; // Mid point
//    _global.pot_offset[POT_NAME::POT2] = 512; // Mid point

    _global.pwm_max = MAX_PWM;

    _global.bUsePID = false;

    _global.dAcc = _global.dDec = 1.0f;

    _global.pwm_point1[0] = PWM_TOLERANCE_DEG;
    _global.pwm_point1[1] = LOW_PWM;
    _global.pwm_point2[0] = PWM_MAX_ACC_DEG;
    _global.pwm_point2[1] = BYTE_MAX;

    MotorsStruct eeprom_data;
    EEPROM.get(EEPROM_DATA_OFFSET, eeprom_data);
    if (eeprom_data.version >= EEPROM_DATA_VER1)
    {
        _global = eeprom_data;
        if (eeprom_data.version < EEPROM_DATA_VER2)
            _global.pwm_max = MAX_PWM;

        if (eeprom_data.version < EEPROM_DATA_VER3)
        {
            _global.bUsePID = false;
        }

        if (eeprom_data.version < EEPROM_DATA_VER4)
        {
            _global.dAcc = _global.dDec = 1.0f;
            _global.pwm_point1[0] = PWM_TOLERANCE_DEG;
            _global.pwm_point1[1] = LOW_PWM;
            _global.pwm_point2[0] = PWM_MAX_ACC_DEG;
            _global.pwm_point2[1] = BYTE_MAX;
        }
    }
    else
    {
        // Default values
        _global.version = EEPROM_DATA_VER;
        _global.m1_limits[0] = -80.0f; // negative degree
//        _global.m2_limits[0] = -80.0f; // negative degree
        _global.m1_limits[1] = 80.0f;  // positive degree
//        _global.m2_limits[1] = 80.0f;  // positive degree
        _global.pot_offset[POT_NAME::POT1] = 512;             // Mid point
//        _global.pot_offset[POT_NAME::POT2] = 512;             // Mid point

        _global.frequency = PID_FREQUENCY;
        _global.pid1[0] = 0.05f;
//        _global.pid2[0] = 0.05f;
        _global.pid1[1] = 0.05f;
//        _global.pid2[1] = 0.05f;
        _global.pid1[2] = 0.03f;
//        _global.pid2[2] = 0.03f;
        _global.bUsePID = false;

        _global.pwm_max = MAX_PWM;

        _global.dAcc = _global.dDec = 1.0f;

        _global.pwm_point1[0] = PWM_TOLERANCE_DEG;
        _global.pwm_point1[1] = LOW_PWM;
        _global.pwm_point2[0] = PWM_MAX_ACC_DEG;
        _global.pwm_point2[1] = BYTE_MAX;
    }
}

// Store settings to EEPROM
void StorePidData()
{
    MotorsStruct eeprom_data;
    EEPROM.get(EEPROM_DATA_OFFSET, eeprom_data);
    if (memcmp(&eeprom_data, &_global, sizeof(MotorsStruct)) != 0)
    {
        _global.version = EEPROM_DATA_VER;
        EEPROM.put(0, _global);
    }
}

// Relay switching function
inline void SwitchRelay(bool bEnable)
{
    digitalWrite(RELAY_PIN, bEnable ? HIGH : LOW);
    digitalWrite(LED_BUILTIN, bEnable ? HIGH : LOW);
}

// Returns current angle in degrees for potentiometer "pot"
inline float GetAngle(POT_NAME pot, float full_angle)
{
    int sensor_value = 0;
    switch (pot)
    {
    case POT1:
        sensor_value = analogRead(ROT_POT1_PIN) - _global.pot_offset[POT_NAME::POT1];
        break;
//    case POT2:
//        sensor_value = analogRead(ROT_POT2_PIN) - _global.pot_offset[POT_NAME::POT2];
//        break;
//    case POT_EXT:
//        sensor_value = analogRead(ROT_POTEXT_PIN) - 512;
//        break;
    }
    return (float)sensor_value * full_angle / 1023.0f;
}

// Returns current angle in degrees for potentiometer at "potPin"
inline float GetAngle(int potPin, float full_angle, float offset)
{
//    int sensor_value = analogRead(ROT_POT1_PIN) - offset; // Fehler?
    int sensor_value = analogRead(potPin) - offset;

    return (float)sensor_value * full_angle / 1023.0f;
}

// Don't stop motor, just very slow move
#define MIN_LOWER_PWM 2

// Move motor in defined direction and speed
void Move(MOTOR_NAME mot, MOTOR_DIRECTION dir, int pwm)
{
    uint8_t pwmr_pin;
    uint8_t pwml_pin;
    uint8_t ENr_pin;
    uint8_t ENl_pin;

    // rescale pwm
    pwm = map(abs(pwm), 0, BYTE_MAX, MIN_LOWER_PWM, _global.pwm_max);

    switch (mot)
    {
    case MOTOR_NAME::MOTOR1:
//        pwm_pin = M1_PWM_PIN;
//        dir_pin = M1_DIR_PIN;
        pwmr_pin = M1_PWMr_PIN;
        pwml_pin = M1_PWMl_PIN;
        ENr_pin = M1_ENr_PIN;
        ENl_pin = M1_ENl_PIN;
        break;
//    case MOTOR_NAME::MOTOR2:
//        pwm_pin = M2_PWM_PIN;
//        dir_pin = M2_DIR_PIN;
//        break;
    default:
        return;
    }

    switch (dir)
    {
    default:
    case MOTOR_DIRECTION::STOP:
      motorController1.Stop();
      motorController1.Disable();
        break;
    case MOTOR_DIRECTION::CW:
      motorController1.Enable();
      motorController1.TurnRight(pwm);
        break;
    case MOTOR_DIRECTION::CCW:
      motorController1.Enable();
      motorController1.TurnLeft(pwm);
        break;
    }
}

// Setup PWM clock divider
void setPwmFrequency(int pin, int divisor)
{
    byte mode;
    if (pin == 5 || pin == 6 || pin == 9 || pin == 10)
    {
        switch (divisor)
        {
        case 1:
            mode = 0x01;
            break; // 62.5 kHz
        case 8:
            mode = 0x02;
            break; // 7.8 kHz
        case 64:
            mode = 0x03;
            break; // 976 Hz
        case 256:
            mode = 0x04;
            break; // 244 Hz
        case 1024:
            mode = 0x05;
            break; // 61 Hz
        default:
            return;
        }
        if (pin == 5 || pin == 6)
        {
            TCCR0B = TCCR0B & 0b11111000 | mode;
        }
        else
        {
            TCCR1B = TCCR1B & 0b11111000 | mode;
        }
    }
    else if (pin == 3 || pin == 11)
    {
        switch (divisor)
        {
        case 1:
            mode = 0x01;
            break; // 31.25 kHz
        case 8:
            mode = 0x02;
            break; // 3.9 kHz
        case 32:
            mode = 0x03;
            break; // 976 Hz
        case 64:
            mode = 0x04;
            break; // 488 Hz
        case 128:
            mode = 0x05;
            break; // 244 Hz
        case 256:
            mode = 0x06;
            break; // 122 Hz
        case 1024:
            mode = 0x07;
            break; // 30 Hz
        default:
            return;
        }
        TCCR2B = TCCR2B & 0b11111000 | mode;
    }
}

// Emergency Stop
void EmergencyStop()
{
    _device_state = DEVICE_STATE::EMERGENCY_STOP;
    Move(MOTOR_NAME::MOTOR1, MOTOR_DIRECTION::STOP, 0);
//    Move(MOTOR_NAME::MOTOR2, MOTOR_DIRECTION::STOP, 0);
    // Turn power Off
    SwitchRelay(false);
}

void ShutDown()
{
    _device_state = DEVICE_STATE::SHUTDOWN;
    Move(MOTOR_NAME::MOTOR1, MOTOR_DIRECTION::STOP, 0);
//    Move(MOTOR_NAME::MOTOR2, MOTOR_DIRECTION::STOP, 0);
    // Turn power Off
    SwitchRelay(false);
}

float mapf(float x, float in_min, float in_max, float out_min, float out_max)
{
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

#ifdef USE_TEMP_SENSOR
#define TS_DELAY 1000
size_t ts_ticks = TS_DELAY;
float ts_temperature = 25.0f;

float ReadTemperature()
{
    if ((ts_ticks % TS_DELAY) == 0)
    {
        ts.reset();
        ts.write(0xCC);
        ts.write(0x44);

        ts.reset();
        ts.write(0xCC);
        ts.write(0xBE);

        byte b0 = ts.read();
        byte b1 = ts.read();

        ts_ticks = 0;
        return ts_temperature = ((b1 << 8) | b0) * 0.0625;
    }

    ts_ticks++;
    return ts_temperature;
}
#else
float ReadTemperature()
{
    return 0;
}
#endif
