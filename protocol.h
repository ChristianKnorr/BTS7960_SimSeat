// Andrey Zhuravlev
// email: v.azhure@gmail.com

#ifndef Protocol_h
#define Protocol_h

#define SIZEOF_FLOAT 4

// Command packet head byte value
#define PACKET_HEAD_BYTE 0xAA
#define CMD_HEADER PACKET_HEAD_BYTE
// Command: Set X
#define CMD_SET_X 0x01
// Command: Set Y
#define CMD_SET_Y 0x02
// Command: Set X and Y
#define CMD_SET_XY 0x03
// Command: Set X and Y with reply current position
#define CMD_SET_XY_REPLY 0x04
// Command: Set PWM 1
#define CMD_SET_PWM1 0x05
// Command: Set PWM 2
#define CMD_SET_PWM2 0x06
// Command: Get PWM 1
#define CMD_GET_PWM1 0x07
// Command: Get PWM 2
#define CMD_GET_PWM2 0x08
// Command: Enable Motor
#define CMD_ENABLE_MOTOR 0x09
// Command: Setup M1 PID
#define CMD_SETPID1 0x0A
// Command: Setup M2 PID
#define CMD_SETPID2 0x0B
// Command: Setup M1 PID
#define CMD_GETPID1 0x0C
// Command: Setup M2 PID
#define CMD_GETPID2 0x0D
// Command: STORE PID to EEPROM
#define CMD_SET_STOREPID 0x0E
// Command:
#define CMD_RESERVED_1 0x0F
// Command: Get Motor State
#define CMD_GET_ENABLE_MOTOR 0x10
// Command: Set PID frequency
#define CMD_SETPID_FREQ 0x11
// Command: Get PID frequency
#define CMD_GETPID_FREQ 0x12
// Command: Get Potentiometers offsets
#define CMD_GET_POT_OFFSETS 0x13
// Command: Set Potentiometers offsets
#define CMD_SET_POT_OFFSETS 0x14
// Command: Get PWM
#define CMD_GET_PWM 0x15
// Command: Set PWM
#define CMD_SET_PWM 0x16
// Command: Get PID state
#define CMD_SET_ENABLE_PID 0x17
// Command: Set PID state
#define CMD_GET_ENABLE_PID 0x18
// Command: Manual EMERGENCY move motor  
#define CMD_EMERGENCY_MOVE 0x19
// Command: Get Acceleration / Deceleration delta
#define CMD_GET_ACC 0x1A
// Command: Set Acceleration / Deceleration delta
#define CMD_SET_ACC 0x1B
// Command: Get PWM control Point A
#define CMD_GET_PWM_POINT_A 0x1C
// Command: Set PWM control Point A
#define CMD_SET_PWM_POINT_A 0x1D
// Command: Get PWM control Point B
#define CMD_GET_PWM_POINT_B 0x1E
// Command: Set PWM control Point B
#define CMD_SET_PWM_POINT_B 0x1F
#define CMD_GET_TEMPERATURE 0x20
// Start motion
#define CMD_START_UP 0xFA
// End motion
#define CMD_SHUT_DOWN 0xFE
// Command: Request device name
#define CMD_REQUEST_NAME 0xff

typedef struct
{
    float f1;
    float f2;
    float f3;
} CMD_DATA;

typedef struct
{
    byte header;    // start byte (0xAA)
    short datasize; // cmd length
    byte cmd;       // cmd id
    CMD_DATA data;  // cmd data
    byte crc;       // control summ
} CMD;

// CRC table
static const uint8_t PROGMEM dscrc_table[] = {
    0, 94, 188, 226, 97, 63, 221, 131, 194, 156, 126, 32, 163, 253, 31, 65,
    157, 195, 33, 127, 252, 162, 64, 30, 95, 1, 227, 189, 62, 96, 130, 220,
    35, 125, 159, 193, 66, 28, 254, 160, 225, 191, 93, 3, 128, 222, 60, 98,
    190, 224, 2, 92, 223, 129, 99, 61, 124, 34, 192, 158, 29, 67, 161, 255,
    70, 24, 250, 164, 39, 121, 155, 197, 132, 218, 56, 102, 229, 187, 89, 7,
    219, 133, 103, 57, 186, 228, 6, 88, 25, 71, 165, 251, 120, 38, 196, 154,
    101, 59, 217, 135, 4, 90, 184, 230, 167, 249, 27, 69, 198, 152, 122, 36,
    248, 166, 68, 26, 153, 199, 37, 123, 58, 100, 134, 216, 91, 5, 231, 185,
    140, 210, 48, 110, 237, 179, 81, 15, 78, 16, 242, 172, 47, 113, 147, 205,
    17, 79, 173, 243, 112, 46, 204, 146, 211, 141, 111, 49, 178, 236, 14, 80,
    175, 241, 19, 77, 206, 144, 114, 44, 109, 51, 209, 143, 12, 82, 176, 238,
    50, 108, 142, 208, 83, 13, 239, 177, 240, 174, 76, 18, 145, 207, 45, 115,
    202, 148, 118, 40, 171, 245, 23, 73, 8, 86, 180, 234, 105, 55, 213, 139,
    87, 9, 235, 181, 54, 104, 138, 212, 149, 203, 41, 119, 244, 170, 72, 22,
    233, 183, 85, 11, 136, 214, 52, 106, 43, 117, 151, 201, 74, 20, 246, 168,
    116, 42, 200, 150, 21, 75, 169, 247, 182, 232, 10, 84, 215, 137, 107, 53};

// CRC main function
uint8_t crc8(const uint8_t *addr, uint8_t len)
{
    uint8_t crc = 0;
    while (len--)
    {
        crc = pgm_read_byte(dscrc_table + (crc ^ *addr++));
    }
    return crc;
}

// Command length in bytes
#define CMD_LEN sizeof(CMD)
// CMD data length in bytes
#define CMD_DATA_LEN sizeof(CMD_DATA)

class ExpFilter
{
  private:
    bool m_bInitialized;
    float m_fpreviousAverage;

  public:
    inline void Reset() { m_bInitialized = false; }
    ExpFilter() {Reset();}
    float Smooth(float val, float coeff)
    {
        float mul = constrain(1.0 - coeff, 0, 1);

        if (!m_bInitialized)
        {
            m_bInitialized = true;
            return m_fpreviousAverage = val;
        }

        return m_fpreviousAverage = ((val - m_fpreviousAverage) * mul) + m_fpreviousAverage;
    }
};

#endif
