#define NOMINAL 0
#define ERROR 1
#define WARNING 2
#define DEBUG 3
#define CRITICAL 4
#define DATA 5
#define ACTION 6
#define PING 7
#define STARTUP 8
#define CHECKSUM 9
#define QUERY 10
#define STATUS 11
#define INFO 12

#define SOURCE_FC 0 // Flight computer
#define SOURCE_SC 1 // Safety computer
#define SOURCE_LC 2 // Launch computer

#define CODE_HW_BME 0
#define CODE_HW_MPU 1
#define CODE_HW_SD 2
#define CODE_HW_GEN 3
#define CODE_BATTERY 4
#define CODE_COMMS 5
#define CODE_BATTERY 6

#define CODE_FATAL 0
#define CODE_RECOVERABLE 1

#define CODE_EJECT_PARACHUTE 0

#define DATA_BME 0
#define DATA_MPU 1
#define DATA_BME_EXT 2
#define DATA_MPU_EXT 3

#define STATUS_LAUNCH 0
#define STATUS_READY 1
#define STATUS_UNREADY 2
#define STATUS_ABORT 3
#define STATUS_TEST 4
#define STATUS_COMMS 5

#define QUERY_LAUNCH 0
#define QUERY_INFO 1
