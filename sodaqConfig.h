#define DEVICE_URN      "urn:dev:IMEI:FILL-ME:"
#define DEVICE_KEY      "FILL-ME"

#define PINNUMBER       "" // Normally not needed

#define APN             "kpnthings.iot"
#define APN_LOGIN       "" // Normally not needed
#define APN_PASSWORD    "" // Normally not needed

#define HTTP_HOST       "m.m"
#define HTTP_IP         "10.151.236.157"
#define HTTP_PATH       "/ingestion/m2m/senml/v1"
#define HTTP_PORT       80

#define CONSOLE_STREAM   SerialUSB
#define MODEM_STREAM     Serial1

#define CURRENT_OPERATOR AUTOMATIC_OPERATOR
#define CURRENT_URAT     SODAQ_R4X_LTEM_URAT
#define CURRENT_MNO_PROFILE MNOProfiles::STANDARD_EUROPE

#define NBIOT_BANDMASK BAND_MASK_UNCHANGED

static Sodaq_R4X r4x;
static Sodaq_SARA_R4XX_OnOff saraR4xxOnOff;
static bool isReady;
static bool isOff;
