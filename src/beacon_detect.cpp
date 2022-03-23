//
// Low latency BLE arrival detection and reporting
//

#include <stdlib.h>
#include <stdarg.h>
#include <errno.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <sys/times.h>
#include <fcntl.h>
#include <sys/select.h>
#include <signal.h>
#include <string>
#include <list>
#include <chrono>

#include <bluetooth/bluetooth.h>
#include <bluetooth/hci.h>
#include <bluetooth/hci_lib.h>

typedef long long int64;

extern "C" {
    #include <MQTTClient.h>
    #include <MQTTClientPersistence.h>
}


// Configuration
// ================================================================================================================

#define DEVICEFILE  "/home/pi/bdetect/known_devices.txt"
#define LOGFILE     "/home/pi/bdetect/log.txt"

#define ADDRESS     "tcp://192.168.5.100:1883"
#define CLIENTID    "BLE Beacon scanner"

#define MIN_DETECTION_RSSI  -101   // in dBm


// Logging
// ================================================================================================================

#define KNRM "\x1B[0m"
#define KRED "\x1B[31;1m"
#define KGRN "\x1B[32;1m"
#define KYEL "\x1B[33;1m"
#define KBLU "\x1B[34;1m"
#define KMAG "\x1B[35;1m"
#define KCYA "\x1B[36;1m"
#define KWHT "\x1B[37;1m"

void log(const std::string &module, const std::string &color, const char *format, ...)
{
    static char Buffer[500];

    va_list args;
    va_start(args, format);
    vsprintf(Buffer, format, args);
    va_end(args);

    auto now = std::chrono::system_clock::now();
    auto now_c = std::chrono::system_clock::to_time_t(now);
    std::tm *t = std::localtime(&now_c);

    static char Buffer2[500];
    sprintf(Buffer2, "%4d%02d%02d %02d:%02d:%02d", t->tm_year + 1900, t->tm_mon + 1, t->tm_mday, t->tm_hour, t->tm_min, t->tm_sec);

    printf("%s [%s%s" KNRM "] %s\n", Buffer2, color.c_str(), module.c_str(), Buffer);

    FILE *fp = fopen(LOGFILE, "a");
    fprintf(fp, "%s [%s] %s\n", Buffer2, module.c_str(), Buffer);
    fclose(fp);
}


// Extract the beacon GUID from manufacturer payload
// ================================================================================================================

static std::string ParseGUID(uint8_t *Data, int len)
{
    char Buffer[40];
    
    // Apple iBeacon
    if( Data[0] == 0x4c && Data[1] == 0x00 && Data[2] == 0x02 ) {
        sprintf(Buffer, "%02X%02X%02X%02X-%02X%02X-%02X%02X-%02X%02X-%02X%02X-%02X%02X%02X%02X:%02X%02X-%02X%02X", Data[4], Data[5], Data[6], Data[7], Data[8], Data[9], Data[10], Data[11], Data[12], Data[13], Data[14], Data[15], Data[16], Data[17], Data[18], Data[19], Data[20], Data[21], Data[22], Data[23]); 
        return( Buffer );
    }
    
    // Microsoft BLE beacon
    if( Data[0] == 0x06 && Data[1] == 0x00 && Data[2] == 0x01 ) {
        sprintf(Buffer, "%02X/%02X%02X%02X%02X%02X%02X%02X%02X%02X%02X%02X%02X%02X%02X%02X%02X", Data[3], Data[10], Data[11], Data[12], Data[13], Data[14], Data[15], Data[16], Data[17], Data[18], Data[19], Data[20], Data[21], Data[22], Data[23], Data[24], Data[25]);
        return( Buffer );
    }
    
    return( "" );
}

static int ParseRSSI(uint8_t *Data, int len)
{
    // Apple iBeacon
    if( Data[0] == 0x4c && Data[1] == 0x00 && Data[2] == 0x02 ) {
        return( Data[24] );
    }
    
    return( 0 );
}


// Timer
// ================================================================================================================

static int64 GetTickTimer(void)
{
    static int64 clocksPerSec = 0;
    static struct tms tmdummy;

    // Get number of clock ticks per second
    if( !clocksPerSec ) clocksPerSec = (int64)sysconf(_SC_CLK_TCK);

    // Return system time in msec
    return( (int64)(times(&tmdummy) * 1000LL / clocksPerSec) );
}


// Bluetooth HCI low level
// ================================================================================================================

class CHCIDevice {
    
private:
    int             p_device;
    
    void            p_ParseEIR(uint8_t *eir, int eir_len, std::string &guid);

public:
    CHCIDevice();
    ~CHCIDevice();
    
    bool            IsDeviceValid() const { return( p_device >= 0 ); }
    
    void            SetScanParameters(bool active, bool whitelist);
    
    void            ScanEnabled(bool state);
    
    int             GetNextScannedDevice(bdaddr_t &bdaddr, std::string &guid, int &rssi);
    
    std::string     IdentifyDevice(bdaddr_t &bdaddr);

};

CHCIDevice::CHCIDevice()
{
    int devid = hci_get_route(NULL);
    
    p_device = hci_open_dev(devid);
    
    fcntl(p_device, F_SETFL, fcntl(p_device, F_GETFL, 0) | O_NONBLOCK);
    
    hci_filter nf;
    hci_filter_clear(&nf);
    hci_filter_set_ptype(HCI_EVENT_PKT, &nf);
    hci_filter_set_event(EVT_LE_META_EVENT, &nf);

    if( setsockopt(p_device, SOL_HCI, HCI_FILTER, &nf, sizeof(nf)) < 0 ) {
        log("BTLE", KRED, "Could not set socket options");
        p_device = -1;
    }
}

CHCIDevice::~CHCIDevice()
{
    if( p_device >= 0 ) {
        hci_close_dev(p_device);
    }
}

void CHCIDevice::SetScanParameters(bool active, bool whitelist)
{
    int error = hci_le_set_scan_parameters(p_device, active ? 1 : 0, htobs(0x0010), htobs(0x0010), 0, whitelist ? 1 : 0, 1000);
    
    if( error < 0 ) {
        log("BTLE", KRED, "hci_le_set_scan_parameters failed %d", error);
    }
}

void CHCIDevice::ScanEnabled(bool state)
{
    int error = hci_le_set_scan_enable(p_device, state ? 0x01 : 0, 0, 1000);
    
    if( error < 0 ) {
        log("BTLE", KRED, "hci_le_set_scan_enable failed %d", error);
    }
}

void CHCIDevice::p_ParseEIR(uint8_t *eir, int eir_len, std::string &guid)
{
    int offset = 0;

    while( offset < eir_len ) {
        
        int field_len = eir[0];

        // Check for the end of EIR 
        if( !field_len ) break;

        if( offset + field_len > eir_len ) break;

        switch( eir[1] ) {
            
            case 8:
            case 9:
                /*int name_len = field_len - 1;
                if( name_len <= 50 ) {
                    char buf[50] = { 0 };
                    memcpy(buf, &eir[2], name_len);
                    name = buf;
                }*/
                break;
                
            case 0xff:
            {
                guid = ParseGUID(&eir[2], field_len);
                break;
            }
            
        }

        offset += field_len + 1;
        eir += field_len + 1;
    }
}

int CHCIDevice::GetNextScannedDevice(bdaddr_t &bdaddr, std::string &guid, int &rssi)
{
    // Return value: 0 = ok, data available
    //               1 = no data available yet
    //               2 = error
    
    fd_set set;
    FD_ZERO(&set);
    FD_SET(p_device, &set);
    
    timeval timeout;
    timeout.tv_sec = 2;
    timeout.tv_usec = 0;

    select(p_device + 1, &set, NULL, NULL, &timeout);
    
    unsigned char buf[HCI_MAX_EVENT_SIZE];
    
    int len = read(p_device, buf, sizeof(buf));

    if( len < 0 )
        return( ( errno == EAGAIN || errno == EINTR ) ? 1 : 2 );

    unsigned char *ptr = buf + (1 + HCI_EVENT_HDR_SIZE);
    len -= (1 + HCI_EVENT_HDR_SIZE);

    evt_le_meta_event *meta = (evt_le_meta_event *)ptr;
    
    if( meta->subevent != 0x02 ) return( 2 );

    le_advertising_info *info = (le_advertising_info *)(meta->data + 1);

    p_ParseEIR(info->data, info->length, guid);
    
    bdaddr = info->bdaddr;

    rssi = (int)((char)info->data[info->length]) - 256;

    //log("BTLE", KGRN, "rssi = -%ddBm", rssi);

    return( 0 );
}


// MQTT client
// ================================================================================================================

class CMQTT {
    
private:
    MQTTClient      p_client;
    
    const int       p_timeoutConnect = 2;
    const int       p_timeoutDelivery = 5;

public:
    CMQTT();
    ~CMQTT();
    
    bool            Connect();
    void            Disconnect();

    void            Yield();
    
    void            Publish(const std::string &name, const std::string &state);
    
};

CMQTT::CMQTT()
{
    MQTTClient_create(&p_client, ADDRESS, CLIENTID, MQTTCLIENT_PERSISTENCE_NONE, NULL);
}

CMQTT::~CMQTT()
{
    MQTTClient_destroy(&p_client);
}

bool CMQTT::Connect()
{
    log("MQTT", KCYA, "Connecting with broker at %s...", ADDRESS);
    
    MQTTClient_connectOptions conn_opts = MQTTClient_connectOptions_initializer;
    conn_opts.keepAliveInterval = 60;
    conn_opts.cleansession = 0;
    conn_opts.connectTimeout = p_timeoutConnect;
    
    int rc = MQTTClient_connect(p_client, &conn_opts);
    
    if( rc != MQTTCLIENT_SUCCESS ) {
        log("MQTT", KRED, "Failed to connect, return code %d", rc);
        return( false );
    }
    
    log("MQTT", KCYA, "OK Connected !");
    
    return( true );
}

void CMQTT::Disconnect()
{
    MQTTClient_disconnect(p_client, p_timeoutDelivery * 1000);
}

void CMQTT::Yield()
{
    MQTTClient_yield();

    if( !MQTTClient_isConnected(p_client) ) {
    log("MQTT", KRED, "MQTT broker disconnected after yield, attempting to reestablish connection");
    Connect();
    }
}

void CMQTT::Publish(const std::string &name, const std::string &state)
{
    if( !MQTTClient_isConnected(p_client) ) {
        log("MQTT", KRED, "MQTT broker disconnected, attempting to reestablish connection");
        Connect();
    }
    
    if( MQTTClient_isConnected(p_client) ) {
    
        std::string topic = "blescanner/" + name;
        std::string payload = "{ \"state\" : \"" + state + "\" }";
    
        MQTTClient_message pubmsg = MQTTClient_message_initializer;
        pubmsg.payload = (void *)payload.c_str();
        pubmsg.payloadlen = payload.length();
        pubmsg.qos = 1;
        pubmsg.retained = 0;
        
        MQTTClient_deliveryToken token;
        
        int rc = MQTTClient_publishMessage(p_client, topic.c_str(), &pubmsg, &token);
        
        if( rc == MQTTCLIENT_SUCCESS ) {
           
            rc = MQTTClient_waitForCompletion(p_client, token, p_timeoutDelivery * 1000);
        
            if( rc == MQTTCLIENT_SUCCESS ) {
                log("MQTT", KCYA, "Message with delivery token %d delivered", token);   
            } else
                log("MQTT", KRED, "Message could not be delivered (%d)", rc);

        } else
        
            log("MQTT", KRED, "Message publish failed with code %d", rc);
    
    }
}


// Active BLE device
// ================================================================================================================

struct SActiveDevice {
    
    std::string     guid;
    bdaddr_t        bdaddr;
    std::string     knownName;
    int             type;           // 0 = empty, 1 = unknown, 2 = known
    int64           lastseen;
    int             rssiMax;
    
    SActiveDevice() : type(0), lastseen(0) { }

};


// MQTT actions
// ================================================================================================================

void MessageNewDevice(CMQTT &MQTT, SActiveDevice &Device, int rssi)
{
    char addr[18];
    ba2str(&Device.bdaddr, addr);
    
    if( Device.type == 2 ) {
        log("BTLE", KBLU, "New known device %s (%ddBm) - %s", addr, rssi, Device.knownName.c_str());
        MQTT.Publish(Device.knownName, "on");
    } else
        log("BTLE", KBLU, "New device %s (%ddBm) - [%s]", addr, rssi, Device.guid.c_str());
}

void MessageLostDevice(CMQTT &MQTT, SActiveDevice &Device)
{
    char addr[18];
    ba2str(&Device.bdaddr, addr);
    
    if( Device.type == 2 ) {
        log("BTLE", KBLU, "Lost known device %s (%ddBm max) - %s", addr, Device.rssiMax, Device.knownName.c_str());
        MQTT.Publish(Device.knownName, "off");
    } else
        log("BTLE", KBLU, "Lost device %s (%ddBm max) - %s", addr, Device.rssiMax,  Device.guid.c_str());
}


// Device list management
// ================================================================================================================

static std::list<SActiveDevice> ActiveDevices;

const int lostTimeoutKnown   = 4 * 1000;    // in msec
const int lostTimeoutUnknown = 30 * 1000;   // in msec

SActiveDevice *FindActiveDevice(const bdaddr_t &bdaddr)
{
    for( auto &i : ActiveDevices ) {
        if( i.type > 0 ) {
            if( !bacmp(&bdaddr, &i.bdaddr) ) return( &i );
        }
    }
    
    return( nullptr );
}

void CollectLostDevices(CMQTT &MQTT, int64 timeNow)
{
    for( auto i = ActiveDevices.begin(); i != ActiveDevices.end(); ++i ) {
        if( i->type > 0 ) {
            int dt = (int)(timeNow - i->lastseen);
            if( ( i->type == 2 && dt > lostTimeoutKnown ) || ( i->type == 1 && dt > lostTimeoutUnknown ) ) {
                MessageLostDevice(MQTT, *i);
                ActiveDevices.erase(i);
                break;
            }
        }
    }
}


// Known devices configuration file
// ================================================================================================================

std::string SearchKnownDevices(const std::string &guid)
{
    std::string result;
    
    FILE *fp = fopen(DEVICEFILE, "r");
    
    if( fp ) {
    
        char Buffer[200];
        
        while( !feof(fp) && result.empty() ) {
            fgets(Buffer, 200, fp);
            char *token = strtok(Buffer, " \t\n\r");
            std::string name = token ? token : "";
            if( !name.empty() ) {
                token = strtok(nullptr, " \t\n\r");
                if( token && std::string(token) == guid ) result = name;
            }
        }
        
        fclose(fp);
    
    }
    
    return( result );
}


// Main
// ================================================================================================================

const int64 yieldTime = 10 * 1000;

static bool running = true;

void SignalCallback(int)
{
    running = false;
}

int main()
{
    signal(SIGINT, SignalCallback);
    
    log("STRT", KYEL, "------ BLE Detect started ! ------");
    
    CMQTT MQTT;
    MQTT.Connect();
    
    static int64 lastYieldTime = GetTickTimer();
    
    CHCIDevice HCIDevice;
    HCIDevice.SetScanParameters(false, false);
    HCIDevice.ScanEnabled(true);
    
    while( running ) {
        
        bdaddr_t bdaddr;    
        std::string guid;
        int rssi;
        
        int result = HCIDevice.GetNextScannedDevice(bdaddr, guid, rssi);
        
        int64 timeNow = GetTickTimer();
        
        if( result == 2 ) break;
        
        if( result == 0 && rssi >= MIN_DETECTION_RSSI ) {
            
            SActiveDevice *ActiveDevice = FindActiveDevice(bdaddr);
            
            if( !ActiveDevice ) {
                
                SActiveDevice newDevice;
                newDevice.bdaddr = bdaddr;
                newDevice.guid = guid;
                newDevice.knownName = SearchKnownDevices(guid);
                newDevice.type = newDevice.knownName.empty() ? 1 : 2;
                newDevice.lastseen = timeNow;
                newDevice.rssiMax = rssi;
                ActiveDevices.push_back(newDevice);
                
                MessageNewDevice(MQTT, newDevice, rssi);
                
            } else {
                
                ActiveDevice->lastseen = timeNow;
                if( rssi > ActiveDevice->rssiMax ) ActiveDevice->rssiMax = rssi;
                
            }
            
        }
        
        CollectLostDevices(MQTT, timeNow);

        if( timeNow - lastYieldTime >= yieldTime || timeNow < lastYieldTime ) {
            lastYieldTime = timeNow;
            MQTT.Yield();
        }
        
    }
    
    log("EXIT", KYEL, "SIGINT received, shutting down");
    
    HCIDevice.ScanEnabled(false);
    
    MQTT.Disconnect();
    
    return 0;
}


