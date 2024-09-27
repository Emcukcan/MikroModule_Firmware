//25----30 SD Card Open
//31----35 CANBUS  Open
//45----48 SD Card Close
//50----55 Firebase Open











bool AlarmLogOperation = false;
bool FooterTouch = false;
int RecoveryCounter = 0;

bool SDReadVolt = true;
bool SDReadTemp = true;
bool SDReadCharge = true;
bool SDReadDischarge = true;
bool deleteFilesProgress = false;
bool AlarmReadTrigger = false;

bool pulled = false;
bool pulled2 = false;



int FirebaseCounter = 0;

bool display1 = true;
bool update1 = true;


double NormVoltageArray[24];



bool SimulationStatus = false;
bool AnimationBlink = false;
int NominalVoltage = 48;
int RemainingTime = 0;
int ReadCellMod = 16;



int maxValV = -99999;
int minValV = 999999;

int SetCharge = 0;
int SetDischarge = 0;
int METHOD = 0;
int TYPE = 0;
bool SetEnergies = false;
bool SetCOMM = false;
int BMSAlarmArray[10][23];



int Security = 0;
int boardSetted = true;
bool Trigger = false;
bool SetCANID = false;

//#include "bms_temp.h"
#include <math.h>
#include <IOXhop_FirebaseESP32.h>
#include "nvs_flash.h"
#include <ArduinoJson.h>
#include <TFT_eSPI.h> // Hardware-specific library
#include <SPI.h>
#include <HTTPClient.h>
#include <Arduino_JSON.h>
#include "time.h"
#include <DNSServer.h>
#include <Wifi.h>
#include <WiFiManager.h> // https://github.com/tzapu/WiFiManager
#include <WiFiUdp.h>
#include <Arduino.h>
#include "AsyncUDP.h"


#include "BluetoothSerial.h"
#include <WiFi.h>
#include <HTTPClient.h>
#include <HTTPUpdate.h>
#include <WiFiClientSecure.h>
#include "cert.h"
#include "soc/rtc_wdt.h"
#include <esp_task_wdt.h>

float SIM_SOC ;
float SIM_MAXCELL ;
float SIM_MIN_CELL ;
float SIM_SUM ;
float SIM_CURRENT;
float SIM_TEMP ;


int AlarmLogStep = 0;
int LastPageNumber = 0;
bool readSerial = false;
bool readCANID = false;
bool coordinatesTaken = false;
bool landingDone = false;
float lat = 0;
float lon = 0;
bool SafetyAlarm = false;
int FooterEvent = 0;
float CalibrationValue = 0;
int CalibrationStatus = 0;

char asciiArrayVoltage[4][24];
char asciiArrayTemperature[4][24];
char asciiArrayCharge[4][24];
char asciiArrayDischarge[4][24];


String StringVoltageArray[24];
String StringTemperatureArray[24];
String StringHourlyDischargeArray[24];
String StringHourlyChargeArray[24];
int x_index = 0;
int y_index = 0;


#define DARKER_GREY 0x18E3

bool updateStarted = false;

//osman libraries
#include <gpio.h>
#include <TCA9548A.h>
#include <rtc_data.h>
#include <mcp2515_can.h>
#include <controlsd.h>
#include <bms_spi.h>
#include <bmsrs485.h>
#include <SC16S740.h>
#include <bms_temp.h>
#include <canbus.h>
//QR code
#include "qrcode.h"
QRCode qrcode;
bool ShowBarcode = false;
/////////

//geolocation

double ConvertedOffset;
double Latitude;
double Longitude;
String CountryName;
bool GPSFetched = false;


//gpio read
uint8_t GPIO_READ[3] = {0, 0, 0};

double VoltageArray[24];
String VoltageArrayString[24];

double TemperatureArray[24];
String TemperatureArrayString[24];

double HourlyDischargeArray[24];
String HourlyDischargeArrayString[24];

double HourlyChargeArray[24];
String HourlyChargeArrayString[24];
String BMSAlarmArrayString[10][10];

String updateResponse = "Uploading";
String calibratedHour = "";
String calibratedMinute = "";
String calibratedSecond = "";
String AlarmStringLog = "No Alarm";
String AlarmStringLogPro = "";
String AlarmStringLogTime = "";
uint8_t bmsCounter = 0;
String TimeArray[24];
uint8_t board1[3] = {0, 0, 0};
bool buzzerActive = false;
bool control_sd = false;
const int SD_CS_PIN = 12;

String SDCard_row[26] = {"TimeStamp", "TerminalV", "TerminalC", "CellT", "MaxV", "MinV", "MaxDiff", "SOC" "ChargeE", "DischargeE",
                         "Cell#1", "Cell#2", "Cell#3", "Cell#4", "Cell#5", "Cell#6", "Cell#7", "Cell#8", "Cell#9", "Cell#10", "Cell#11", "Cell#12", "Cell#13", "Cell#14", "Cell#15", "Cell#16"

                        };
//canbus
uint8_t alarmsr[14] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}; // 13 alarms

String UDPRequest = "";

//DRY CONTACT LIST
bool ContactNameActive = false;
bool ContactTypeActive = false;
bool ContactEnableActive = false;
bool ContactDisableActive = false;
bool OperationActive = false;

int ContactNameIndex = 0;
int ContactTypeIndex = 0;
int OperationTypeIndex = 0;

bool pinged = false;
bool SafetyVisible = false;

int DRY_VALUES[4] = {0, 0, 0, 0};



String DRYA_REMAINING;
String DRYA_ARRAY[4];
String DRYA_ARRAY_TEMP[4];

String DRYB_REMAINING;
String DRYB_ARRAY[4];
String DRYB_ARRAY_TEMP[4];

String DRYC_REMAINING;
String DRYC_ARRAY[4];
String DRYC_ARRAY_TEMP[4];

String DRYD_REMAINING;
String DRYD_ARRAY[4];
String DRYD_ARRAY_TEMP[4];

String DRYA = "";
String DRYB = "";
String DRYC = "";
String DRYD = "";
String DryEnable = "NA";
String DryDisable = "NA";
bool DryEnableStatus = false;
bool DryDisableStatus = false;

float CellTemp1 = 0;
float CellTemp2 = 0;

int SettedCANID = 0;



bool mute = false;

bool RTCCongifured = false;
bool RTCFetched = false;
int saat ;
int dakika;
int saniye;
int gun;
int ay;
int yil;
int CellMod = 0;
int CompanyMod = 0;
bool SimMode = false;
int SimCell = 16;
int SimVolt = 48;
String PreviousDay = "";

//UDP VARIABLES
String UDP_STRING;
String UDP_REMAINING;
int UDP_RTCindex = 0;
int UDP_RTC_ARRAY[6];


int param_start2 = 0;
int param_end2 = 0;
int UDP_Dryindex;
String UDP_DRY_ARRAY[4];
String UDPDryString;


unsigned int parameterudp_start = 0;
unsigned int parameterudp_end = 0;


bool service = false;



bool SDReset = false;
File rootSD;
int DeletedCount = 0;
int FolderDeleteCount = 0;
int FailCount = 0;
String rootpath = "/";

uint8_t saatUInt = (uint8_t)saat;
uint8_t dakikaUInt = (uint8_t)dakika;
uint8_t saniyeUInt = (uint8_t)saniye;
uint8_t gunUInt = (uint8_t)gun;
uint8_t ayUInt = (uint8_t)ay;
int timeoffset = 0;

String AlarmLog[30];
bool AlarmTrigger = false;

String OperationList[2] = {"<=", ">"};
String ContactNameList[4] = {"Contact A", "Contact B", "Contact C", "Contact D"};
String ContactFunctionList[9] = {"Term.Volt.", "abs(Current)", "Temperature", "SOC", "DISABLE", "ENABLE", "WAKE", "SAFETY", "CELL-DIF"};
int OperationListIndex = 0;
int ContactNameListIndex = 0;
int ContactFunctionListIndex = 0;
//SIRIUS VARIABLES
int dashboard_page = 0;



TaskHandle_t FIREBASE;
TaskHandle_t UDPTEST;
TaskHandle_t TOUCH_SCREEN;
TaskHandle_t BMS_COMM;
TaskHandle_t BT;
TaskHandle_t ENCONNECT;
TaskHandle_t SDCANBUS;
TaskHandle_t THERMISTOR;

bool WIFIBT = false;

//ENCONNECT

typedef struct
{
  String filename;
  String ftype;
  String fsize;
} fileinfo;

String   webpage, MessageLine;
fileinfo Filenames[200]; // Enough for most purposes!
bool     StartupErrors = false;
int      start, downloadtime = 1, uploadtime = 1, downloadsize, uploadsize, downloadrate, uploadrate, numfiles;



#include "Free_Fonts.h"

#include "charge.h"
#include "discharge.h"
#include "uplog.h"
#include "downlog.h"
#include "maxIcon.h"
#include "minIcon.h"
#include "voltage.h"
#include "current.h"
#include "temp.h"
#include "wifi.h"
#include "dashboard.h"
#include "graph.h"
#include "cell.h"
#include "balance.h"
#include "energy.h"
#include "settings.h"
#include "battery.h"
#include "Encap.h"
#include "alarms.h"
#include "timeIcon.h"
#include "date.h"
#include "back.h"
#include "SYSTEMSTAT.h"
#include "simulation.h"
#include "alarmlog.h"
#include "info.h"
#include "left.h"
#include "main.h"
#include "right.h"
#include "serial.h"
#include "banner.h"
#include "canid.h"
#include "OFF.h"
#include "ON.h"
#include "up.h"
#include "down.h"
#include "save.h"
#include "menu1.h"
#include "max.h"
#include "min.h"
#include "sim2.h"
#include "manuals.h"
#include "wifidirect.h"
#include "updateFirmware.h"
#include "onlineMonitoring.h"
#include "bluetooth.h"
#include "restart.h"
#include "ssid.h"
#include "protocol.h"
#include "ip.h"
#include "port.h"
#include "mac.h"
#include "drycontact.h"
#include "networksettings.h"
#include "etisalaticon.h"
#include "encaplogo.h"
#include "harnysslogo.h"
#include "WIFILAND1.h"
#include "WIFILAND2.h"
#include "flags.h"
#include "CLOUDLAND1.h"
#include "CLOUDLAND2.h"
#include "BLUETOOTHLAND1.h"
#include "BLUETOOTHLAND2.h"
#include "safety.h"
#include "nosafety.h"
#include "volume.h"
#include "novolume.h"

#include "calibrate.h"
#include "monitoringqr.h"
#include "ensiriuslogo.h"
#include "ensegalogo.h"
#include "bannerright.h"
#include "bannerleft.h"
#include "bannermiddle.h"
#include "clockfooter.h"
#include "footernew.h"
#include "blue_dash.h"
#include "can_dash.h"
#include "sd_dash.h"
#include "wifi_dash.h"
#include "alarm_dash.h"
#include "sim_dash.h"
#include "erase_record.h"
#include "firebaseReadyIcon.h"
#include "udp_dash.h"
#include "safe.dash.h"
#include "unsafe.dash.h"
#include "hazard.h"
#include "calender.h"
#include "enlight.h"
#include "anim2.h"
#include "etisalat.h"
#include "eventrecord.h"
#include "inverter.h"
#include "ethernet.h"
#include "ENWALL.h"

//FIREBASE
#define FIREBASE_HOST "encap-2f8c2-default-rtdb.firebaseio.com"
#define FIREBASE_AUTH "l7nIs6w3xs8IYebe1SZgjqIod0SpGygWRyMXNgfh"
unsigned long sendDataPrevMillis = 0;
unsigned long count = 0;






//FONTS
#include "Fonts.h"

//ENCONNECT
///////////////////////////
// Built-in
#include <WebServer.h>
#include <ESPmDNS.h>
#include "CSS.h"

WebServer server(80);

////////////////////////////


String LanguageArray[4] = {"English", "Turkce", "Deutsch", "Francais"};
int indexLanguageArray = 0;


String SerialNumber;
boolean firebaseReady = false;

float AllCells[16];
String UDPprocessor = "";

//////////////////////////////

String INVERTERS[20] = { "NONE", "PYLON", "GROWATT", "SOFAR", "VOLTRONIC", "GOODWE", "SRNE", "MUST", "VICTRON", "SMA", "DYE", "AISWEI", "SOCALAR", "SOLARK", "XMT", "R2", "R3" };
String COMM[2] = { "RS485", "CAN" };
int indexINVERTERSArray = 0;
int indexCOMMArray = 0;
bool SetInverterComm = false;







////////////////////

#include <Wire.h>
#include <Adafruit_FT6206.h>
Adafruit_FT6206 ts = Adafruit_FT6206();
#include <Preferences.h>



Preferences preferences;
WiFiManager wifiManager;

TFT_eSPI    tft = TFT_eSPI();
TFT_eSprite Display = TFT_eSprite(&tft);
TFT_eSprite PageFrame = TFT_eSprite(&tft);




TFT_eSprite ContentFrame = TFT_eSprite(&tft);
TFT_eSprite BannerFrame = TFT_eSprite(&tft);


//TFT_eSprite GraphBackDrop = TFT_eSprite(&tft);
//TFT_eSprite GraphDrawing = TFT_eSprite(&tft);



TFT_eSprite Icon1 = TFT_eSprite(&tft);
TFT_eSprite Icon2 = TFT_eSprite(&tft);
TFT_eSprite Animation = TFT_eSprite(&tft);




TFT_eSprite BannerIconLeft = TFT_eSprite(&tft);
TFT_eSprite BannerIconMiddle = TFT_eSprite(&tft);
TFT_eSprite BannerIconRight = TFT_eSprite(&tft);

TFT_eSprite FooterIconLeft = TFT_eSprite(&tft);
TFT_eSprite FooterFrame = TFT_eSprite(&tft);

#if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED)
#error Bluetooth is not enabled! Please run `make menuconfig` to and enable it
#endif


/////////////////backlight

const uint8_t ledPin = 23;
// setting PWM properties
const int freq = 5000;
const uint8_t ledChannel = 0;
const uint8_t resolution = 8;

bool canbusReady = false;
bool bluetoothReady = false;

//Safety Settings
bool Safety = false;
bool SafetyPrevious = Safety;
bool SafetyChanged = false;
bool SafetyTurnOn = false;
bool SafetyTurnOff = false;
bool SafetyTurnOffApproved = false;
bool SafetyTurnOnApproved = false;
bool SafetyStartUp = false;
bool SafetyStartUp2 = false;

//WIFI SETTINGS

String SSIDName = "ENCAP Controller";
String Port = "2001";
String MAC = "No Connection";
String IP = "192.168.4.1";
bool wifistatus = false;


//Configuration settings
bool UpdateNextRestart = false;
bool WDNextRestart = false;
bool FBNextRestart = false;
bool BTNextRestart = false;
bool UDPNextRestart = true;

int SystemSOC = 100;
float SystemCAP = 216;

float SUMHI = 57;
float SUMLOW = 57;

float SOCLOW = 0;
float SOCHI = 100;

String AlarmArray[28] = {"Cell High 1", "CellHigh 2", "CellLow 1", "CellLow2", "Sum High 1", "SumHigh2", "Sum Low 1", "SumLow2",
                         "Chg.Temp.High", "Chg.Temp.High", "Chg.Temp.Low", "Chg.Temp.Low", "Disch.Temp.High", "Disch.Temp.High",
                         "Disch.Temp.Low", "Disch.Temp.Low",
                         "Chg.O.Curr.", "Chg.O.Curr.", "Disch.O.Curr.", "Disch.O.Curr.", "ModuleFull", "ModuleFull", "SOCLow", "SOCLow",
                         "Diff.Voltage", "Diff.Voltage", "Diff.Temp.", "Diff.Temp."
                        };






float MaxTemp1 = 0;
float MaxTemp2 = 0;

float MaxVoltage = 0;
float MinVoltage = 500;

float MaxCurrent = -500;
float MinCurrent = 500;

float MaxCurrentP = -500;
float MinCurrentP = 500;


//PREFERENCE
float ChargeEnergy;
float DischargeEnergy;

float PreChargeEnergy;
float PreDischargeEnergy;


float HourlyDischargeEnergy = 0;
float HourlyChargeEnergy = 0;

float ChargeFix;
float DischargeFix;
float OpSec = 0;
int OpMin = 0;
int OpHour = 0;
int OpDay = 0;

String LoggedAlarm_1 = "";
String LoggedAlarm_2 = "";
String LoggedAlarm_3 = "";
String LoggedAlarm_4 = "";


// Meter colour schemes
#define RED2RED 0
#define GREEN2GREEN 1
#define BLUE2BLUE 2
#define BLUE2RED 3
#define GREEN2RED 4
#define RED2GREEN 5


////TIMER VARIABLES//////////////////
const char* ntpServer = "pool.ntp.org";
long  gmtOffset_sec = 14400;
int   daylightOffset_sec = 3600;

char timeHour[4];
char timeMin[4];
char timeSec[4];
char timeDay[4];
char timeMonth[4];
char timeYear[10];

String timeHourS;
String timeMinS;
String timeSecS;
String timeDayS;
String timeMonthS;
String timeYearS;

String TimeString = "No Data";
String DateString = "No Data";

//UI Parameters
uint8_t PageNumber = 0;
uint8_t PageLevel = 0;
int SleepCounter = 0;
bool EEPROMbusy = false;
bool balancerState = false;

unsigned long previousMillisUI = 0;
unsigned long intervalUI = 250;
unsigned long currentMillisUI = 0;

bool numpadEnable = false;
String numpadValue = "";
bool notificationEnable = false;
uint8_t notificationCounter = 0;
bool CommandStatus = false;


//BMS Parameters.....
String EmbeddedSerial = "ENCAP0001DEFAULT";
float CVD = 0;
int restartCounter = 0;
float HCD = 0;
float HCC = 0;
float HVC = 0;
float LVC = 0;
float RC = 0;
float RV = 0;
float BV = 0;
float BD = 0;
int HTC = 0;
int HTD = 0;
float SOC_CAL_VAL = 0;
uint16_t SLP = 3600;
bool resetBMS = false;
bool HCDEnable = false;
bool HCCEnable = false;
bool HVCEnable = false;
bool LVCEnable = false;
bool RCEnable = false;
bool RVEnable = false;
bool BVEnable = false;
bool BDEnable = false;
bool SOCEnable = false;

float DryContactValueEnable;
float DryContactValueDisable;
bool DCEnable = false;
bool DCDisable = false;

bool HCC_HCD = false;
bool HVC_LVC = false;
bool RC_RV = false;
bool BV_BD = false;
bool SOC_CAL = false;
bool SOC_ALARM = false;
bool SUM_CAL = false;
bool HTC_HTD = false;
bool SLP_CAL = false;
bool CVD_CAL = false;

bool SERIAL_CAL = false;
bool CURRENT_CAL = false;
bool CURRENT_ZERO_CAL = false;



//EQU PARAMETER

float HVCEQU = 0;
float LVCEQU = 0;
bool HVC_LVC_EQU = false;

bool debug = false;

bool CAL_EQUVOLT = false;
float EQUVOLT_VAL = 0;

bool CAL_EQUCURRENT = false;
float EQUCURRENT_VAL = 0;

bool CAL_EQUSLEEP = false;
uint16_t  EQUSLEEP_VAL = 65535;

bool CAL_EQUSUM = false;
float EQUSUM_HI = 0;
float EQUSUM_LOW = 0;



bool ReadAllParameters = false;

//MOSFET SETTING
bool CHARGE_SET = false;
bool DISCHARGE_SET = false;
bool BALANCE_SET = false;
bool updateIcon = false;


bool notificationStatus = false;
bool parameterSet = false;
int alarmNo = 0;
bool confirmation = false;
bool loading = false;
int loadingPercentage = 0;

bool WriteCharge = false;
bool WriteDischarge = false;
bool WriteBalance = false;


int UDP_COMMAND_INDEX = -1;



//GITHUB UPDATE

String FirmwareVer = {
  "3.2.0 Micro"  //5-20-2024
};


bool UpdateAvailable = false;
bool IPReady = true;
bool CoordinatesReady = true;



//asas
String Languages[5][200] =
{
  {"is ready", "New firmware update", "Initialization", "Term. Voltage", "Term. Current", "Max Cell", "Min Cell", "Max Diff.", "Term. Temp.", "Charge Energy", "Disch. Energy", "System Time", "System Date", "System Alarms", "System Mode", "Cell Temp#1", "Cell Temp#2", "SOC", "Disch.", "Charge", "OFF", "ON", "Main", "Dashboard", "Network", "Settings", "Daily", "Statistics", "Cell", "Monitoring", "Dry", "Contacts", "System", "Settings", "IP Address", "Port Number", "Comm. Protocol", "SSID", "MAC Address", "Forget Network", "Maximum Voltage", "Minimum Voltage", "Maximum Current", "Minimum Current", "Maximum Temp#1", "Maximum Temp#2", "Reset Statistics", "Max Cell", "Min Cell", "Avg Cell", "Contacts", "Types", "Operation", "Enable", "Disable", "Result", "High Current (Disch.)", "High Current (Char.)", "High Voltage (Sum)", "Low Voltage (Sum)", "High Voltage (Cell)", "Low Voltage (Cell)", "High Temp. (Char.)", "High Temp(Disch.)", "Online", "Monitoring", "Wifi Direct", "Monitoring", "Bluetooth", "Monitoring", "Restart", "Module", "Firmware", "Update", "Product", "Manual", "Product Manual", "System", "Setup", "Alarm", "Record", "Monitoring", "QR", "Set Dry", "Contacts", "System", "Stats", "BMS Buzzer", "Terminal Safety", "SET CAN ID", "READ CAN ID", "SET CAN ID", "Set Language", "Alarm Type", "Record Time", "WI-FI Direct", "Monitoring", "Contact Name", "Contact Condition", "Enable Value", "Disable Value", "Contact Type", "Save Configuration", "Total Charge Energy", "Total Discharge Energy", "Highest Curr.Read", "Lowest Curr. Read", "Run Time", "Main Serial:", "BMU Serial:", "Please disconnect load/source to turn on safety", "Online Monitoring is setted", "Wifi Direct Monitoring is setted", "System will restart in 2 sec.", "Firmware update is setted", "QR Code is generated for Sirius ENCAP TDS pdf file", "Bluetooth Monitoring is setted", "Dashboard Page", "Module Cells Page", "Network Settings Page", "Dry Contacts Page", "Daily Statistics Page", "System Setup Page", "Safety is enabled", "Safety is disabled", "Buzzer is muted", "Buzzer is enabled", "Alarm Records Page", "Module is pinged", "Scan barcode for monitoring", "Configure Dry Contacts", "Dry Contacts are saved", "Scan barcode for monitoring", "Numpad is opened for CANID", "Full System Statistics", "Setting CAN ID", "Please choose the preferred language", "Choosen language:", "Mosfets are self switch on by timeout", "Firmware is updating", "Please wait.", "Serial Number:", "Firmware Version:", "Minute", "Daily Voltage (V)", "Max", "Min", "Time", "Daily Charge Energy (W)", "Daily Discharge Energy (W)", "Daily  Temperature (C)", "Select", "Inverter", "Set Inverter Brand", "Set Comm. Type", "Read Inverter Brand", "Read Comm. Type", "SET COMMUNICATION", "Set", "Simulation", "Memory (kb)", "No", "Time", "Event", "Voltage", "Current", "SOC", "Temp", "MaxCell", "MinCell"},
  {"Bulundu", "Yeni Guncelleme", "Baslatiliyor", "Term. Voltaji", "Term Akimi", "Max Hucre", "Min Hucre", "Max Fark", "Term.Sicak.", "Sarj Enerjisi", "Desarj Enerjisi", "Sistem Zamani", "Sistem Tarihi", "Sistem Alarmi", "Sistem Modu", "Hucre Sicakligi#1", "Hucre Sicakligi#2", "SOC", "Desarj", "Sarj", "KAPALI", "ACIK", "Ana", "Ekran", "Ag", "Ayarlari", "Gunluk", "Degerler", "Hucre", "Izleme", "Kuru", "Kontaklar", "Sistem", "Ayarlari", "IP Adresi", "Port Numarasi", "Hab.Protocol", "Ag adi", "MAC Adresi", "Agi Unut", "Maksimum Volt", "Minimum Volt", "Maximum Akim", "Minimum Akim", "Maximum Sic. #1", "Maximum Sic. #2", "Degerleri sifirla", "Max Hucre", "Min Hucre", "Ort Hucre", "Role", "Tip", "Operasyon", "Acik", "Kapali", "Sonuc", "Yuksek Akim(Des.)", "Yuksek Akim(Sarj)", "Yuksek Volt.(Top.)", "Dusuk Volt.(Top.)", "Yuksek Volt.(Hucre)", "Dusuk Volt.(Hucre)", "Yuksek Sic.(Sarj)", "Yuksek Sic.(Des.)", "Cevrimici", "Izleme", "Wifi", "Izleme", "Bluetooth", "Izleme", "Tekrar", "Baslat", "Yazilim", "Guncelle", "Cihaz", "Kitapcigi", "Cihaz Kitapcigi", "Sistem", "Kurulum", "Hata", "Kayitlari", "QR", "Izleme", "Role", "Yaz", "Sistem", "Degerleri", "Sistem Uyarici", "Terminal Guvenlik", "CAN ID Yaz", "CAN ID Oku", "CAN ID Yaz", "Dil Secimi Yap", "Hata Tipi", "Kayit Zamani", "WIFI Direkt", "Izleme", "Role ismi", "Role Operasyon", "Acik Deger", "Kapali Deger", "Role Tipi", "Ayarlari Kaydet", "Toplam Sarj Enerji", "Toplam Desarj Enerji", "En Yuksek Akim", "En Dusuk Akim", "Sistem Zaman Sayicisi", "Ana Kart SN:", "Islemci SN:", "Lutfen sistemden akimi kesiniz.", "Ag ici izleme secildi", "Wifi izleme secildi", "Sistem tekrar baslayacak.", "Yazilim guncelleniyor", "QR Barkod olusturuldu.", "Bluetooth izleme secildi", "Ana Ekran", "Hucre Izleme", "Ag Atarlari", "Roleler", "Gunluk Degerler", "Sistem Kurulumlari", "Guvenlik Acildi", "Guvenlik Kapatildi", "Uyarici Acildi", "Uyarici Kapandi", "Alarm Kayitlari Acildi.", "Cihaza ping atildi", "Izlemek icin barkodu tarayiniz", "Roleleri Ayarla", "Roleler kaydedildi", "Izlemek icin barkodu tarayiniz", "Numpad CAN ID icin acildi", "Tum Sistem Degerleri", "CAN ID Yaz", "Tercih edilen dili seciniz.", "Secilen Dil:", "Roleler kendiliginden acildi.", "Yazilim Guncelleniyor", "Lutfen Bekleyiniz.", "Seri Numarasi", "Yazilim Surumu:", "Dakika", "Gunluk Voltaj (V)", "Maks", "Min", "Zaman", "Gunluk Sarj Enerjisi (W)", "Gunluk Desark Enerjisi (W)", "Gunluk Sicaklik (C)", "Inverter", "Secimi", "Inverter Markasi Sec", "Hab. Methodu Sec", "Inverter Markasi Oku", "Hab. Methodu Oku", "HAB. AYARLA", "Simulasyon ", "Kur", "Hafiza (kb)", "No", "Zaman", "Olay", "Voltaj", "Akim", "SOC", "Sicaklik", "Maks Huc", "Min Huc"},
  {"Bereit", "Neues Firmware-Update", "Initialisierung", "Klemmenspannung", "Klemmenstrom", "Max Zelle", "Min Zelle", "Max Diff.", "Temp. Zelle", "Ladeenergie", "Entl. Energie", "Systemzeit", "Systemdatum", "Systemalarme", "Systemmodus", "Zelltemp#1", "Zelltemp#2", "SOC", "Entl.", "Laden", "AUS", "AN", "Haupt", "Dashboard", "Netzwerk", "Einst.", "Taglich", "Statistiken", "Zelle", "Uberw.", "Pot. freie", "Kontakte", "System", "Einst", "IP-Adresse", "Portnummer", "Komm. Protokoll", "SSID", "MAC-Adresse", "Netzwerk loschen", "Max Spannung", "Min. Spannung", "Max. Strom", "Min. Strom", "Max Temperatur #1", "Max Temperatur #2", "Statistik zurucksetzen", "Max. Zelle", "Min. Zelle", "Dursch. Zelle", "Kontakte", "Typen", "Betrieb", "Aktiv.", "Deakt.", "Erg.", "Strom hoch (Entl.)", "Strom hoch (Laden)", "Spannung hoch (ges.)", "Spannung niedrig (ges.)", "Spannung hoch (Zelle)", "Spannung niedrig (Zelle)", "Temp. hoch (Laden)", "Temp. hoch (Entl.)", "Online", "Ubwg.", "Wifi Direct", "Ubwg", "Bluetooth", "Ubwg.", "Neustart", "Modul", "Firmware", "Update", "Produkt", "Handbuch", "Produkthandbuch", "System", "einrichten", "Alarm", "Daten", "Ubwg.", "QR", "Pot. Kontak.", "Einstellen", "System", "Statistiken", "BMS Summer", "Sicherheit Klemme", "CAN_ID EINS.", "CAN ID lesen", "CAN ID eins.", "Sprache einstellen", "Alarmtyp", "Aufzeichnungszeit", "WI-FI Direct", "Ubwg.", "Name Kontakt", "Bedingung Kontakt", "Wert aktivieren", "Deaktivierungswert", "Kontakt Typ", "Konfiguration speichern", "Ladeenergie ges.", "Entladeenergie ges.", "Max Strom anzeigen", "Min Strom anzeigen", "Laufzeit", "HSN:", "BMU Seriennr.", "Last oder Quelle trennen fur Sicherheitsaktivierung.", "Online-Ubwg. ist aktiviert", "Wifi Direct-uberwachung ist aktiviert", "Das System wird in 2 Sekunden neu starten.", "Firmware-Update ist eingestellt", "QR-Code wird fur die Sirius ENCAP TDS PDF-Datei generiert", "Bluetooth-uberwachung ist aktiviert", "Dashboard Seite", "Modul Zellen Seite", "Netzwerkeinstellungen Seite", "Trockenkontakt Seite", "Tagliche Statistiken Seite", "Systemeinrichtungs Seite", "Sicherheit ist aktiviert", "Sicherheit ist deaktiviert", "Summer ist stummgeschaltet", "Summer ist aktiviert", "Alarmprotokolle Seite", "Modul wird angesteuert", "Barcode scannen zur uberwachung", "Trockenkontakte konfigurieren", "Trockenkontakte sind gespeichert", "Barcode scannen zur uberwachung", "Nummernfeld fur CAN-ID wird geÃ¶ffnet", "VollstÃ¤ndige Systemstatistiken", "CAN-ID festlegen", "Bitte wÃ¤hlen Sie die bevorzugte Sprache", "AusgewÃ¤hlte Sprache:", "Mosfets schalten sich durch Timeout selbst ein", "Firmware wird aktualisiert", "Bitte warten.", "Seriennr.", "Firm. Version:", "Minute", "Tagliche Spannung (V)", "Max", "Min", "Zeit", "Tagliche Ladeenergie (W)", "Tagliche Entladeenergie (W)", "Tagliche Temperatur (C)", "Wahlen ", "WR", "Set WR Marke", "Set Komm. Typ.", "Lesen WR Marke", "Lesen Komm. Typ.", "KOMM. EINST.", "Simulation ", "Einstellen", "Speicher (Kb)", "NR.", "Zeit", "Vorgang", "Spannung", "Strom", "SOC", "Temp", "MaxZelle", "MinZelle"},
  {"Pret", "Nouvelle mise a jour du firmware", "Initialisation", "Tension de Terme.", "Courant de Terme.", "Cell. Max.", "Cell Min.", "Diff. Max.", "Temp. de Terme.", "Energie de Charge", "Energie de Dech", "Heure Systeme", "Date Systeme", "Alarmes Systeme", "Mode Systeme", "Temp. Cell. #1", "Temp. Cell. #2", "SOC", "Dech", "Charge", "ET.", "ALL.", "Principal", "Tab. Bord.", "Reseau", "Parametres", "Quotidien", "Statistiques", "Cellule", "Surveil.", "Sec", "Contacts", "Systeme", "Parametres", "Adresse IP", "Numero de Port", "Protocole de Comm.", "SSID", "Adresse MAC", "Oublier le Reseau", "Tension Maximale", "Tension Minimale", "Courant Maximal", "Courant Minimal", "Temp. Max. #1", "Temp. Max. #2", "Reinitialiser les Statistiques", "Cell. Max.", "Cell. Min.", "Cell. Moy.", "Contacts", "Types", "Operation", "Activer", "Desact", "Result", "Courant Eleve (Dech)", "Courant Eleve (Charge)", "Tension Elevee (Somme)", "Tension Faible (Somme)", "Tension Elevee (Cell.)", "Tension Faible (Cell.)", "Temp. Elevee (Charge)", "Temp. Elevee (Dech)", "En Ligne", "Surveil.", "Wifi Direct", "Surveil.", "Bluetooth", "Surveil.", "Redemarrer", "Module", "Firmware", "Mise a Jour", "Produit", "Manuel", "Manuel du Produit", "Config.", "Mise en Pl.", "Alarme", "Enreg.", "Surveil.", "QR", "Config. Sec.", "Contacts", "Systeme", "Statistiques", "Buzzer BMS", "Securite Terminale", "CONFIGURER l'ID CAN", "LIRE l'ID CAN", "CONFIGURER l'ID CAN", "Choisir la Langue", "Type d'Alarme", "Temps d'Enregistrement", "Wifi Direct", "Surveil.", "Nom du Contact", "Etat du Contact", "Valeur d'Activation", "Valeur de Desactivation", "Type de Contact", "Sauvegarder la Configuration", "Energ. Total. Charg.", "Energ. Total. Dech.", "Lect. Cour. Plus Elev.", "Lect. Cour. Plus Faib.", "Temps Fonc.", "Num. Ser. Princ.", "Numero de Serie BMU :", "Veuillez deconnecter la charge ou la source pour activer la securite", "La Surveil. en Ligne est activee", "La Surveil. Wifi Direct est activee", "Le Systeme va redemarrer dans 2 secondes", "La mise a jour du firmware est activee", "Le Code QR est genere pour le fichier PDF de Sirius ENCAP TDS", "La Surveil. Bluetooth est activee", "Page du Tab. Bord.", "Page des Cellules du Module", "Page des Parametres Reseau", "Page des Contacts Secs", "Page des Statistiques Quot", "Page de Configuration du Systeme", "La Securite est activee", "La Securite est Desacte", "Le Buzzer est mis en sourdine", "Le Buzzer est active", "Page des Alarmes Enregistrees", "Le Module est interroge", "Scanner le code-barres pour la Surveil.", "Configurer les Contacts Secs", "Les Contacts Secs sont sauvegardes", "Scanner le code-barres pour la Surveil.", "Le Pave Numerique est ouvert pour l'ID CAN", "Statistiques Completes du Systeme", "Configuration de l'ID CAN", "Veuillez choisir la langue preferee", "Langue choisie :", "Les Mosfets s'allument automatiquement apres un delai d'attente", "Le Firmware est en cours de mise a jour", "Veuillez patienter.", "Numero de Serie :", "Version du Firmware :", "Minute", "Tension Quot (V)", "Max", "Min", "Tmp", "Energie de Charge Quot (W)", "Energie de Dech Quot (W)", "Temperature Quot (C)", "Choisir ", "Onduleur", "Ens. Marque D'onduleur", "Ens. Comm. Type", "Lire Marque D'onduleur", "Lire Comm. Type", "CONFIG.COMM.", "Set ", "Simulation", "Memory (kb)", "No", "Time", "Event", "Voltage", "Current", "SOC", "Temp", "MaxCell", "MinCell"}
};


#define URL_fw_Version  "https://raw.githubusercontent.com/Emcukcan/FirmwareRep_GodBinary/master/bin_version.txt"
#define URL_fw_Bin  "https://raw.githubusercontent.com/Emcukcan/FirmwareRep_GodBinary/master/fw.bin"
String CellValuesSentence = "";

int Dryindex = 0;

void setup() {

  Serial.begin(9600);
  while (!Serial) continue;
  Serial.setTimeout(250);



  tft.init();
  tft.setRotation(1);
  tft.fillScreen(TFT_BLACK);


  //  //  module
  RXD2 = 32;
  TXD2 = 33;
  //bmsCellNo = 130;


  Serial2.begin(9600, SERIAL_8N1, RXD2, TXD2);

  //buzzer rtc equalizer setup//////////////////////////////////
  //  begin_i2cdevice();
  //  set_gpio();
  //  delay(10);
  //  uarti2c_begin(9600);
  //  delay(100);
  //  device_select(2);
  //  delay(10);
  //  RTC.begin(&I2C_0);

  //MODBUS SLAVE ID
  // modbusid = 0x42;


  //setup sd
  spi_class_begin();
  delay(100);
  //clearSD();
  delay(100);


  //Backlight
  ledcSetup(ledChannel, freq, resolution);
  ledcAttachPin(ledPin, ledChannel);
  ledcWrite(ledChannel, 255);

  EEPROMbusy = true;


  Serial.println("nvs is init");
  esp_err_t ret = nvs_flash_init();
  if (ret == ESP_ERR_NVS_NO_FREE_PAGES) {
    ESP_ERROR_CHECK(nvs_flash_erase());
    ret = nvs_flash_init();
  }
  ESP_ERROR_CHECK(ret);



  preferences.begin("my-app", false);

  indexLanguageArray = preferences.getInt("indexLang", 0);
  SerialNumber = preferences.getString("SerialNumber", "EN02048V00330023A00001");
  timeoffset = preferences.getInt("timeoffset", 0);



  String AS = SerialNumber.substring(22, 24);
  String ASY = SerialNumber.substring(0, 22);
  ASY.trim();
  int ASI = AS.toInt();


  Serial.println(SerialNumber);
  Serial.println(AS);
  Serial.println(ASI);
  Serial.println(ASY);
  Serial.println(ASY.length());


  Serial.println(ASY == "KWL02048V034300024A000" );
  Serial.println(ASI < 45);

  if (SerialNumber.indexOf("SIM") != -1) {
    SimMode = true;   //SIM
    SimCell = SerialNumber.substring(SerialNumber.indexOf("SIM") + 3, SerialNumber.indexOf("SIM") + 5).toInt();
    SimVolt = SerialNumber.substring(SerialNumber.indexOf("SIM") - 2, SerialNumber.indexOf("SIM")).toInt();
    Serial.println("SimCell:" + String(SimCell));
    Serial.println("SimVolt:" + String(SimVolt));
  }
  else {
    SimMode = false;
  }





  if (SerialNumber.substring(0, 17) == "EN03048V054300024") {
    FirmwareVer = "3.0.4";   //non cyclic trick for firmware version
  }

  if (SerialNumber.substring(0, 16) == "EN03048V00330024") {
    FirmwareVer = "1.0.23";   //STANDARD ETISLAT trick for firmware version
  }


  if (SerialNumber.substring(0, 16) == "EN03048V00330023") {
    FirmwareVer = "1.0.23";   //STANDARD ETISLAT trick for firmware version
  }

  if (SerialNumber.substring(0, 16) == "EN03048V00530023") {
    FirmwareVer = "1.0.23";   //STANDARD ETISLAT trick for firmware version
  }

  if (SerialNumber.substring(0, 16) == "KWL02048V0343000") {
    FirmwareVer = "3.0.0";   //STANDARD ETISLAT trick for firmware version
  }


  if (SerialNumber.substring(0, 16) == "EN03048V00330124") {
    CompanyMod = 9;
  }

  else if (SerialNumber.substring(0, 2) == "ES") {
    CompanyMod = 3;   //ENSIRIUS
  }

  else if (SerialNumber.substring(0, 3) == "ENS") {
    CompanyMod = 4;  //ENSEGA
  }


  else if (SerialNumber.substring(0, 4) == "EN03") {
    CompanyMod = 5;   //ETISALAT

  }

  else if (SerialNumber.substring(0, 4) == "EN06") {
    CompanyMod = 5;   //ETISALAT

  }
  else if (SerialNumber.substring(0, 3) == "ELL") {
    CompanyMod = 6;   //ENLIGHT
  }


  else if (ASY == "KWL02048V034300024A000"  &&   ASI < 45 && ASI > 4 ) {
    CompanyMod = 3 ;  //ENSIRIUS

  }


  else if (SerialNumber.substring(0, 3) == "KWL") {
    CompanyMod = 5;   //ENSIRIUS

  }

  else if (SerialNumber.substring(0, 3) == "ENC") {
    CompanyMod = 7 ;  //Containerized ENCAP
  }

  else if (SerialNumber.substring(0, 3) == "ENW") {
    CompanyMod = 8 ;  //Containerized ENWALL
  }
  else {
    CompanyMod = 2;   //ENCAP
  }
  //  Serial.println(CompanyMod);
  //  Serial.println(SerialNumber.substring(0, 4));

  //  Serial.println("Statics are fetched");
  //  Serial.println("Max Permanent Current:" + String(MaxCurrentP));
  //  Serial.println("Min Permanent Current:" + String(MinCurrentP));
  //
  //  Serial.println("timeoffset:");
  //  Serial.println(timeoffset);

  restartCounter = preferences.getInt("RC", 0);
  UpdateNextRestart = preferences.getBool("UNR", false);
  WDNextRestart = preferences.getBool("WDNR", false);
  FBNextRestart = preferences.getBool("FBNR", false);
  BTNextRestart = preferences.getBool("BTNR", true);
  UDPNextRestart = preferences.getBool("UDPNR", false);

  //  Serial.print("Restart Counter:");
  //  Serial.println(restartCounter);

  Safety = preferences.getBool("Safety", false);


  DRYA = preferences.getString("DRYA", "0/0/0/0/0/");
  DRYB = preferences.getString("DRYB", "0/0/0/0/0/");
  DRYC = preferences.getString("DRYC", "0/0/0/0/0/");
  DRYD = preferences.getString("DRYD", "0/0/0/0/0/");

  DRYA_REMAINING = DRYA;
  DRYB_REMAINING = DRYB;
  DRYC_REMAINING = DRYC;
  DRYD_REMAINING = DRYD;


  for (int i = 0; i < 4; i++) {
    // Serial.println("------------");
    Dryindex = DRYA_REMAINING.indexOf('/');
    DRYA_ARRAY[i] = DRYA_REMAINING.substring(0, Dryindex);
    DRYA_REMAINING = DRYA_REMAINING.substring(Dryindex + 1);
    //    Serial.println("Number:" + DRYA_ARRAY[i]);
    //    Serial.println("Remaining:" + DRYA_REMAINING);
    //    Serial.println("------------");
  }

  for (int i = 0; i < 4; i++) {
    // Serial.println("------------");
    Dryindex = DRYB_REMAINING.indexOf('/');
    DRYB_ARRAY[i] = DRYB_REMAINING.substring(0, Dryindex);
    DRYB_REMAINING = DRYB_REMAINING.substring(Dryindex + 1);
    //   Serial.println("Number:" + DRYB_ARRAY[i]);
    //  Serial.println("Remaining:" + DRYB_REMAINING);
    //   Serial.println("------------");
  }


  for (int i = 0; i < 4; i++) {
    // Serial.println("------------");
    Dryindex = DRYC_REMAINING.indexOf('/');
    DRYC_ARRAY[i] = DRYC_REMAINING.substring(0, Dryindex);
    DRYC_REMAINING = DRYC_REMAINING.substring(Dryindex + 1);
    //  Serial.println("Number:" + DRYC_ARRAY[i]);
    //  Serial.println("Remaining:" + DRYC_REMAINING);
    //  Serial.println("------------");
  }

  for (int i = 0; i < 4; i++) {
    // Serial.println("------------");
    Dryindex = DRYD_REMAINING.indexOf('/');
    DRYD_ARRAY[i] = DRYD_REMAINING.substring(0, Dryindex);
    DRYD_REMAINING = DRYD_REMAINING.substring(Dryindex + 1);
    //  Serial.println("Number:" + DRYD_ARRAY[i]);
    //  Serial.println("Remaining:" + DRYD_REMAINING);
    // Serial.println("------------");
  }


  mute = preferences.getBool("mute", false);

  restartCounter++;
  preferences.putInt("RC", restartCounter);
  preferences.end();
  EEPROMbusy = false;

  // SerialNumber = preferences.getString("SerialNumber", "ENC10482000001");




  if (!mute) {
    for (int i = 0; i < 10; i++) {
      delay(100);
      buzzer_on();
      delay(100);
      buzzer_off();
    }
  }




  if (!UpdateNextRestart) {
    //CREATING TASK FOR BMS_____________________________________________________________________________________________________
    xTaskCreatePinnedToCore(
      BMS_COMM_CODE,   /* Task function. */
      "BMS_COMM",     /* name of task. */
      4000,//5000       /* Stack size of task */
      NULL,        /* parameter of the task */
      100,           /* priority of the task */
      &BMS_COMM,      /* Task handle to keep track of created task */
      0);          /* pin task to core 0 */
    delay(50);
  }

  //CREATING TASK FOR BMS_____________________________________________________________________________________________________
  if (FBNextRestart && !UpdateNextRestart) {
    Serial.println("FB started");
    xTaskCreatePinnedToCore(
      FIREBASE_CODE,   /* Task function. */
      "FIREBASE",     /* name of task. */  //8000
      5000,  //8500     /* Stack size of task */
      NULL,        /* parameter of the task */
      12,           /* priority of the task */
      &FIREBASE,      /* Task handle to keep track of created task */
      0);          /* pin task to core 0 */
    delay(50);
  }


  //CREATING TASK FOR BMS_____________________________________________________________________________________________________

  if (WDNextRestart && !UpdateNextRestart) {
    Serial.println("UDP started");
    xTaskCreatePinnedToCore(
      UDPTEST_CODE,   /* Task function. */
      "UDPTEST",     /* name of task. */
      5000,//5000       /* Stack size of task */                  //6000
      NULL,        /* parameter of the task */
      13,           /* priority of the task */
      &UDPTEST,      /* Task handle to keep track of created task */
      0);          /* pin task to core 0 */
    delay(50);
  }


  //CREATING TASK FOR TOUCH SCREEN_____________________________________________________________________________________________________
  xTaskCreatePinnedToCore(
    TOUCH_SCREEN_CODE,   /* Task function. */
    "TOUCH_SCREEN",     /* name of task. */
    4000,       /* Stack size of task */    //4000   //3000
    NULL,        /* parameter of the task */
    96,           /* priority of the task */
    &TOUCH_SCREEN,      /* Task handle to keep track of created task */
    1);          /* pin task to core 0 */
  delay(50);



  if (BTNextRestart) {
    Serial.println("Bluetooth started");
    //CREATING TASK FOR TOUCH SCREEN_____________________________________________________________________________________________________
    xTaskCreatePinnedToCore(
      BT_CODE,   /* Task function. */
      "BT",     /* name of task. */
      9000,//9000       /* Stack size of task */
      NULL,        /* parameter of the task */
      2,           /* priority of the task */
      &BT,      /* Task handle to keep track of created task */
      1);          /* pin task to core 0 */
    delay(50);
  }




  drawLandingPage();
  landingDone = true;


  if (WDNextRestart && !UpdateNextRestart) {
    Serial.println("Wifi-direct started");
    //CREATING TASK FOR TOUCH SCREEN_____________________________________________________________________________________________________
    xTaskCreatePinnedToCore(
      ENCONNECT_CODE,   /* Task function. */
      "ENCONNECT",     /* name of task. */
      6000,       /* Stack size of task */
      NULL,        /* parameter of the task */
      2,           /* priority of the task */
      &ENCONNECT,      /* Task handle to keep track of created task */
      0);          /* pin task to core 0 */
    delay(50);
  }


  Serial.println("SD CARD CANBUS Started");
  //CREATING TASK FOR TOUCH SCREEN_____________________________________________________________________________________________________
  xTaskCreatePinnedToCore(
    SDCANBUS_CODE,   /* Task function. */
    "SDCANBUS",     /* name of task. */
    9500, //9000      /* Stack size of task */   //MINIMUM 7500
    NULL,        /* parameter of the task */
    1,           /* priority of the task */
    &SDCANBUS,      /* Task handle to keep track of created task */
    0);          /* pin task to core 0 */
  delay(50);
}


void SDCANBUS_CODE( void * pvParameters ) {

  Serial.print("WDNextRestart:");
  Serial.println(WDNextRestart);
  Serial.print("UDPNextRestart:");
  Serial.println(UDPNextRestart);
  Serial.print("BTNextRestart:");
  Serial.println(BTNextRestart);
  Serial.print("WIFI:");
  Serial.println(WiFi.status() != WL_CONNECTED);



  if (WDNextRestart || FBNextRestart) {
    Serial.println("WIFI WILL START HERE");


    WiFiManager wifiManager;
    std::vector<const char *> wm_menu  = {"wifi", "exit"};
    wifiManager.setShowInfoUpdate(false);
    wifiManager.setShowInfoErase(false);
    wifiManager.setMenu(wm_menu);

    esp_task_wdt_init(150, false);
    String isim =  "ENCAP-WIFI-" + SerialNumber.substring(SerialNumber.length() - 5);
    //Wifi interface____________________________________________________________________________________________________________
    wifiManager.setCustomHeadElement("<style> h2 {padding: 30px;text-align: center;background: #0099ff;color: white;font-size: 30px; }footer {position: fixed;left: 0; bottom: 0;width: 100%;background-color: #0099ff;color: white;text-align: center;</style><h2>ENCONNECT+ Interface</h2><footer><p>Amber&Waseem Software Development and Design</p><p></p></footer>");
    wifiManager.setTimeout(120);
    if (!wifiManager.autoConnect((const char*)isim.c_str())) {
      //if (!wifiManager.autoConnect("encap", "adminadmin")) {
      Serial.println(F("failed to connect and hit timeout"));
      delay(250);
      Serial.println("re-connecting.....");
      Serial.println("resetting");

      //WiFi.disconnect(true);
      // WiFi.mode(WIFI_OFF);
      delay(100);
      // WIFIBT = true;
      //  Serial.println(F("BT Terminal is enabled"));
      wifistatus = false;
    }

    Serial.println(F("Connection is established"));
    //Serial.println(F("BT Terminal is disabled"));
    wifiManager.autoConnect((const char*)isim.c_str());
    //  IPAddress subnet(255, 255, 255, 0);
    //
    //  IPAddress localGateway(192, 168, 60, 1); //hardcoded
    Serial.println((WiFi.status()));
    Serial.printf("Now listening at IP %s, UDP port %d\n", WiFi.localIP().toString().c_str(), 2001);
    delay(250);

    unsigned long previousMillis = 0;
    unsigned long interval = 5000;
    unsigned long currentMillis = 0;
    unsigned long UDPCOMMAND = 0;
    SSIDName = WiFi.SSID();
    IP = WiFi.localIP().toString().c_str();
    Port = 2001;
    MAC = WiFi.macAddress();
    wifistatus = true;

    PreviousDay = String(BMS.day);
  }


  if (!BTNextRestart ) {
    while (WiFi.status() != WL_CONNECTED ) {
      Serial.print("SDCAN TASK:");
      Serial.println(F("1-)Waiting for Network Connection"));
      delay(500);
    }
  }





  //  for (int i = 0; i < 5; i++) {
  //
  //    if (CAN_OK == CAN.begin(CAN_500KBPS, MCP_8MHz)) {
  //      Serial.println("");
  //      Serial.println("CAN BUS init ok!.....................................");
  //      Serial.println("");
  //      canbusReady = true;
  //      delay(100);
  //      break;
  //    }
  //    else {
  //      Serial.println("");
  //      Serial.println("CAN BUS init fail.............................");
  //      Serial.println("");
  //      canbusReady = false;
  //      delay(100);
  //    }
  //  }




  for (;;) {

    if (WDNextRestart || FBNextRestart) {
      if ((WiFi.status() != WL_CONNECTED) ) {
        Serial.println(F("RECONNECTING TO WIFI"));
        WiFi.disconnect();
        WiFi.reconnect();
        wifistatus = false;
        delay(250);
      }
      else {
        wifistatus = true;
      }
    }


    //Serial.println("SD ---------------------opsec start" + String(ceil(OpSec)));
    if (GPSFetched == false && WiFi.status() == WL_CONNECTED && (ceil(OpSec) >= 5 && ceil(OpSec) <= 11) && OpMin == 2) {
      Serial.println("checking for location...");
      getLocation();
      delay(250);
    }

    //
    //
    //
    //    if (RTCFetched == false  && WiFi.status() == WL_CONNECTED && (ceil(OpSec) >= 15 && ceil(OpSec) <= 20)) {
    //
    //      Serial.println("checking for rtc");
    //      Serial.print("timeoffset:");
    //      Serial.println(timeoffset);
    //
    //      gmtOffset_sec = timeoffset;
    //      configTime(gmtOffset_sec, daylightOffset_sec, ntpServer);
    //
    //      if (printLocalTime()) {
    //
    //        saat = timeHourS.toInt();
    //        dakika = timeMinS.toInt();
    //        saniye = timeSecS.toInt();
    //        gun = timeDayS.toInt();
    //        ay = timeMonthS.toInt();
    //        yil = timeYearS.toInt();
    //
    //        saatUInt = (uint8_t)saat;
    //        dakikaUInt = (uint8_t)dakika;
    //        saniyeUInt = (uint8_t)saniye;
    //        gunUInt = (uint8_t)gun;
    //        ayUInt = (uint8_t)ay;
    //        RTCCongifured = true;
    //        RTCFetched = true;
    //
    //        Serial.print("Time Configuration is done:");
    //        Serial.println(timeHourS + ":" + timeMinS + ":" + timeSecS);
    //        Serial.print(saatUInt);
    //        Serial.print(dakikaUInt);
    //        Serial.print(saniyeUInt);
    //        SystemSOC = BMS.SOC * 0.1;
    //        SOC_CAL = true;
    //        Serial.print("SOC:");
    //        Serial.println(SystemSOC );
    //        Serial.println("BMS RTC & SOC  will be setted");
    //
    //
    //      }
    //
    //      else {
    //        Serial.println("Time Configuration is failed");
    //      }
    //    }
    //    else {
    //      //      Serial.print("RTCFetched:");
    //      //      Serial.println(RTCFetched);
    //      //
    //      //      Serial.print("WiFi.status():");
    //      //      Serial.println(WiFi.status() == WL_CONNECTED );
    //      //
    //      //      Serial.print("OPSEC:");
    //      //      Serial.println(ceil(OpSec));
    //
    //    }


    int a = OpSec;
    //    if ( canbusReady == true && !(ceil(OpSec) >= 20 && ceil(OpSec) < 30) ) {
    //      //Serial.println("CANBUS WORKED");
    //
    //      if (BMS.SOC * 0.1 < 80 && BMS.SOC * 0.1 > 30) {
    //        can_send_charge(55 , 200, 200);
    //
    //      }
    //      else if (BMS.SOC * 0.1 > 80 && BMS.SOC * 0.1 < 90)
    //      {
    //        can_send_charge(55 , 100, 200);
    //      }
    //      else if (BMS.SOC * 0.1 > 90)
    //      {
    //        can_send_charge(55 , 20, 200);
    //      }
    //      else if (BMS.SOC * 0.1 < 30 && BMS.SOC * 0.1 > 20)
    //      {
    //        can_send_charge(55 , 200, 100);
    //      }
    //
    //      else if (BMS.SOC * 0.1 < 20)
    //      {
    //        // can_send_charge(55 , 200, 20);
    //      }
    //
    //      can_send_soc(BMS.SOC * 0.1 , BMS.SOC * 0.1);
    //      // Serial.println(read_can(),HEX);
    //      can_send_total_volt(BMS.sum_voltage * 0.1, (BMS.current - 30000) * 0.1, BMS.max_cell_temp - 40);
    //      can_send_request(1, 1 , 0, 0, 0);
    //      can_send_alarm(alarmsr) ;
    //      // Serial.println(read_can(), HEX);
    //      /////////////////////
    //
    //    }



    delay(500);


    //SD_CARD
    if (!SDReset) {




      //  Serial.println("SD ---------------------opsec sd execute:" + String(ceil(OpSec)));
      if (ceil(OpSec) >= 25 && ceil(OpSec) <= 26 ) {

        for (int i = 0; i < 20; i++) {

          control_sd = sd_card_init(SD_CS_PIN , hspi );
          if (control_sd) {
            Serial.println("");
            Serial.println("SD Card init ok!.....................................");
            Serial.println("");
            delay(100);
            break;
          }
          else {

            Serial.println("");
            Serial.println("SD Card init fail.............................");
            Serial.println("");
            clearSD();
            Serial.println("SD Card is resetted");

            delay(250);
          }
        }




        //STATISTICAL GRAPH SETUP START

        //  createDirectory();

        //String Address = "/BMS_" + String(BMS.day) + "_" + String(BMS.month) + "_" + String(BMS.year) + "-" + SerialNumber.substring(SerialNumber.length() - 5) + ".xls";
        String Address = "/BMS_" + String(BMS.day) + "_" + String(BMS.month) + "_" + String(BMS.year) + "-" + SerialNumber.substring(SerialNumber.length() - 5) + ".xls";
        String DailyVoltageAddress = "/Stat_" + String(BMS.day) + "_" + String(BMS.month) + "_" + String(BMS.year) + "-" + SerialNumber.substring(SerialNumber.length() - 5) + "_Voltage.xls";
        String DailyTemperatureAddress = "/Stat_" + String(BMS.day) + "_" + String(BMS.month) + "_" + String(BMS.year) + "-" + SerialNumber.substring(SerialNumber.length() - 5) + "_Temperature.xls";
        String HourlyDischargeAddress = "/Stat_" + String(BMS.day) + "_" + String(BMS.month) + "_" + String(BMS.year) + "-" + SerialNumber.substring(SerialNumber.length() - 5) + "Discharge.xls";
        String HourlyChargeAddress = "/Stat_" + String(BMS.day) + "_" + String(BMS.month) + "_" + String(BMS.year) + "-" + SerialNumber.substring(SerialNumber.length() - 5) + "Charge.xls";

        // Serial.println(Address);


        //    BMS.year = (2000 + BMS_Recieve[4]);
        //            BMS.month = BMS_Recieve[5];
        //            BMS.day = BMS_Recieve[6];
        //            BMS.when = BMS_Recieve[7];
        //            BMS.point = BMS_Recieve[8];
        //            BMS.second = BMS_Recieve[9];


        String SDCard_row[18];

        SDCard_row[0] = String(BMS.when) + ":" + String(BMS.point) + ":" + String(BMS.second);
        SDCard_row[1] = String(BMS.sum_voltage * 0.1, 3);
        SDCard_row[2] = String((BMS.current - 30000) * 0.1, 3);
        SDCard_row[3] = String(BMS.max_cell_temp - 40);
        SDCard_row[4] = String(BMS.charge) + "/" + String(BMS.discharge) + "/" + "Not Valid" + "/" + String(Safety);
        SDCard_row[5] = String(BMS.max_cell_volt * 0.001, 3);
        SDCard_row[6] = String(BMS. min_cell_volt * 0.001, 3);
        SDCard_row[7] = String(BMS.max_cell_volt * 0.001 - BMS. min_cell_volt * 0.001, 3);
        SDCard_row[8] = String(BMS.SOC * 0.1, 3);
        SDCard_row[9] = String(ChargeEnergy, 3);
        SDCard_row[10] = String(DischargeEnergy, 3);

        if (DRY_VALUES[0] == 0) {
          SDCard_row[11] = "OFF";
        }
        else {
          SDCard_row[11] = "ON";
        }
        if (DRY_VALUES[1] == 0) {
          SDCard_row[12] = "OFF";
        }
        else {
          SDCard_row[12] = "ON";
        }

        if (DRY_VALUES[2] == 0) {
          SDCard_row[13] = "OFF";
        }
        else {
          SDCard_row[13] = "ON";
        }

        if (DRY_VALUES[3] == 0) {
          SDCard_row[14] = "OFF";
        }
        else {
          SDCard_row[14] = "ON";
        }

        String AlarmString3 = "No Alarm";
        for (int i = 0; i < 28; i++) {
          if (BMS.error[i]) {
            AlarmString3 = AlarmArray[i];
            break;
          }
        }
        SDCard_row[15] = AlarmString3;
        SDCard_row[16] = String(CellTemp1) + "/" + String(CellTemp2);
        SDCard_row[17] = String(WDNextRestart) + "/" + String(FBNextRestart) + "/" + String(BTNextRestart);













        // if (String(BMS.year).toInt() >= 1 && abs((BMS.current - 30000) * 0.1) < 400 && abs(BMS.max_cell_temp - 40) < 100) {

        if (String(BMS.year).toInt() >= 1 ) {

          if (!SD.exists(Address)) {
            write_title( Address.c_str());
            Serial.println("title is written");
            control_sd = false;
            SD.end();
          }
          else {
            control_sd = true;
          }

          write_data( Address.c_str(), SDCard_row);
          delay(500);




          //    BMS.year = (2000 + BMS_Recieve[4]);
          //            BMS.month = BMS_Recieve[5];
          //            BMS.day = BMS_Recieve[6];
          //            BMS.when = BMS_Recieve[7];
          //            BMS.point = BMS_Recieve[8];
          //            BMS.second = BMS_Recieve[9];





          for (int i = 0; i < 24; i++) {
            VoltageArrayString[i] = String(VoltageArray[i]) ;
            //Serial.println("Written1 VoltageString Hour:" + String(i));
            // Serial.println(VoltageArrayString[i]);
          }




          VoltageArray[String(BMS.when).toInt()] = BMS.sum_voltage;
          for (int i = 0; i < 24; i++) {
            VoltageArrayString[i] = String(VoltageArray[i]) ;
            //            Serial.println("Written2 VoltageString Hour:" + String(i));
            //            Serial.println(VoltageArrayString[i]);
          }



          TemperatureArray[String(BMS.when).toInt()] = (BMS.max_cell_temp - 40) * 10;
          for (int i = 0; i < 24; i++) {
            TemperatureArrayString[i] = String(TemperatureArray[i]) ;
            //            Serial.println("TemperatureString Hour:" + String(i));
            //            Serial.println(TemperatureArrayString[i]);
          }
          //
          //
          HourlyDischargeArray[String(BMS.when).toInt()] = HourlyDischargeEnergy;
          // HourlyDischargeArray[String(BMS.when).toInt()] = 89;
          for (int i = 0; i < 24; i++) {
            HourlyDischargeArrayString[i] = String(HourlyDischargeArray[i]) ;
            //            Serial.println("HourlyDischargeArrayString Hour:" + String(i));
            //            Serial.println(HourlyDischargeArrayString[i]);
          }

          //
          HourlyChargeArray[String(BMS.when).toInt()] = HourlyChargeEnergy;

          for (int i = 0; i < 24; i++) {
            HourlyChargeArrayString[i] = String(HourlyChargeArray[i]) ;
            //            Serial.println("HourlyChargeArrayString Hour:" + String(i));
            //            Serial.println(HourlyChargeArrayString[i]);
          }



          if (String(BMS.year).toInt() > 1 && String(BMS.when).toInt() == 0 && (String(BMS.point).toInt() > 2 && String(BMS.point).toInt() < 5)) { // Reset Voltage Data for the new day
            for (int i = 0; i < 24; i++) {
              HourlyDischargeArray[i] = 0;
              HourlyChargeArray[i] = 0;
              VoltageArray[i] = 0;
              TemperatureArray[i] = 0;
            }
            Serial.println("DATA IS RESETTED----------------------------------------------------------------------------------------------------");
          }

          if ((String(BMS.point).toInt() == 0 && String(BMS.year).toInt() > 1 )) {
            HourlyDischargeEnergy = 0;
            HourlyChargeEnergy = 0;
          }



          if (control_sd ) {

            delay(250);

            if ((FBNextRestart || BTNextRestart)) {
              Serial.println("----------------------SD Closed--------------");
              SD.end();
            }
            else {
              if (OpMin == 1 ) {
                SD.end();
              }
            }



            //
          }
          else {
            Serial.println("System is waiting for SD.................................");
          }





        }
        else {
          Serial.println("Main Log is waiting for RTC.....................................");
          Serial.println("Year:" + String(BMS.year));
        }



        // Statistical Measurements Done/////////////////
      }


    }


    else {
      rootSD = SD.open("/");
      delay(100);
      rm(rootSD, rootpath);
      SDReset = false;
      Serial.println("SD Card files deleted!!!!!!!!!!!");
    }







    if (landingDone) {
      // Serial.println("-----------------Statics working----------------");
      if (PreviousDay != String(BMS.day) && PreviousDay != "") {
        //Serial.println("PreviousDay:" + PreviousDay);
        // Serial.println("RTC DAY:" + String(BMS.day));
        MaxTemp1 = 0;
        MaxTemp2 = 0;
        MaxVoltage = 0;
        MinVoltage = 500;
        MaxCurrent = -500;
        MinCurrent = 500;
        Serial.print("PreviousDay:");
        Serial.println(PreviousDay);
        Serial.print("CurrentDay:");
        Serial.println(BMS.day);
        Serial.println("-----------------Statics are resetted----------------");
        PreviousDay = String(BMS.day);
      }


      if (MaxTemp1 < CellTemp1 && CellTemp1 < 90) {
        MaxTemp1 = CellTemp1;
      }
      if (MaxTemp2 < CellTemp2 && CellTemp2 < 90) {
        MaxTemp2 = CellTemp2;
      }
      if (MaxVoltage < BMS.sum_voltage * 0.1) {
        MaxVoltage = BMS.sum_voltage * 0.1;
      }
      if (MinVoltage > BMS.sum_voltage * 0.1 && BMS.sum_voltage * 0.1 != 0) {
        MinVoltage = BMS.sum_voltage * 0.1;
      }
      if (MaxCurrent < (BMS.current - 30000) * 0.1 && abs((BMS.current - 30000) * 0.1) != 3000) {
        MaxCurrent = (BMS.current - 30000) * 0.1;
      }
      if (MinCurrent > (BMS.current - 30000) * 0.1 && abs((BMS.current - 30000) * 0.1) != 3000) {
        MinCurrent = (BMS.current - 30000) * 0.1;
      }
      if (MaxCurrentP < (BMS.current - 30000) * 0.1 && abs((BMS.current - 30000) * 0.1) != 3000) {
        MaxCurrentP = (BMS.current - 30000) * 0.1;
      }
      if (MinCurrentP > (BMS.current - 30000) * 0.1 && abs((BMS.current - 30000) * 0.1) != 3000) {
        MinCurrentP = (BMS.current - 30000) * 0.1;
      }
    }
    delay(1000);

  }

}





void ENCONNECT_CODE( void * pvParameters ) {


  while (WiFi.status() != WL_CONNECTED) {
    Serial.println(F("Enconnect Server waiting..."));
    delay(500);
  }

  // The logical name http://fileserver.local will also access the device if you have 'Bonjour' running or your system supports multicast dns
  if (!MDNS.begin("fileserver")) {          // Set your preferred server name, if you use "myserver" the address would be http://myserver.local/
    Serial.println(F("Error setting up MDNS responder!"));
    ESP.restart();
  }




  server.on("/",         HomePage);
  server.on("/dir",       Dir);
  server.on("/download", File_Download);
  server.on("/upload",   File_Upload);
  server.on("/fupload",  HTTP_POST, []() {
    server.send(200);
  }, handleFileUpload);
  ///////////////////////////// End of Request commands
  server.begin();
  Serial.println("HTTP server started");




  /////////////////


  for (;;) {
    // Serial.println(F("Enconnect Server working"));
    server.handleClient(); // Listen for client connections

    delay(500);
  }

}

void BT_CODE( void * pvParameters ) {

  WIFIBT = true;
  String message = "";
  char incomingChar;
  int param_start = 0;
  int param_end = 0;
  String BTProcessor;
  int param_start2 = 0;
  int param_end2 = 0;
  String BTProcessor2;

  String BT_STRING;
  String BT_REMAINING;
  int BT_Dryindex;
  int BT_RTCindex;
  String BT_DRY_ARRAY[4];
  String DryString;
  int BT_RTC_ARRAY[6];





  Serial.println(F("WIFI is Off, BT is On"));
  wifistatus = false;
  String BT_STATION = "ENCAP-BT-" + SerialNumber.substring(SerialNumber.length() - 5);
  Serial.println(BT_STATION);

  BluetoothSerial SerialBT;
  SerialBT.begin(BT_STATION.c_str()); //Bluetooth device name
  Serial.println(F("The device started, now you can pair it with bluetooth!"));
  bluetoothReady = true;

  for (;;) {

    if (SerialBT.available()) {
      char incomingChar = SerialBT.read();
      if (incomingChar != '\n') {
        message += String(incomingChar);
      }
      else {
        message = "";
      }


      if (message != "") {
        Serial.println(message);
      }



      ////////////////////NEW COMMANDS 9-21-2023


      //SET METHOD////////////////////////////////
      param_start2 = message.indexOf("SETMETHOD");
      param_end2 = message.indexOf("/");
      if (param_start2 != -1 && param_end2 != -1) {
        BTProcessor2 = message.substring(param_start2 + 9, param_end2);
        METHOD = BTProcessor2.toInt();


      }///////////////////////////////////////////////

      //SET TYPES////////////////////////////////
      param_start2 = message.indexOf("SETTYPE");
      param_end2 = message.indexOf("#");
      if (param_start2 != -1 && param_end2 != -1) {
        BTProcessor2 = message.substring(param_start2 + 7, param_end2);
        TYPE = BTProcessor2.toInt();
        message = "";
        SerialBT.print(METHOD);
        SerialBT.print("/");
        SerialBT.println(TYPE);
        SetCOMM = true;
      }///////////////////////////////////////////////





      //SET energy records////////////////////////////////
      param_start2 = message.indexOf("SETCHARGE");
      param_end2 = message.indexOf("/");
      if (param_start2 != -1 && param_end2 != -1) {
        BTProcessor2 = message.substring(param_start2 + 9, param_end2);
        SetCharge = BTProcessor2.toFloat();
        SerialBT.println(SetCharge);

      }///////////////////////////////////////////////


      param_start = message.indexOf("SETDISCHARGE");
      param_end = message.indexOf("#");
      if (param_start != -1 && param_end != -1) {
        BTProcessor = message.substring(param_start + 12, param_end);
        SetDischarge = BTProcessor.toFloat();
        message = "";
        SerialBT.print(SetCharge);
        SerialBT.print("/");
        SerialBT.println(SetDischarge);
        SetEnergies = true;
      }///////////////////////////////////////////////









      //RESET ALARMS RECORDS////////////////////////////////
      param_start = message.indexOf("RESETALARM");
      param_end = message.indexOf("#");
      if (param_start != -1 && param_end != -1) {


        for (int i = 0; i < 5; i++) {
          AlarmLog[i] = "No Alarm";
        }

        EEPROMbusy = true;

        EEPROMbusy = false;

        SerialBT.println("Alarm records are resetted");
        message = "";

      }///////////////////////////////////////////////


      //RESET ALARMS RECORDS////////////////////////////////
      param_start = message.indexOf("RESETCOUNTERS");
      param_end = message.indexOf("#");
      if (param_start != -1 && param_end != -1) {

        OpSec = 0;
        OpMin = 0;
        OpHour = 0;
        OpDay = 0;
        MinCurrentP = 0;
        MaxCurrentP = 0;



        SerialBT.println("Statistical records are resetted");
        message = "";

      }///////////////////////////////////////////////

      //SET BMU SERIAL////////////////////////////////
      param_start = message.indexOf("SETBMUSERIAL");
      param_end = message.indexOf("#");
      if (param_start != -1 && param_end != -1) {


        SERIAL_CAL = true;

        FooterEvent = 30;

        SerialBT.println("BMU Serial is resetted");
        message = "";

      }///////////////////////////////////////////////


      //CURRENT CAL////////////////////////////////
      param_start = message.indexOf("CURRENTCAL");
      param_end = message.indexOf("#");
      if (param_start != -1 && param_end != -1) {


        BTProcessor = message.substring(param_start + 10, param_end);
        CalibrationValue = BTProcessor.toFloat();
        Serial.println("Current is calibrated:" + String(CalibrationValue));

        CURRENT_CAL = true;
        FooterEvent = 17;
        SerialBT.println("Current is calibrated");
        message = "";

      }///////////////////////////////////////////////

      //SET cell MODE////////////////////////////////
      param_start = message.indexOf("CELLMOD");
      param_end = message.indexOf("#");
      if (param_start != -1 && param_end != -1) {

        BTProcessor = message.substring(param_start + 7, param_end);
        CellMod = BTProcessor.toInt();
        Serial.println(CellMod);
        boardSetted = false;

        SerialBT.println("cell mode is setted");
        message = "";

      }///////////////////////////////////////////////


      ////////////////////////////////////


      //SET RTC /////////////////////////////////
      param_start = message.indexOf("SETRTC");
      param_end = message.indexOf("#");
      if (param_start != -1 && param_end != -1) {
        BTProcessor = message.substring(param_start + 6, param_end);

        BT_STRING = BTProcessor;
        BT_REMAINING = BTProcessor;
        BT_RTCindex = 0;


        for (int i = 0; i < 6; i++) {
          // Serial.println("------------");
          BT_RTCindex = BT_REMAINING.indexOf('/');
          BT_RTC_ARRAY[i] = BT_REMAINING.substring(0, BT_RTCindex).toInt();
          BT_REMAINING = BT_REMAINING.substring(BT_RTCindex + 1);
        }

        SerialBT.println(String(BT_RTC_ARRAY[0]) + "/" + String(BT_RTC_ARRAY[1]) + "/" + String(BT_RTC_ARRAY[2]) + "/" + String(BT_RTC_ARRAY[3]) + "/" + String(BT_RTC_ARRAY[4]) + "/" + String(BT_RTC_ARRAY[5]));
        yil = BT_RTC_ARRAY[0];
        ayUInt = BT_RTC_ARRAY[1];
        gunUInt = BT_RTC_ARRAY[2];
        saatUInt = BT_RTC_ARRAY[3];
        dakikaUInt = BT_RTC_ARRAY[4];
        saniyeUInt = BT_RTC_ARRAY[5];
        RTCCongifured = true;

        message = "";


      }///////////////////////////////////////////////





      //STARTUP TRIGGER////////////////////////////////
      param_start2 = message.indexOf("STARTUP");
      param_end2 = message.indexOf("#");

      if (param_start2 != -1 && param_end2 != -1) {
        SafetyStartUp2 = true;
        message = "";
      }///////////////////////////////////////////////



      //RESET SD CARD/////////////////////////////////

      param_start = message.indexOf("RESETSD");
      param_end = message.indexOf("#");
      if (param_start != -1 && param_end != -1) {
        SerialBT.println("SD CARD RESETTED");
        SDReset = true;
        message = "";
      }///////////////////////////////////////////////


      //READ GPIO/////////////////////////////////
      param_start = message.indexOf("READGPIO");
      param_end = message.indexOf("#");
      if (param_start != -1 && param_end != -1) {
        SerialBT.println(String(GPIO_READ[0]) + "/" + String(GPIO_READ[1]) + "/" + String(GPIO_READ[2]));
        message = "";
      }///////////////////////////////////////////////


      //READ DRYA/////////////////////////////////
      param_start = message.indexOf("GETDRYA");
      param_end = message.indexOf("#");
      if (param_start != -1 && param_end != -1) {
        SerialBT.println(String(DRYA_ARRAY[0]) + "/" + String(DRYA_ARRAY[1]) + "/" + String(DRYA_ARRAY[2]) + "/" + String(DRYA_ARRAY[3]));
        message = "";
      }///////////////////////////////////////////////


      //READ DRYB/////////////////////////////////
      param_start = message.indexOf("GETDRYB");
      param_end = message.indexOf("#");
      if (param_start != -1 && param_end != -1) {
        SerialBT.println(String(DRYB_ARRAY[0]) + "/" + String(DRYB_ARRAY[1]) + "/" + String(DRYB_ARRAY[2]) + "/" + String(DRYB_ARRAY[3]));
        message = "";
      }///////////////////////////////////////////////

      //READ DRYC/////////////////////////////////
      param_start = message.indexOf("GETDRYC");
      param_end = message.indexOf("#");
      if (param_start != -1 && param_end != -1) {
        SerialBT.println(String(DRYC_ARRAY[0]) + "/" + String(DRYC_ARRAY[1]) + "/" + String(DRYC_ARRAY[2]) + "/" + String(DRYC_ARRAY[3]));
        message = "";
      }///////////////////////////////////////////////


      //READ DRYD/////////////////////////////////
      param_start = message.indexOf("GETDRYD");
      param_end = message.indexOf("#");
      if (param_start != -1 && param_end != -1) {
        SerialBT.println(String(DRYD_ARRAY[0]) + "/" + String(DRYD_ARRAY[1]) + "/" + String(DRYD_ARRAY[2]) + "/" + String(DRYD_ARRAY[3]));
        message = "";
      }///////////////////////////////////////////////




      //SET DRYA /////////////////////////////////
      param_start = message.indexOf("SETDRYA");
      param_end = message.indexOf("#");
      if (param_start != -1 && param_end != -1) {
        BTProcessor = message.substring(param_start + 7, param_end);

        BT_STRING = BTProcessor;
        BT_REMAINING = BTProcessor;
        BT_Dryindex = 0;
        BT_DRY_ARRAY[4];

        for (int i = 0; i < 4; i++) {
          // Serial.println("------------");
          BT_Dryindex = BT_REMAINING.indexOf('/');
          BT_DRY_ARRAY[i] = BT_REMAINING.substring(0, BT_Dryindex);
          BT_REMAINING = BT_REMAINING.substring(BT_Dryindex + 1);
        }
        SerialBT.println(BT_DRY_ARRAY[0] + "/" + BT_DRY_ARRAY[1] + "/" + BT_DRY_ARRAY[2] + "/" + BT_DRY_ARRAY[3]);

        DryString = String(BT_DRY_ARRAY[0]) + "/" + String(BT_DRY_ARRAY[1]) + "/" + String(BT_DRY_ARRAY[2]) + "/" + String(BT_DRY_ARRAY[3]) + "/" + String(DRY_VALUES[0]);
        Serial.println(DryString);
        EEPROMbusy = true;
        preferences.begin("my-app", false);
        preferences.putString("DRYA", DryString);
        DRYA_ARRAY[0] = BT_DRY_ARRAY[0];
        DRYA_ARRAY[1] = BT_DRY_ARRAY[1];
        DRYA_ARRAY[2] = BT_DRY_ARRAY[2];
        DRYA_ARRAY[3] = BT_DRY_ARRAY[3];
        preferences.end();
        EEPROMbusy = false;
        message = "";
      }///////////////////////////////////////////////




      //SET DRYB /////////////////////////////////
      param_start = message.indexOf("SETDRYB");
      param_end = message.indexOf("#");
      if (param_start != -1 && param_end != -1) {
        BTProcessor = message.substring(param_start + 7, param_end);

        BT_STRING = BTProcessor;
        BT_REMAINING = BTProcessor;
        BT_Dryindex = 0;
        BT_DRY_ARRAY[4];

        for (int i = 0; i < 4; i++) {
          // Serial.println("------------");
          BT_Dryindex = BT_REMAINING.indexOf('/');
          BT_DRY_ARRAY[i] = BT_REMAINING.substring(0, BT_Dryindex);
          BT_REMAINING = BT_REMAINING.substring(BT_Dryindex + 1);
        }
        SerialBT.println(BT_DRY_ARRAY[0] + "/" + BT_DRY_ARRAY[1] + "/" + BT_DRY_ARRAY[2] + "/" + BT_DRY_ARRAY[3]);

        String DryString = String(BT_DRY_ARRAY[0]) + "/" + String(BT_DRY_ARRAY[1]) + "/" + String(BT_DRY_ARRAY[2]) + "/" + String(BT_DRY_ARRAY[3]) + "/" + String(DRY_VALUES[0]);
        Serial.println(DryString);
        EEPROMbusy = true;
        preferences.begin("my-app", false);
        preferences.putString("DRYB", DryString);
        DRYB_ARRAY[0] = BT_DRY_ARRAY[0];
        DRYB_ARRAY[1] = BT_DRY_ARRAY[1];
        DRYB_ARRAY[2] = BT_DRY_ARRAY[2];
        DRYB_ARRAY[3] = BT_DRY_ARRAY[3];
        preferences.end();
        EEPROMbusy = false;
        message = "";
      }///////////////////////////////////////////////



      //SET DRYC /////////////////////////////////
      param_start = message.indexOf("SETDRYC");
      param_end = message.indexOf("#");
      if (param_start != -1 && param_end != -1) {
        BTProcessor = message.substring(param_start + 7, param_end);

        BT_STRING = BTProcessor;
        BT_REMAINING = BTProcessor;
        BT_Dryindex = 0;
        BT_DRY_ARRAY[4];

        for (int i = 0; i < 4; i++) {
          // Serial.println("------------");
          BT_Dryindex = BT_REMAINING.indexOf('/');
          BT_DRY_ARRAY[i] = BT_REMAINING.substring(0, BT_Dryindex);
          BT_REMAINING = BT_REMAINING.substring(BT_Dryindex + 1);
        }
        SerialBT.println(BT_DRY_ARRAY[0] + "/" + BT_DRY_ARRAY[1] + "/" + BT_DRY_ARRAY[2] + "/" + BT_DRY_ARRAY[3]);

        String DryString = String(BT_DRY_ARRAY[0]) + "/" + String(BT_DRY_ARRAY[1]) + "/" + String(BT_DRY_ARRAY[2]) + "/" + String(BT_DRY_ARRAY[3]) + "/" + String(DRY_VALUES[0]);
        Serial.println(DryString);
        EEPROMbusy = true;
        preferences.begin("my-app", false);
        preferences.putString("DRYC", DryString);
        DRYC_ARRAY[0] = BT_DRY_ARRAY[0];
        DRYC_ARRAY[1] = BT_DRY_ARRAY[1];
        DRYC_ARRAY[2] = BT_DRY_ARRAY[2];
        DRYC_ARRAY[3] = BT_DRY_ARRAY[3];
        preferences.end();
        EEPROMbusy = false;
        message = "";
      }///////////////////////////////////////////////





      //SET DRYD /////////////////////////////////
      param_start = message.indexOf("SETDRYD");
      param_end = message.indexOf("#");
      if (param_start != -1 && param_end != -1) {
        BTProcessor = message.substring(param_start + 7, param_end);

        BT_STRING = BTProcessor;
        BT_REMAINING = BTProcessor;
        BT_Dryindex = 0;
        BT_DRY_ARRAY[4];

        for (int i = 0; i < 4; i++) {
          // Serial.println("------------");
          BT_Dryindex = BT_REMAINING.indexOf('/');
          BT_DRY_ARRAY[i] = BT_REMAINING.substring(0, BT_Dryindex);
          BT_REMAINING = BT_REMAINING.substring(BT_Dryindex + 1);
        }
        SerialBT.println(BT_DRY_ARRAY[0] + "/" + BT_DRY_ARRAY[1] + "/" + BT_DRY_ARRAY[2] + "/" + BT_DRY_ARRAY[3]);

        String DryString = String(BT_DRY_ARRAY[0]) + "/" + String(BT_DRY_ARRAY[1]) + "/" + String(BT_DRY_ARRAY[2]) + "/" + String(BT_DRY_ARRAY[3]) + "/" + String(DRY_VALUES[0]);
        Serial.println(DryString);
        EEPROMbusy = true;
        preferences.begin("my-app", false);
        preferences.putString("DRYD", DryString);
        DRYD_ARRAY[0] = BT_DRY_ARRAY[0];
        DRYD_ARRAY[1] = BT_DRY_ARRAY[1];
        DRYD_ARRAY[2] = BT_DRY_ARRAY[2];
        DRYD_ARRAY[3] = BT_DRY_ARRAY[3];
        preferences.end();
        EEPROMbusy = false;
        message = "";
      }///////////////////////////////////////////////


      //GET FIXED ENERGY/////////////////////////////////
      param_start = message.indexOf("GETENERGY");
      param_end = message.indexOf("#");
      if (param_start != -1 && param_end != -1) {
        SerialBT.print(ChargeEnergy);
        SerialBT.print("/");
        SerialBT.print(DischargeEnergy);
        message = "";
      }///////////////////////////////////////////////


      //reset module/////////////////////////////////
      param_start = message.indexOf("RESETENCAP");
      param_end = message.indexOf("#");
      if (param_start != -1 && param_end != -1) {
        SerialBT.println("ENCAP WILL RESTART");
        delay(1000);

        for (int i = 0; i < 100; i++ )  {
          if (!deleteFilesProgress) {
            ESP.restart();
            break;
          }
          delay(10);
        }

        message = "";
      }///////////////////////////////////////////////



      //PINGING BMS/////////////////////////////////
      param_start = message.indexOf("PING");
      param_end = message.indexOf("#");
      if (param_start != -1 && param_end != -1) {
        SerialBT.println("PONG_BMS");
        pinged = true;
        SleepCounter = 1;
        message = "";
      }///////////////////////////////////////////////




      //SET SERIAL BMS////////////////////////////////
      //EN-02-048V0-0-3-3-0-0-23-A-00001
      param_start = message.indexOf("SETSR");
      param_end = message.indexOf("#");
      if (param_start != -1 && param_end != -1) {
        BTProcessor = message.substring(param_start + 5, param_end);
        Serial.println(BTProcessor);
        SerialNumber = BTProcessor;
        SerialBT.println(SerialNumber);
        Serial.println(SerialNumber);
        message = "";
        EEPROMbusy = true;
        preferences.begin("my-app", false);
        preferences.putString("SerialNumber", SerialNumber);
        preferences.end();
        EEPROMbusy = false;
        SERIAL_CAL = true;
      }///////////////////////////////////////////////


      //SET time OFFSET////////////////////////////////

      param_start = message.indexOf("SETOFFSET");
      param_end = message.indexOf("#");
      if (param_start != -1 && param_end != -1) {
        BTProcessor = message.substring(param_start + 9, param_end);
        SerialBT.println(BTProcessor);
        message = "";
        EEPROMbusy = true;
        timeoffset = BTProcessor.toInt();

        EEPROMbusy = true;
        preferences.begin("my-app", false);
        delay(10);
        preferences.putInt("timeoffset", timeoffset);
        delay(10);
        preferences.end();
        EEPROMbusy = false;




        RTCFetched = false;
        EEPROMbusy = false;
        Serial.println(BTProcessor.toInt() - 3600);


      }///////////////////////////////////////////////


      //SET CAP BMS////////////////////////////////
      param_start = message.indexOf("SETCAP");
      param_end = message.indexOf("#");
      if (param_start != -1 && param_end != -1) {
        BTProcessor = message.substring(param_start + 6, param_end);
        SystemCAP = BTProcessor.toInt();
        SerialBT.println(SystemCAP);
        RC_RV = true;
        RV = 0;
        message = "";

      }///////////////////////////////////////////////



      //SET SLEEP BMS////////////////////////////////
      param_start = message.indexOf("SETSLP");
      param_end = message.indexOf("#");
      if (param_start != -1 && param_end != -1) {
        BTProcessor = message.substring(param_start + 6, param_end);
        SLP = BTProcessor.toInt();
        SerialBT.println(SLP);
        SLP_CAL = true;
        //Serial.println("Sleep command is given");
        //Serial.println(SLP_CAL);
        message = "";

      }///////////////////////////////////////////////




      //SET SOC BMS////////////////////////////////
      param_start = message.indexOf("SETSOC");
      param_end = message.indexOf("#");
      if (param_start != -1 && param_end != -1) {
        BTProcessor = message.substring(param_start + 6, param_end);
        SystemSOC = BTProcessor.toInt();
        SerialBT.println(SystemSOC);
        SOC_CAL = true;
        message = "";

      }///////////////////////////////////////////////





      //SET CVD BMS////////////////////////////////
      param_start = message.indexOf("SETCVD");
      param_end = message.indexOf("#");
      if (param_start != -1 && param_end != -1) {
        BTProcessor = message.substring(param_start + 6, param_end);
        CVD = BTProcessor.toFloat();
        SerialBT.println(CVD);
        CVD_CAL = true;
        message = "";

      }///////////////////////////////////////////////





      //SET BV_BD BMS////////////////////////////////
      param_start2 = message.indexOf("SETBV");
      param_end2 = message.indexOf("/");
      if (param_start2 != -1 && param_end2 != -1) {
        BTProcessor2 = message.substring(param_start2 + 5, param_end2);
        BV = BTProcessor2.toFloat();
        //SerialBT.println(HCC);

      }///////////////////////////////////////////////

      param_start = message.indexOf("SETBD");
      param_end = message.indexOf("#");
      if (param_start != -1 && param_end != -1) {
        BTProcessor = message.substring(param_start + 5, param_end);
        BD = BTProcessor.toFloat();
        message = "";
        SerialBT.print(BV);
        SerialBT.print("/");
        SerialBT.println(BD);
        BV_BD = true;
      }///////////////////////////////////////////////



      //SET SOC_ALARM BMS////////////////////////////////
      param_start2 = message.indexOf("SETSHI");
      param_end2 = message.indexOf("/");
      if (param_start2 != -1 && param_end2 != -1) {
        BTProcessor2 = message.substring(param_start2 + 6, param_end2);
        SOCHI = BTProcessor2.toFloat();
        //SerialBT.println(HCC);

      }///////////////////////////////////////////////

      param_start = message.indexOf("SETSLOW");
      param_end = message.indexOf("#");
      if (param_start != -1 && param_end != -1) {
        BTProcessor = message.substring(param_start + 7, param_end);
        SOCLOW = BTProcessor.toFloat();

        Serial.println("SOCALARM");
        Serial.println(SOCLOW);
        Serial.println(SOCHI);
        message = "";
        SerialBT.print(SOCHI);
        SerialBT.print("/");
        SerialBT.println(SOCLOW);
        SOC_ALARM = true;
      }///////////////////////////////////////////////

      //SET SUM_CALL BMS////////////////////////////////
      param_start2 = message.indexOf("SETSUMHI");
      param_end2 = message.indexOf("/");
      if (param_start2 != -1 && param_end2 != -1) {
        BTProcessor2 = message.substring(param_start2 + 8, param_end2);
        SUMHI = BTProcessor2.toFloat();
        //SerialBT.println(HCC);

      }///////////////////////////////////////////////

      param_start = message.indexOf("SETSUMLOW");
      param_end = message.indexOf("#");
      if (param_start != -1 && param_end != -1) {
        BTProcessor = message.substring(param_start + 9, param_end);
        SUMLOW = BTProcessor.toFloat();
        message = "";
        SerialBT.print(SUMHI);
        SerialBT.print("/");
        SerialBT.println(SUMLOW);
        SUM_CAL = true;
      }///////////////////////////////////////////////


      //SET HCC&HCD BMS////////////////////////////////
      param_start2 = message.indexOf("SETHCC");
      param_end2 = message.indexOf("/");
      if (param_start2 != -1 && param_end2 != -1) {
        BTProcessor2 = message.substring(param_start2 + 6, param_end2);
        HCC = BTProcessor2.toInt();
        //SerialBT.println(HCC);

      }///////////////////////////////////////////////

      param_start = message.indexOf("SETHCD");
      param_end = message.indexOf("#");
      if (param_start != -1 && param_end != -1) {
        BTProcessor = message.substring(param_start + 6, param_end);
        HCD = BTProcessor.toInt();
        message = "";
        SerialBT.print(HCC);
        SerialBT.print("/");
        SerialBT.println(HCD);
        HCC_HCD = true;
      }///////////////////////////////////////////////


      //SET HVC_LVC BMS////////////////////////////////
      param_start2 = message.indexOf("SETHVC");
      param_end2 = message.indexOf("/");
      if (param_start2 != -1 && param_end2 != -1) {
        BTProcessor2 = message.substring(param_start2 + 6, param_end2);
        HVC = BTProcessor2.toFloat();
        //SerialBT.println(HCC);

      }///////////////////////////////////////////////


      param_start = message.indexOf("SETLVC");
      param_end = message.indexOf("#");
      if (param_start != -1 && param_end != -1) {
        BTProcessor = message.substring(param_start + 6, param_end);
        LVC = BTProcessor.toFloat();
        message = "";
        SerialBT.print(HVC);
        SerialBT.print("/");
        SerialBT.println(LVC);
        HVC_LVC = true;
      }///////////////////////////////////////////////





      ///////////////////////////////////////////
      //SET HTC_HTD BMS////////////////////////////////
      param_start2 = message.indexOf("SETHTC");
      param_end2 = message.indexOf("/");
      if (param_start2 != -1 && param_end2 != -1) {
        BTProcessor2 = message.substring(param_start2 + 6, param_end2);
        HTC = BTProcessor2.toFloat();
        //SerialBT.println(HCC);

      }///////////////////////////////////////////////


      param_start = message.indexOf("SETHTD");
      param_end = message.indexOf("#");
      if (param_start != -1 && param_end != -1) {
        BTProcessor = message.substring(param_start + 6, param_end);
        HTD = BTProcessor.toFloat();
        message = "";
        SerialBT.print(HTC);
        SerialBT.print("/");
        SerialBT.println(HTD);
        HTC_HTD = true;
      }///////////////////////////////////////////////



      //GET SERIAL BMS////////////////////////////////
      //EN-02-048V0-0-3-3-0-0-23-A-00001
      param_start = message.indexOf("GETSR");
      param_end = message.indexOf("#");
      if (param_start != -1 && param_end != -1) {
        SerialBT.println(SerialNumber);
        Serial.println("Serial number is requested");
        message = "";
      }///////////////////////////////////////////////

      //GET FIRMWARE BMS////////////////////////////////
      param_start = message.indexOf("GETFW");
      param_end = message.indexOf("#");
      if (param_start != -1 && param_end != -1) {
        SerialBT.println(FirmwareVer);
        message = "";
      }///////////////////////////////////////////////

      //GET MEAS1 BMS////////////////////////////////
      param_start = message.indexOf("GET_MEAS1");
      param_end = message.indexOf("#");
      if (param_start != -1 && param_end != -1) {

        Serial.println("Measurements are requested");


        String MEAS1 = "TV" + String(BMS.sum_voltage * 0.1) +  "/TC" + String((BMS.current - 30000) * 0.1) + "/SOC" +  String(BMS.SOC * 0.1) + "/TEMP" +  String(BMS.max_cell_temp - 40) +
                       "/MAX" + String(BMS.max_cell_volt * 0.001) + "/MIN" + String(BMS. min_cell_volt * 0.001) +  "/STT" + String(BMS.state) +
                       "/CH" + String(BMS.charge) + "/DSH" + String(BMS.discharge) + "/LF" + String(BMS.bms_life) +
                       "/RC" + String(BMS.rem_cap);

        SerialBT.println(MEAS1);
        message = "";
      }///////////////////////////////////////////////

      //GET MEAS2 BMS////////////////////////////////
      param_start = message.indexOf("GET_MEAS2");
      param_end = message.indexOf("#");
      if (param_start != -1 && param_end != -1) {

        Serial.println("Measurements are requested");



        String MEAS2 = "/BL" + String(balancerState) + "/CHE" + String(ChargeEnergy, 3) + "/DHE" + String(DischargeEnergy) + "/TM" + TimeString + "/DT" + DateString +
                       "/FW" + FirmwareVer + "/DA" + String(DRY_VALUES[0]) + "/DB" + String(DRY_VALUES[1]) + "/DC" + String(DRY_VALUES[2]) + "/DD" + String(DRY_VALUES[3]) + "/SF" + String(Safety) +
                       "/CELLTEMP1" + String(CellTemp1) + "/CELLTEMP2" + String(CellTemp2) + "/SD" + String(control_sd) + "/CR" + String(canbusReady) + "#";

        SerialBT.println(MEAS2);
        message = "";
      }///////////////////////////////////////////////


      //GET CELLS BMS////////////////////////////////
      param_start = message.indexOf("GET_CELLS");
      param_end = message.indexOf("#");
      if (param_start != -1 && param_end != -1) {

        String CellString = "";
        for (int i = 0; i < ReadCellMod; i++) {
          CellString = CellString + String(BMS.cell_voltage[i] * 0.001) + "/";
        }

        CellString = "CELLS" + CellString + "#";
        SerialBT.println(CellString);
        message = "";
      }///////////////////////////////////////////////


      //GET ALARMS BMS////////////////////////////////
      param_start = message.indexOf("GET_ALARMS");
      param_end = message.indexOf("#");
      if (param_start != -1 && param_end != -1) {

        String AlarmsString = "";
        for (int i = 0; i < 28; i++) {
          AlarmsString = AlarmsString + String(BMS.error[i]) + "/";
        }
        AlarmsString = "ALARMS" + AlarmsString + "#";
        SerialBT.println(AlarmsString);
        message = "";
      }///////////////////////////////////////////////




      //GET LIMITS BMS////////////////////////////////
      param_start = message.indexOf("GET_LIMITS");
      param_end = message.indexOf("#");
      if (param_start != -1 && param_end != -1) {
        //ReadAllParameters = true;

        String LIMITS = "DSCHI" + String(BMS.dischar_curr2) + "/" + "CHCHI" + String(BMS.charge_curr2) + "/" + "SUMVHI" + String(BMS.sumv_high2 * 0.1) + "/" +
                        "SUMVLO" + String(BMS.sumv_low2 * 0.1) + "/" + "CVHI" + String(BMS.cell_volthigh2 * 0.001) + "/" + "CVLO" + String(BMS.cell_voltlow2 * 0.001) + "/" +
                        "BV" + String(BMS.balance_volt * 0.001) + "/" + "BDV" + String(BMS.balance_volt_diff) + "/" + "CTHI" + String(BMS.charge_temp_high2) + "/" +
                        "CTLO" + String(BMS.charge_temp_low2) + "/" + "DTHI" + String(BMS.discharge_temp_high2) + "/" + "DTLO" + String(BMS.discharge_temp_low2) + "/" +
                        "VDF" + String(BMS.volt_diff2)  +
                        "/" + "RTVOLT" + String(BMS.rated_volt * 0.001) + "/" +
                        "SOCHI" + String(BMS.SOC_high2) + "/" +  "SOCLO" + String(BMS.SOC_low2) + "/" +  "VOLTDIF" + String(BMS.volt_diff2) + "/" +  "SYSCAP" + String(BMS.rem_cap * 0.001 * 0.001 * 48) + "/" +
                        "SLPTM" + String(BMS.secondsleepbm) +   "/";

        //BMS.rated_cap



        LIMITS = "LIMITS" + LIMITS + "#";
        SerialBT.println(LIMITS);
        message = "";
      }///////////////////////////////////////////////

    }

    delay(100);
  }
}

void BMS_COMM_CODE( void * pvParameters ) {
  unsigned long previousMillisBMS = 0;
  unsigned long intervalBMS = 100;
  unsigned long currentMillisBMS = 0;


  //Serial.println("Preffered Language:" + LanguageArray[indexLanguageArray]);

  float periodBMS;
  //Serial.println("BMS Started");
  //ReadAllParameters = true;
  for (;;) {




    if (readCANID == false) {
      readCANID = true;
      for (int i = 0; i < 10; i++) {
        delay(1);

        BMS_recieve(0x65);
        delay(1);
        // Serial.println("CAN ID:" + String(BMS.board_number));
      }
      delay(1);
    }





    currentMillisBMS = millis();
    if ((currentMillisBMS - previousMillisBMS) >= intervalBMS && (!HCC_HCD) && (!SLP_CAL) && (!HVC_LVC)  && (!HTC_HTD) && (!RC_RV) && (!pinged)
        && (!BV_BD) && (!SOC_CAL) && (!SUM_CAL) && (!SOC_ALARM) && (!CHARGE_SET) && (!DISCHARGE_SET) && (!BALANCE_SET)
        && (!CVD_CAL) && (!SafetyTurnOnApproved) && (!SafetyTurnOffApproved) && (!CURRENT_CAL) && (!SERIAL_CAL) && (!CURRENT_ZERO_CAL) && (!SetCANID) && (!SetEnergies) && (!SetCOMM))
    {
      float period = (currentMillisBMS - previousMillisBMS) * 0.001;


      esp_task_wdt_init(60, false);
      //      if (RTCCongifured == true) {
      //        RTCCongifured = false;
      //        rtc_bms_begin(yil, ayUInt, gunUInt, saatUInt, dakikaUInt, saniyeUInt);
      //        // Serial.println(String(BMS.day) + "/" + String(BMS.month) + "/" + String(BMS.year));
      //        // Serial.println(String(BMS.when) + "/" + String(BMS.point));
      //      }


      //      if (!buzzerActive) {
      //        get_time();
      //      }


      if (String(BMS.when).length() < 2) {
        calibratedHour = "0" + String(BMS.when);
      }
      else {
        calibratedHour = String(BMS.when);
      }

      if (String(BMS.point).length() < 2) {
        calibratedMinute = "0" + String(BMS.point);
      }
      else {
        calibratedMinute = String(BMS.point);
      }

      if (String(BMS.second).length() < 2) {
        calibratedSecond = "0" + String(BMS.second);
      }
      else {
        calibratedSecond = String(BMS.second);
      }





      TimeString = calibratedHour + ":" + calibratedMinute;
      DateString = String(BMS.day) + "/" + String(BMS.month) + "/" + String(BMS.year);



      //SERVICE DETECTION
      if (BMS.current != 0) {
        if (((BMS.current - 30000) * 0.1 > 2 && !BMS.charge) || ((BMS.current - 30000) * 0.1 < -2 && !BMS.discharge)) {

          service = true;
          // Serial.println("Please contact service");
        }
        else {
          service = false;
        }
      }





      //recovery


      bool RecoveryState = ((BMS.current - 30000) * 0.1 == 0 || (BMS.current - 30000) * 0.1 == -3000) && (!BMS.discharge || !BMS.charge) && !Safety;
      //   Serial.println("Recovery State:" + String(RecoveryState));
      if (RecoveryState) {
        //Serial.println("Recovery Counter:" + String(RecoveryCounter));
        RecoveryCounter++;
        if (RecoveryCounter > 90) {
          RecoveryCounter = 0;
        }
      }
      else {
        RecoveryCounter = 0;
      }









      ////////////////////////////CHECKING DRY CONTACTS
      //DRY_VALUES


      switch (DRYA_ARRAY[0].toInt()) {  //DRY CONTACT A

        case 0:  // Terminal Voltage Rule
          if (DRYA_ARRAY[1].toInt() == 0) { //<=

            if (BMS.sum_voltage * 0.1 <= DRYA_ARRAY[2].toFloat()) {
              // Serial.println("Dry contact A  voltageis enabled ");
              DRY_VALUES[0] = 1;
            }
            else if (BMS.sum_voltage * 0.1 > DRYA_ARRAY[3].toFloat()) {
              //  Serial.println("Dry contact A  voltageis disabled ");
              DRY_VALUES[0] = 0;
            }
          }
          else {//>
            if (BMS.sum_voltage * 0.1 > DRYA_ARRAY[2].toFloat()) {
              //  Serial.println("Dry contact A voltage is enabled ");
              DRY_VALUES[0] = 1;
            }
            else if (BMS.sum_voltage * 0.1 <= DRYA_ARRAY[3].toFloat()) {
              //Serial.println("Dry contact A voltage is disabled ");
              DRY_VALUES[0] = 0;
            }
          }
          break;

        case 1:  // Current
          if (DRYA_ARRAY[1].toInt() == 0) { //<=

            if (abs((BMS.current - 30000 ) * 0.1) <= DRYA_ARRAY[2].toFloat()) {
              // Serial.println("Dry contact max volt A is enabled ");
              DRY_VALUES[0] = 1;
            }
            else if (abs((BMS.current - 30000) * 0.1) > DRYA_ARRAY[3].toFloat()) {
              //  Serial.println("Dry contact max volt A is disabled ");
              DRY_VALUES[0] = 0;
            }
          }
          else {//>
            if (abs((BMS.current - 30000) * 0.1) > DRYA_ARRAY[2].toFloat()) {
              // Serial.println("Dry contact max volt A is enabled ");
              DRY_VALUES[0] = 1;
            }
            else if (abs((BMS.current - 30000) * 0.1) <= DRYA_ARRAY[3].toFloat()) {
              // Serial.println("Dry contact max volt A is disabled ");
              DRY_VALUES[0] = 0;
            }
          }
          break;


        case 2:  // temperature
          if (DRYA_ARRAY[1].toInt() == 0) { //<=

            if (BMS.max_cell_temp - 40 <= DRYA_ARRAY[2].toFloat()) {
              // Serial.println("Dry contact A is enabled ");
              DRY_VALUES[0] = 1;
            }
            else if (BMS.max_cell_temp - 40 > DRYA_ARRAY[3].toFloat()) {
              //   Serial.println("Dry contact A is disabled ");
              DRY_VALUES[0] = 0;
            }
          }
          else {//>
            if (BMS.max_cell_temp - 40 > DRYA_ARRAY[2].toFloat()) {
              //   Serial.println("Dry contact A is enabled ");
              DRY_VALUES[0] = 1;
            }
            else if (BMS.max_cell_temp - 40 <= DRYA_ARRAY[3].toFloat()) {
              //  Serial.println("Dry contact A is disabled ");
              DRY_VALUES[0] = 0;
            }
          }
          break;

        case 3:  // soc
          if (DRYA_ARRAY[1].toInt() == 0) { //<=

            if (BMS.SOC * 0.1 <= DRYA_ARRAY[2].toFloat()) {
              //  Serial.println("Dry contact A is enabled ");
              DRY_VALUES[0] = 1;
            }
            else if (BMS.SOC * 0.1 > DRYA_ARRAY[3].toFloat()) {
              //  Serial.println("Dry contact A is disabled ");
              DRY_VALUES[0] = 0;
            }
          }
          else {//>
            if (BMS.SOC * 0.1 > DRYA_ARRAY[2].toFloat()) {
              // Serial.println("Dry contact A is enabled ");
              DRY_VALUES[0] = 1;
            }
            else if (BMS.SOC * 0.1 <= DRYA_ARRAY[3].toFloat()) {
              //   Serial.println("Dry contact A is disabled ");
              DRY_VALUES[0] = 0;
            }
          }
          break;

        case 4:
          DRY_VALUES[0] = 0;
          break;

        case 5:
          DRY_VALUES[0] = 1;
          break;

        case 6:
          if (SleepCounter < 57) {
            DRY_VALUES[0] = 1;
          }
          if (SleepCounter > 58) {
            DRY_VALUES[0] = 0;
          }
          break;

        case 7:
          DRY_VALUES[0] = !Safety;
          break;


        case 8:  // CELL-DIFF
          if (DRYA_ARRAY[1].toInt() == 0) { //<=

            if (BMS.max_cell_volt  - BMS.min_cell_volt  <= DRYA_ARRAY[2].toFloat()) {
              // Serial.println("Dry contact A is enabled ");
              DRY_VALUES[0] = 1;
            }
            else if (BMS.max_cell_volt  - BMS.min_cell_volt  > DRYA_ARRAY[3].toFloat()) {
              //   Serial.println("Dry contact A is disabled ");
              DRY_VALUES[0] = 0;
            }
          }
          else {//>
            if (BMS.max_cell_volt  - BMS.min_cell_volt  > DRYA_ARRAY[2].toFloat()) {
              //   Serial.println("Dry contact A is enabled ");
              DRY_VALUES[0] = 1;
            }
            else if (BMS.max_cell_volt  - BMS.min_cell_volt  <= DRYA_ARRAY[3].toFloat()) {
              //  Serial.println("Dry contact A is disabled ");
              DRY_VALUES[0] = 0;
            }
          }
          break;





        default:
          break;
      }

      ////////////////////


      switch (DRYB_ARRAY[0].toInt()) {  //DRY CONTACT B

        case 0:  // Terminal Voltage Rule
          if (DRYB_ARRAY[1].toInt() == 0) { //<=

            if (BMS.sum_voltage * 0.1 <= DRYB_ARRAY[2].toFloat()) {
              // Serial.println("Dry contact A  voltageis enabled ");
              DRY_VALUES[1] = 1;
            }
            else if (BMS.sum_voltage * 0.1 > DRYB_ARRAY[3].toFloat()) {
              //  Serial.println("Dry contact A  voltageis disabled ");
              DRY_VALUES[1] = 0;
            }
          }
          else {//>
            if (BMS.sum_voltage * 0.1 > DRYB_ARRAY[2].toFloat()) {
              //  Serial.println("Dry contact A voltage is enabled ");
              DRY_VALUES[1] = 1;
            }
            else if (BMS.sum_voltage * 0.1 <= DRYB_ARRAY[3].toFloat()) {
              //Serial.println("Dry contact A voltage is disabled ");
              DRY_VALUES[1] = 0;
            }
          }
          break;

        case 1:  // Current
          if (DRYB_ARRAY[1].toInt() == 0) { //<=

            if (abs((BMS.current - 30000 ) * 0.1) <= DRYB_ARRAY[2].toFloat()) {
              // Serial.println("Dry contact max volt A is enabled ");
              DRY_VALUES[1] = 1;
            }
            else if (abs((BMS.current - 30000) * 0.1) > DRYB_ARRAY[3].toFloat()) {
              //  Serial.println("Dry contact max volt A is disabled ");
              DRY_VALUES[1] = 0;
            }
          }
          else {//>
            if (abs((BMS.current - 30000) * 0.1) > DRYB_ARRAY[2].toFloat()) {
              // Serial.println("Dry contact max volt A is enabled ");
              DRY_VALUES[1] = 1;
            }
            else if (abs((BMS.current - 30000) * 0.1) <= DRYB_ARRAY[3].toFloat()) {
              // Serial.println("Dry contact max volt A is disabled ");
              DRY_VALUES[1] = 0;
            }
          }
          break;


        case 2:  // temperature
          if (DRYB_ARRAY[1].toInt() == 0) { //<=

            if (BMS.max_cell_temp - 40 <= DRYB_ARRAY[2].toFloat()) {
              // Serial.println("Dry contact A is enabled ");
              DRY_VALUES[1] = 1;
            }
            else if (BMS.max_cell_temp - 40 > DRYB_ARRAY[3].toFloat()) {
              //   Serial.println("Dry contact A is disabled ");
              DRY_VALUES[1] = 0;
            }
          }
          else {//>
            if (BMS.max_cell_temp - 40 > DRYB_ARRAY[2].toFloat()) {
              //   Serial.println("Dry contact A is enabled ");
              DRY_VALUES[1] = 1;
            }
            else if (BMS.max_cell_temp - 40 <= DRYB_ARRAY[3].toFloat()) {
              //  Serial.println("Dry contact A is disabled ");
              DRY_VALUES[1] = 0;
            }
          }
          break;

        case 3:  // soc
          if (DRYB_ARRAY[1].toInt() == 0) { //<=

            Serial.println("DRY B has  <=soc RULE-------------------------------------------");

            if (BMS.SOC * 0.1 <= DRYB_ARRAY[2].toFloat()) {
              //  Serial.println("Dry contact A is enabled ");
              DRY_VALUES[1] = 1;
            }
            else if (BMS.SOC * 0.1 > DRYB_ARRAY[3].toFloat()) {
              //  Serial.println("Dry contact A is disabled ");
              DRY_VALUES[1] = 0;
            }
          }
          else {//>
            if (BMS.SOC * 0.1 > DRYB_ARRAY[2].toFloat()) {
              // Serial.println("Dry contact A is enabled ");
              DRY_VALUES[1] = 1;
            }
            else if (BMS.SOC * 0.1 <= DRYB_ARRAY[3].toFloat()) {
              //   Serial.println("Dry contact A is disabled ");
              DRY_VALUES[1] = 0;
            }
          }
          break;

        case 4:
          DRY_VALUES[1] = 0;
          break;

        case 5:
          DRY_VALUES[1] = 1;
          break;

        case 6:
          if (SleepCounter < 57) {
            DRY_VALUES[1] = 1;
          }
          if (SleepCounter > 58) {
            DRY_VALUES[1] = 0;
          }
          break;
        case 7:
          DRY_VALUES[1] = !Safety;
          break;



        case 8:  // CELL-DIFF
          if (DRYA_ARRAY[1].toInt() == 0) { //<=

            if (BMS.max_cell_volt  - BMS.min_cell_volt <= DRYA_ARRAY[2].toFloat()) {
              // Serial.println("Dry contact A is enabled ");
              DRY_VALUES[1] = 1;
            }
            else if (BMS.max_cell_volt  - BMS.min_cell_volt  > DRYA_ARRAY[3].toFloat()) {
              //   Serial.println("Dry contact A is disabled ");
              DRY_VALUES[1] = 0;
            }
          }
          else {//>
            if (BMS.max_cell_volt  - BMS.min_cell_volt  > DRYA_ARRAY[2].toFloat()) {
              //   Serial.println("Dry contact A is enabled ");
              DRY_VALUES[1] = 1;
            }
            else if (BMS.max_cell_volt  - BMS.min_cell_volt <= DRYA_ARRAY[3].toFloat()) {
              //  Serial.println("Dry contact A is disabled ");
              DRY_VALUES[1] = 0;
            }
          }
          break;

        default:
          break;
      }

      ////////////////////

      switch (DRYC_ARRAY[0].toInt()) {  //DRY CONTACT C

        case 0:  // Terminal Voltage Rule
          if (DRYC_ARRAY[1].toInt() == 0) { //<=

            if (BMS.sum_voltage * 0.1 <= DRYC_ARRAY[2].toFloat()) {
              // Serial.println("Dry contact A  voltageis enabled ");
              DRY_VALUES[2] = 1;
            }
            else if (BMS.sum_voltage * 0.1 > DRYC_ARRAY[3].toFloat()) {
              //  Serial.println("Dry contact A  voltageis disabled ");
              DRY_VALUES[2] = 0;
            }
          }
          else {//>
            if (BMS.sum_voltage * 0.1 > DRYC_ARRAY[2].toFloat()) {
              //  Serial.println("Dry contact A voltage is enabled ");
              DRY_VALUES[2] = 1;
            }
            else if (BMS.sum_voltage * 0.1 <= DRYC_ARRAY[3].toFloat()) {
              //Serial.println("Dry contact A voltage is disabled ");
              DRY_VALUES[2] = 0;
            }
          }
          break;

        case 1:  // Current
          if (DRYC_ARRAY[1].toInt() == 0) { //<=

            if (abs((BMS.current - 30000 ) * 0.1) <= DRYC_ARRAY[2].toFloat()) {
              Serial.println("Dry contact C high current enabled ");
              DRY_VALUES[2] = 1;
            }
            else if (abs((BMS.current - 30000) * 0.1) > DRYC_ARRAY[3].toFloat()) {
              Serial.println("Dry contact C high current disabled ");
              DRY_VALUES[2] = 0;
            }
          }
          else {//>
            if (abs((BMS.current - 30000) * 0.1) > DRYC_ARRAY[2].toFloat()) {
              // Serial.println("Dry contact max volt A is enabled ");
              DRY_VALUES[2] = 1;
            }
            else if (abs((BMS.current - 30000) * 0.1) <= DRYC_ARRAY[3].toFloat()) {
              // Serial.println("Dry contact max volt A is disabled ");
              DRY_VALUES[2] = 0;
            }
          }
          break;


        case 2:  // temperature
          if (DRYC_ARRAY[1].toInt() == 0) { //<=

            if (BMS.max_cell_temp - 40 <= DRYC_ARRAY[2].toFloat()) {
              // Serial.println("Dry contact A is enabled ");
              DRY_VALUES[2] = 1;
            }
            else if (BMS.max_cell_temp - 40 > DRYC_ARRAY[3].toFloat()) {
              //   Serial.println("Dry contact A is disabled ");
              DRY_VALUES[2] = 0;
            }
          }
          else {//>
            if (BMS.max_cell_temp - 40 > DRYC_ARRAY[2].toFloat()) {
              //   Serial.println("Dry contact A is enabled ");
              DRY_VALUES[2] = 1;
            }
            else if (BMS.max_cell_temp - 40 <= DRYC_ARRAY[3].toFloat()) {
              //  Serial.println("Dry contact A is disabled ");
              DRY_VALUES[2] = 0;
            }
          }
          break;

        case 3:  // soc
          if (DRYC_ARRAY[1].toInt() == 0) { //<=

            if (BMS.SOC * 0.1 <= DRYC_ARRAY[2].toFloat()) {
              //  Serial.println("Dry contact A is enabled ");
              DRY_VALUES[2] = 1;
            }
            else if (BMS.SOC * 0.1 > DRYC_ARRAY[3].toFloat()) {
              //  Serial.println("Dry contact A is disabled ");
              DRY_VALUES[2] = 0;
            }
          }
          else {//>
            if (BMS.SOC * 0.1 > DRYC_ARRAY[2].toFloat()) {
              // Serial.println("Dry contact A is enabled ");
              DRY_VALUES[2] = 1;
            }
            else if (BMS.SOC * 0.1 <= DRYC_ARRAY[3].toFloat()) {
              //   Serial.println("Dry contact A is disabled ");
              DRY_VALUES[2] = 0;
            }
          }
          break;

        case 4:
          DRY_VALUES[2] = 0;
          break;

        case 5:
          DRY_VALUES[2] = 1;
          break;

        case 6:
          if (SleepCounter < 57) {
            DRY_VALUES[2] = 1;
          }
          if (SleepCounter > 58) {
            DRY_VALUES[2] = 0;
          }
          break;

        case 7:
          DRY_VALUES[2] = !Safety;
          break;

        case 8:  // CELL-DIFF
          if (DRYA_ARRAY[1].toInt() == 0) { //<=

            if (BMS.max_cell_volt - BMS.min_cell_volt <= DRYA_ARRAY[2].toFloat()) {
              // Serial.println("Dry contact A is enabled ");
              DRY_VALUES[2] = 1;
            }
            else if (BMS.max_cell_volt  - BMS.min_cell_volt > DRYA_ARRAY[3].toFloat()) {
              //   Serial.println("Dry contact A is disabled ");
              DRY_VALUES[2] = 0;
            }
          }
          else {//>
            if (BMS.max_cell_volt  - BMS.min_cell_volt  > DRYA_ARRAY[2].toFloat()) {
              //   Serial.println("Dry contact A is enabled ");
              DRY_VALUES[2] = 1;
            }
            else if (BMS.max_cell_volt - BMS.min_cell_volt <= DRYA_ARRAY[3].toFloat()) {
              //  Serial.println("Dry contact A is disabled ");
              DRY_VALUES[2] = 0;
            }
          }
          break;

        default:
          break;
      }

      ////////////////////

      ////////////////////


      switch (DRYD_ARRAY[0].toInt()) {  //DRY CONTACT D

        case 0:  // Terminal Voltage Rule
          if (DRYD_ARRAY[1].toInt() == 0) { //<=

            if (BMS.sum_voltage * 0.1 <= DRYD_ARRAY[2].toFloat()) {
              // Serial.println("Dry contact A  voltageis enabled ");
              DRY_VALUES[3] = 1;
            }
            else if (BMS.sum_voltage * 0.1 > DRYD_ARRAY[3].toFloat()) {
              //  Serial.println("Dry contact A  voltageis disabled ");
              DRY_VALUES[3] = 0;
            }
          }
          else {//>
            if (BMS.sum_voltage * 0.1 > DRYD_ARRAY[2].toFloat()) {
              //  Serial.println("Dry contact A voltage is enabled ");
              DRY_VALUES[3] = 1;
            }
            else if (BMS.sum_voltage * 0.1 <= DRYD_ARRAY[3].toFloat()) {
              //Serial.println("Dry contact A voltage is disabled ");
              DRY_VALUES[3] = 0;
            }
          }
          break;

        case 1:  // Current
          if (DRYD_ARRAY[1].toInt() == 0) { //<=

            if (abs((BMS.current - 30000 ) * 0.1) <= DRYD_ARRAY[2].toFloat()) {
              // Serial.println("Dry contact max volt A is enabled ");
              DRY_VALUES[3] = 1;
            }
            else if (abs((BMS.current - 30000) * 0.1) > DRYD_ARRAY[3].toFloat()) {
              //  Serial.println("Dry contact max volt A is disabled ");
              DRY_VALUES[3] = 0;
            }
          }
          else {//>
            if (abs((BMS.current - 30000) * 0.1) > DRYD_ARRAY[2].toFloat()) {
              // Serial.println("Dry contact max volt A is enabled ");
              DRY_VALUES[3] = 1;
            }
            else if (abs((BMS.current - 30000) * 0.1) <= DRYD_ARRAY[3].toFloat()) {
              // Serial.println("Dry contact max volt A is disabled ");
              DRY_VALUES[3] = 0;
            }
          }
          break;


        case 2:  // temperature
          if (DRYD_ARRAY[1].toInt() == 0) { //<=

            if (BMS.max_cell_temp - 40 <= DRYD_ARRAY[2].toFloat()) {
              // Serial.println("Dry contact A is enabled ");
              DRY_VALUES[3] = 1;
            }
            else if (BMS.max_cell_temp - 40 > DRYD_ARRAY[3].toFloat()) {
              //   Serial.println("Dry contact A is disabled ");
              DRY_VALUES[3] = 0;
            }
          }
          else {//>
            if (BMS.max_cell_temp - 40 > DRYD_ARRAY[2].toFloat()) {
              //   Serial.println("Dry contact A is enabled ");
              DRY_VALUES[3] = 1;
            }
            else if (BMS.max_cell_temp - 40 <= DRYD_ARRAY[3].toFloat()) {
              //  Serial.println("Dry contact A is disabled ");
              DRY_VALUES[3] = 0;
            }
          }
          break;

        case 3:  // soc
          if (DRYD_ARRAY[1].toInt() == 0) { //<=

            if (BMS.SOC * 0.1 <= DRYD_ARRAY[2].toFloat()) {
              //  Serial.println("Dry contact A is enabled ");
              DRY_VALUES[3] = 1;
            }
            else if (BMS.SOC * 0.1 > DRYD_ARRAY[3].toFloat()) {
              //  Serial.println("Dry contact A is disabled ");
              DRY_VALUES[3] = 0;
            }
          }
          else {//>
            if (BMS.SOC * 0.1 > DRYD_ARRAY[2].toFloat()) {
              // Serial.println("Dry contact A is enabled ");
              DRY_VALUES[3] = 1;
            }
            else if (BMS.SOC * 0.1 <= DRYD_ARRAY[3].toFloat()) {
              //   Serial.println("Dry contact A is disabled ");
              DRY_VALUES[3] = 0;
            }
          }
          break;

        case 4:
          DRY_VALUES[3] = 0;
          break;

        case 5:
          DRY_VALUES[3] = 1;
          break;

        case 6:
          if (SleepCounter < 57) {
            DRY_VALUES[3] = 1;
          }
          if (SleepCounter > 58) {
            DRY_VALUES[3] = 0;
          }
          break;


        case 7:
          DRY_VALUES[3] = !Safety;
          break;

        case 8:  // CELL-DIFF
          if (DRYA_ARRAY[1].toInt() == 0) { //<=

            if (BMS.max_cell_volt  - BMS.min_cell_volt  <= DRYA_ARRAY[2].toFloat()) {
              // Serial.println("Dry contact A is enabled ");
              DRY_VALUES[3] = 1;
            }
            else if (BMS.max_cell_volt - BMS.min_cell_volt  > DRYA_ARRAY[3].toFloat()) {
              //   Serial.println("Dry contact A is disabled ");
              DRY_VALUES[3] = 0;
            }
          }
          else {//>
            if (BMS.max_cell_volt  - BMS.min_cell_volt  > DRYA_ARRAY[2].toFloat()) {
              //   Serial.println("Dry contact A is enabled ");
              DRY_VALUES[3] = 1;
            }
            else if (BMS.max_cell_volt  - BMS.min_cell_volt <= DRYA_ARRAY[3].toFloat()) {
              //  Serial.println("Dry contact A is disabled ");
              DRY_VALUES[3] = 0;
            }
          }
          break;

        default:
          break;
      }

      ////////////////////
      if (BMS.sum_voltage * 0.1 > 4) {

        for (int i = 0; i < 4; i++) {

          //   Serial.println("DRY CONTACT...............");

          if (DRY_VALUES[i]) {
            // Serial.println(String(i) + ":ON");
            gpio_out_on(8 - i); // for OLD BOARD
            gpio_out_on(4 - i); // for new board
            //Serial.println("gpio: " + String (i) + " enabled");
          }
          else {
            // Serial.println(String(i) + ":OFF");
            gpio_out_off(8 - i); //OLD BOARD
            gpio_out_off(4 - i); // for new board
            //Serial.println("gpio: " + String (i) + " disabled");
          }
        }
      }

      //DRY INPUTS//////////////////
      //      Serial.println("READ PIN1");
      //      Serial.println(gpio_read(1));
      //      Serial.println("READ PIN2");
      //      Serial.println( gpio_read(2));
      //      Serial.println("READ PIN3");
      //      Serial.println(gpio_read(3));


      GPIO_READ[0] = gpio_read(1);
      GPIO_READ[1] = gpio_read(2);
      GPIO_READ[2] = gpio_read(3);
      ///////////////////////////////////////////////////




      //Serial.println("BMS PARAMETER STARTED");
      //EQUALIZER
      //      EQUON = (BMS.current - 30000) <= 0;

      // Serial.println("BMS started:" + String(millis()));
      getBatteryParameters();
      //  Serial.println("BMS ended:" + String(millis()));


      for (int i = 0; i < 28; i++) {

        if (BMS.error[i] && (i != 0) && (i != 2) && (i != 4) && (i != 6)) {
          AlarmStringLog = AlarmArray[i] + " _Time:" + TimeString + " " + DateString;
          break;
        }
      }

      // EQUOFF = (BMS.current - 30000) > 0;

      //GET BALANCER STATE
      for (uint8_t i = 0; i < 16; i++) {
        balancerState = BMS.cell_balance[i] || balancerState;
        // Serial.println(BMS.cell_balance[i]);
      }
      //Serial.println("BMS PARAMETER ENDED");

      //GET ALARMS

      //Serial.println("ALARMS STARTED");
      alarmNo = 0;
      for (int i = 0; i < 28; i++) {
        if (BMS.error[i] == true) {
          alarmNo++;
        }
      }

      if (alarmNo != 0) {
        // SleepCounter = 0;
      }

      //Serial.println("ALARMS ENDED");

      //EQUAL[IZER2






      //Energy Calculations..........

      if (!EEPROMbusy && period != 0 ) {


        //AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA

        AlarmStringLogPro = AlarmStringLog.substring(0, AlarmStringLog.indexOf("_Time"));
        AlarmStringLogTime = AlarmStringLog.substring(AlarmStringLog.indexOf("Time"));

        //  preferences.begin("my-app", false);
        delay(10);

        //                Serial.println("Raw Alarm Log:" + AlarmStringLog);
        //                Serial.println("Processed Alarm Log:" + AlarmStringLogPro);
        //                Serial.println("Previous Raw Alarm Log:" + AlarmLog[0]);
        //                Serial.println("Previous Processed Alarm Log:" + AlarmLog[0].substring(0, AlarmLog[0].indexOf("_Time")));

        if (AlarmStringLogPro != AlarmLog[0].substring(0, AlarmLog[0].indexOf("_Time")) && AlarmStringLog != "No Alarm" && BMS.sum_voltage * 0.1 > 8 && BMS.max_cell_temp - 40 > -41 && (abs(BMS.current - 30000) * 0.1) < 500 && OpSec > 50) {



          Serial.println(AlarmStringLogPro);
          Serial.println(AlarmLog[0].substring(0, AlarmLog[0].indexOf("_Time")));

          for (int i = 0; i < 5; i++) {
            AlarmLog[15 - i] = AlarmLog[15 - (i + 1)];
          }
          AlarmLog[0] = AlarmStringLog;
          Serial.println("Alarmlog is triggered");
        }
      }
      previousMillisBMS = currentMillisBMS;

    }

    else  //Parameter Set
    {

      if (SafetyTurnOffApproved) {
        Serial.println("Command for turning off the safety is given");

        CommandStatus = false;
        for (int i = 0; i < 5; i++) {
          CommandStatus = set_charge_mode(true, BMS_Set_blue);
          notificationStatus = CommandStatus;
          if (CommandStatus) {
            Serial.println("turning off terminal Command Status:" + String(CommandStatus));
            CommandStatus = false;
            break;
          }
          else {
            //Serial.println(F("Error"));
          }
        }

        CommandStatus = false;
        for (int i = 0; i < 5; i++) {
          CommandStatus = set_discharge_mode(true, BMS_Set_blue);

          notificationStatus = CommandStatus;
          if (CommandStatus) {
            Serial.println("turning off terminal Command Status:" + String(CommandStatus));
            CommandStatus = false;
            break;
          }
          else {
            //Serial.println(F("Error"));
          }
        }


        Safety = false;
        SafetyTurnOffApproved = false;
        EEPROMbusy = true;
        preferences.begin("my-app", false);
        delay(10);
        preferences.putBool("Safety", Safety);
        preferences.end();
        EEPROMbusy = false;

      }

      if (SafetyTurnOnApproved) {
        Serial.println("Command for turning on the safety is given");


        CommandStatus = false;
        for (int i = 0; i < 5; i++) {
          CommandStatus = set_charge_mode(false, BMS_Set_blue);

          notificationStatus = CommandStatus;
          if (CommandStatus) {
            Serial.println("turning on terminal Command Status:" + String(CommandStatus));
            CommandStatus = false;
            break;
          }
          else {
            //Serial.println(F("Error"));
          }
        }


        CommandStatus = false;
        for (int i = 0; i < 5; i++) {
          CommandStatus = set_discharge_mode(false, BMS_Set_blue);

          notificationStatus = CommandStatus;
          if (CommandStatus) {
            Serial.println("turning on terminal Command Status:" + String(CommandStatus));
            CommandStatus = false;
            break;
          }
          else {
            //Serial.println(F("Error"));
          }
        }

        Safety = true;
        SafetyTurnOnApproved = false;
        EEPROMbusy = true;
        preferences.begin("my-app", false);
        delay(10);
        preferences.putBool("Safety", Safety);
        preferences.end();
        EEPROMbusy = false;



      }






      if (CHARGE_SET) {

        CommandStatus = false;
        for (int i = 0; i < 5; i++) {
          CommandStatus = set_charge_mode(WriteCharge, BMS_Set_blue);

          notificationStatus = CommandStatus;
          if (CommandStatus) {
            Serial.println("Command Status:" + String(CommandStatus));
            CommandStatus = false;
            CHARGE_SET = false;
            break;
          }
          else {
            //Serial.println(F("Error"));
          }
        }
        notificationEnable = true;
        CHARGE_SET = false;
        ReadAllParameters = true;
      }


      if (CVD_CAL) {
        CommandStatus = false;
        for (int i = 0; i < 5; i++) {
          CommandStatus = set_volttemp_diff_alarm(CVD, CVD, 0, 0);

          notificationStatus = CommandStatus;
          if (CommandStatus) {
            Serial.println("Command Status:" + String(CommandStatus));
            CommandStatus = false;
            CVD_CAL = false;
            break;
          }
          else {
            //Serial.println(F("Error"));
          }
        }
        notificationEnable = true;
        CVD_CAL = false;
        ReadAllParameters = true;
      }



      //SetEnergies

      if (SetEnergies) {
        CommandStatus = false;
        for (int i = 0; i < 5; i++) {
          CommandStatus = set_cumilative(SetCharge, SetDischarge);

          notificationStatus = CommandStatus;
          if (CommandStatus) {
            Serial.println("Command Status:" + String(CommandStatus));
            Serial.println("Set Energy done");
            Serial.println(SetCharge);
            Serial.println(SetDischarge);
            CommandStatus = false;
            //SetEnergies = false;
            break;
          }
          else {
            //Serial.println(F("Error"));
          }
        }
        notificationEnable = true;
        //SetEnergies = false;
        ReadAllParameters = true;

        delay(500);
        Serial.println("Reset bms command is given");
        resetBms();
        Serial.println("bms is resetted");
        delay(500);
        SetEnergies = false;
      }

      //SETCOMM

      if (SetCOMM) {
        CommandStatus = false;
        for (int i = 0; i < 5; i++) {

          METHOD = indexCOMMArray;
          TYPE = indexINVERTERSArray;
          Serial.println("innnnnnnnnnnnnnnnnnnnnnn");
          Serial.println(METHOD);
          Serial.println(TYPE);
          CommandStatus = set_comm_bms(TYPE, METHOD);

          notificationStatus = CommandStatus;
          if (CommandStatus) {
            Serial.println("Command Status:" + String(CommandStatus));
            Serial.println("SET COMM done");
            Serial.println(METHOD);
            Serial.println(TYPE);
            CommandStatus = false;
            //SetEnergies = false;
            break;
          }
          else {
            //Serial.println(F("Error"));
          }
        }
        notificationEnable = true;
        //SetEnergies = false;
        ReadAllParameters = true;

        delay(500);
        Serial.println("Reset bms command is given");
        resetBms();
        Serial.println("bms is resetted");
        delay(500);
        SetCOMM = false;
        bmsCounter = 6;
      }














      ////

      else if (DISCHARGE_SET) {
        CommandStatus = false;
        for (int i = 0; i < 5; i++) {
          CommandStatus = set_discharge_mode(WriteDischarge, BMS_Set_blue);

          Serial.println("Command Status:" + String(CommandStatus));
          notificationStatus = CommandStatus;
          if (CommandStatus) {

            CommandStatus = false;
            DISCHARGE_SET = false;
            break;
          }
          else {
            //Serial.println(F("Error"));
          }
        }
        notificationEnable = true;
        DISCHARGE_SET = false;
        ReadAllParameters = true;
      }

      //
      else if (HCC_HCD) {
        CommandStatus = false;
        delay(100);
        for (int i = 0; i < 5; i++) {
          CommandStatus = set_chargecurrent_BMS(HCC, HCD, BMS_Set);
          CommandStatus = param_save;
          Serial.println("Command Status:" + String(CommandStatus));
          notificationStatus = CommandStatus;
          if (CommandStatus) {
            CommandStatus = false;
            HCC_HCD = false;
            break;
          }
          else {
            // Serial.println(F("Error"));
          }
        }
        //        Serial.println("param_save:");
        //        Serial.println(param_save);
        notificationEnable = true;
        HCC_HCD = false;
        ReadAllParameters = true;
      }

      //SOC


      //XXXXXXXXXXXXXX
      else if (SOC_CAL) {
        CommandStatus = false;
        for (int i = 0; i < 5; i++) {







          CommandStatus = set_soc_BMS(SystemSOC, (String(yil).substring(2)).toInt(), String(ayUInt).toInt(), String(gunUInt).toInt(), String(saatUInt ).toInt(), String(dakikaUInt).toInt(), String(saniyeUInt).toInt());

          Serial.println("settted rtc");
          Serial.println((String(yil).substring(2)).toInt());
          Serial.println(String(ayUInt).toInt());
          Serial.println(String(gunUInt).toInt());
          Serial.println(String(saatUInt).toInt());
          Serial.println(String(dakikaUInt).toInt());
          Serial.println(String(saniyeUInt).toInt());
          Serial.println("------------settted rtc------------");


          Serial.println("SOC is calibrated");
          Serial.println(SystemSOC);
          notificationStatus = CommandStatus;
          if (CommandStatus) {
            CommandStatus = false;
            SOC_CAL = false;
            break;
          }
          else {
            //Serial.println(F("Error"));
          }
        }
        notificationEnable = true;
        SOC_CAL = false;
        ReadAllParameters = true;
      }

      //
      //SUM_CAL
      else if (SUM_CAL) {
        CommandStatus = false;
        for (int i = 0; i < 5; i++) {
          CommandStatus = set_sumvolt_alarm(SUMHI - 2 , SUMHI, SUMLOW + 2, SUMLOW);

          notificationStatus = CommandStatus;
          Serial.println("Command Status:" + String(CommandStatus));
          if (CommandStatus) {
            CommandStatus = false;
            SUM_CAL = false;
            break;
          }
          else {
            // Serial.println(F("Error"));
          }
        }
        notificationEnable = true;
        SUM_CAL = false;
        ReadAllParameters = true;
      }


      else if (SOC_ALARM) {
        CommandStatus = false;
        for (int i = 0; i < 5; i++) {
          CommandStatus = set_soc_alarm(SOCHI - 2 , SOCHI, SOCLOW + 2, SOCLOW);
          Serial.println(SOCHI);
          Serial.println(SOCLOW);

          notificationStatus = CommandStatus;
          Serial.println("SOC Command Status:" + String(CommandStatus));
          if (CommandStatus) {
            CommandStatus = false;
            SOC_ALARM = false;
            break;
          }
          else {
            // Serial.println(F("Error"));
          }
        }
        notificationEnable = true;
        SOC_ALARM = false;
        ReadAllParameters = true;
      }


      else if (pinged) {
        FooterEvent = 22;
        SleepCounter = 1;
        for (int i = 0; i < 20; i++) {
          delay(250);
          Serial.println("Pinging");
          buzzer_on();
          delay(250);
          buzzer_off();
        }
        pinged = false;

      }



      else if (resetBMS) {
        resetBMS = false;
        resetBms();
      }


      //HVC
      else if (HVC_LVC) {
        CommandStatus = false;
        for (int i = 0; i < 5; i++) {
          CommandStatus = set_cellvolt_BMS(HVC, LVC, BMS_Set) ;
          Serial.println(HVC);
          Serial.println(LVC);
          notificationStatus = CommandStatus;
          Serial.println("Command Status:" + String(CommandStatus));
          if (CommandStatus) {
            CommandStatus = false;
            HVC_LVC = false;
            break;
          }
          else {
            // Serial.println(F("Error"));
          }
        }
        notificationEnable = true;
        HVC_LVC = false;
        ReadAllParameters = true;
      }


      //HTC_HTD
      else if (HTC_HTD) {
        CommandStatus = false;
        for (int i = 0; i < 5; i++) {
          CommandStatus = set_temp_alarm(HTC , HTC, -20, -20, HTD , HTD, -20, -20);
          Serial.println("high temperature is setted");
          Serial.println(HTC);
          Serial.println(HTD);
          notificationStatus = CommandStatus;
          Serial.println("Command Status:" + String(CommandStatus));
          if (CommandStatus) {
            CommandStatus = false;
            HTC_HTD = false;
            break;
          }
          else {
            // Serial.println(F("Error"));
          }
        }
        notificationEnable = true;
        HTC_HTD = false;
        ReadAllParameters = true;
      }


      //SLP
      else if (SLP_CAL) {

        Serial.print("SLP_CAL:");
        Serial.println(SLP_CAL);
        CommandStatus = false;
        for (int i = 0; i < 5; i++) {

          CommandStatus = set_manufect_BMS(0 , board1, SLP, BMS_Set, 2);
          Serial.println("------SLeep---");
          Serial.println(SLP);
          Serial.println("------SLeep---");

          notificationStatus = CommandStatus;
          //Serial.println("Command Status:" + String(CommandStatus));
          if (CommandStatus) {
            CommandStatus = false;
            SLP_CAL = false;
            break;
          }
          else {
            // Serial.println(F("Error"));
          }
        }
        notificationEnable = true;
        SLP_CAL = false;
        ReadAllParameters = true;
      }




      //CURRENT_CALL
      else if (CURRENT_CAL) {
        CommandStatus = false;
        CalibrationStatus = 2;
        for (int i = 0; i < 5; i++) {

          if (CalibrationValue == 0) {
            CommandStatus = set_bmscurrent_calib(CalibrationValue, 1, 1);
            Serial.println("Zero Current Calibration value:");
            Serial.println(CalibrationValue);
            //Serial.println("Command Status: " + String(CommandStatus));
          }

          else {


            if (CalibrationValue < 0) {
              CommandStatus = set_bmscurrent_calib(CalibrationValue, 2, 0);
              Serial.println("negative Current Calibration value:");
              Serial.println(CalibrationValue);
              // Serial.println("Command Status: " + String(CommandStatus));
            }
            else  {
              CommandStatus = set_bmscurrent_calib(CalibrationValue, 2, 1);
              Serial.println("positive Current Calibration value:");
              Serial.println(CalibrationValue);
              // Serial.println("Command Status: " + String(CommandStatus));
            }
          }

          notificationStatus = CommandStatus;
          Serial.println("Command Status:" + String(CommandStatus));
          if (CommandStatus) {
            delay(500);
            CURRENT_CAL = false;
          }
          else {
            // Serial.println(F("Error"));
          }
        }
        notificationEnable = true;
        CURRENT_CAL = false;
        CalibrationStatus = CommandStatus;
        CommandStatus = false;
        ReadAllParameters = true;
      }




      //SET CANBUS ID
      else if (SetCANID) {
        CommandStatus = false;
        CalibrationStatus = 2;
        for (int i = 0; i < 5; i++) {
          CommandStatus = set_boardnumber_BMS( SettedCANID, 1, BMS_Set);
          Serial.println("CAN ID is setting: " + String(SettedCANID));
          Serial.println("Command Status: " + String(CommandStatus));
          notificationStatus = CommandStatus;
          if (CommandStatus) {
            SetCANID = false;
          }
          else {
            // Serial.println(F("Error"));
          }
        }
        CalibrationStatus = CommandStatus;
        notificationEnable = true;
        SetCANID = false;
        CommandStatus = false;
        readCANID = false;
        delay(1);
        ReadAllParameters = true;
      }


      else if (SERIAL_CAL) {
        CommandStatus = false;
        CalibrationStatus = 2;
        for (int i = 0; i < 5; i++) {
          CommandStatus = set_serial_board(SerialNumber);
          Serial.println("serialnumber is setting");
          Serial.println(SerialNumber);
          Serial.println("Command Status: " + String(CommandStatus));


          notificationStatus = CommandStatus;
          //Serial.println("Command Status:" + String(CommandStatus));
          if (CommandStatus) {
            delay(500);

            SERIAL_CAL = false;
          }
          else {
            // Serial.println(F("Error"));
          }
        }
        notificationEnable = true;
        SERIAL_CAL = false;
        CalibrationStatus = CommandStatus;
        CommandStatus = false;
        ReadAllParameters = true;
      }



      else if (RC_RV) {
        CommandStatus = false;
        for (int i = 0; i < 5; i++) {

          if (RV != 0) {
            CommandStatus = set_rated_BMS(RC, RV, BMS_Set);
          }
          else {

            Serial.println("Capacity is setted");
            Serial.println(SystemCAP);
            CommandStatus = set_rated_BMS(SystemCAP, 3.20, BMS_Set);
          }

          notificationStatus = CommandStatus;
          // Serial.println(F("Command Status:" + String(CommandStatus)));
          if (CommandStatus) {
            CommandStatus = false;
            RC_RV = false;
            break;
          }
          else {
            //  Serial.println(F("Error"));
          }

        }
        notificationEnable = true;
        RC_RV = false;
        ReadAllParameters = true;
      }

      //


      //BV-BD
      else if (BV_BD) {
        CommandStatus = false;
        for (int i = 0; i < 5; i++) {
          CommandStatus = set_balancevolt_BMS(BV, BD * 0.001, BMS_Set);

          notificationStatus = CommandStatus;
          // Serial.println(F("Command Status:" + String(CommandStatus)));
          if (CommandStatus) {
            CommandStatus = false;
            BV_BD = false;

            break;
          }
          else {
            // Serial.println(F("Error"));
          }
        }
        notificationEnable = true;
        BV_BD = false;
        ReadAllParameters = true;
      }

      //
    }
    delay(50);
  }
}


void UDPTEST_CODE( void * pvParameters ) {


  AsyncUDP udpcomm;
  WiFiServer serverWifi(81);


  for (;;) {
    // if WiFi is down, try reconnecting
    if ((WiFi.status() != WL_CONNECTED) ) {
      Serial.println(F("waiting for wifi"));
      delay(250);
    }
    else {


      //ASYNC UDP
      if (udpcomm.listen(2001)) {
        udpcomm.onPacket([](AsyncUDPPacket packet) {
          UDPRequest = String( (char*) packet.data());

          //Serial.println("Incoming Request:"+UDPRequest);


          if (UDPRequest.indexOf("SETHCC") != -1) {
            UDP_COMMAND_INDEX = 0;
            packet.printf("Current Alarms", packet.length());
          }

          else if (UDPRequest.indexOf("SETSUMHI") != -1) {
            UDP_COMMAND_INDEX = 1;
            packet.printf("SUM Voltage Alarms", packet.length());
          }


          else if (UDPRequest.indexOf("SETHVC") != -1) {
            UDP_COMMAND_INDEX = 2;
            packet.printf("Cell Voltage Alarms", packet.length());
          }


          else if (UDPRequest.indexOf("SETHTC") != -1) {
            UDP_COMMAND_INDEX = 3;
            packet.printf("Temperature Alarms", packet.length());
          }


          else if (UDPRequest.indexOf("SETCHARGE") != -1) {
            UDP_COMMAND_INDEX = 4;
            packet.printf("Energy configured", packet.length());
          }


          else if (UDPRequest.indexOf("SHI") != -1) {
            UDP_COMMAND_INDEX = 5;
            packet.printf("SOC ALARM CONFIGURED", packet.length());
          }

          else if (UDPRequest.indexOf("RESETBMS") != -1) {
            UDP_COMMAND_INDEX = 6;
            packet.printf("BMS WILL BE RESETTED", packet.length());
          }



          //SET SERIAL BMS////////////////////////////////
          //EN-02-048V0-0-3-3-0-0-23-A-00001
          param_start2 = UDPRequest.indexOf("SETSR");
          param_end2 = UDPRequest.indexOf("#");
          if (param_start2 != -1 && param_end2 != -1) {
            UDPprocessor = UDPRequest.substring(param_start2 + 5, param_end2);
            SerialNumber = UDPprocessor;
            packet.printf(SerialNumber.c_str(), packet.length());
            EEPROMbusy = true;
            preferences.begin("my-app", false);
            preferences.putString("SerialNumber", SerialNumber);
            preferences.end();
            SERIAL_CAL = true;
            EEPROMbusy = false;
          }///////////////////////////////////////////////


          //SET time OFFSET////////////////////////////////
          param_start2 = UDPRequest.indexOf("SETOFFSET");
          param_end2  = UDPRequest.indexOf("#");
          if (param_start2 != -1 && param_end2 != -1) {
            UDPprocessor = UDPRequest.substring(param_start2 + 9, param_end2);
            Serial.println(UDPprocessor);
            EEPROMbusy = true;


            timeoffset = UDPprocessor.toInt();


            EEPROMbusy = true;
            preferences.begin("my-app", false);
            delay(10);
            preferences.putInt("timeoffset", timeoffset);
            delay(10);
            preferences.end();
            EEPROMbusy = false;


            EEPROMbusy = false;
            packet.printf("RTC ofset is calibrated", packet.length());
            RTCFetched = false;
          }///////////////////////////////////////////////


          //SET CAP BMS////////////////////////////////
          param_start2 = UDPRequest.indexOf("SETCAP");
          param_end2 = UDPRequest.indexOf("#");
          if (param_start2 != -1 && param_end2 != -1) {
            UDPprocessor = UDPRequest.substring(param_start2 + 6, param_end2);
            SystemCAP = UDPprocessor.toInt();
            Serial.println(SystemCAP);
            RC_RV = true;
            packet.printf("CAPACITY calibrated", packet.length());
            RV = 0;
          }///////////////////////////////////////////////




          //Current Calibration////////////////////////////////
          param_start2 = UDPRequest.indexOf("CURRENTCAL");
          param_end2 = UDPRequest.indexOf("#");
          if (param_start2 != -1 && param_end2 != -1) {
            UDPprocessor = UDPRequest.substring(param_start2 + 10, param_end2);
            CalibrationValue = UDPprocessor.toFloat();
            Serial.println("Current is calibrated:" + String(CalibrationValue));
            packet.printf("Current is calibrated:", packet.length());
            CURRENT_CAL = true;
            FooterEvent = 17;
          }///////////////////////////////////////////////

          //SET BMU SERIAL////////////////////////////////
          param_start2 = UDPRequest.indexOf("SETBMUSERIAL");
          param_end2 = UDPRequest.indexOf("#");
          if (param_start2 != -1 && param_end2 != -1) {
            Serial.println("SETTED BMU");
            SERIAL_CAL = true;
            FooterEvent = 30;

            packet.printf("BMU Serial is setted", packet.length());

          }///////////////////////////////////////////////


          //SET PING BMS////////////////////////////////
          param_start2 = UDPRequest.indexOf("PING");
          param_end2 = UDPRequest.indexOf("#");
          if (param_start2 != -1 && param_end2 != -1) {
            Serial.println("PINGED");
            pinged = true;
            SleepCounter = 1;
            packet.printf("PONG", packet.length());

          }///////////////////////////////////////////////


          //RESET ALARM LIST////////////////////////////////
          param_start2 = UDPRequest.indexOf("RESETALARM");
          param_end2 = UDPRequest.indexOf("#");
          if (param_start2 != -1 && param_end2 != -1) {
            Serial.println("ALARM LIST RESETTED");

            packet.printf("ALARM LIST RESETTED", packet.length());


            for (int i = 0; i < 5; i++) {
              AlarmLog[i] = "No Alarm";
            }



            EEPROMbusy = true;

            EEPROMbusy = false;

          }///////////////////////////////////////////////




          //RESET FIXED STATS////////////////////////////////
          param_start2 = UDPRequest.indexOf("RESETCOUNTERS");
          param_end2 = UDPRequest.indexOf("#");
          if (param_start2 != -1 && param_end2 != -1) {
            Serial.println("COUNTERS ARE RESETTED");



            OpSec = 0;
            OpMin = 0;
            OpHour = 0;
            OpDay = 0;
            MinCurrentP = 0;
            MaxCurrentP = 0;



          }///////////////////////////////////////////////


          //DRY CONTACTS_________________________________________________


          //SET DRYA /////////////////////////////////
          param_start2 = UDPRequest.indexOf("SETDRYA");
          param_end2 = UDPRequest.indexOf("#");
          if (param_start2 != -1 && param_end2 != -1) {
            UDPprocessor = UDPRequest.substring(param_start2 + 7, param_end2);

            UDP_STRING = UDPprocessor;
            UDP_REMAINING = UDPprocessor;
            UDP_Dryindex = 0;
            UDP_DRY_ARRAY[4];

            for (int i = 0; i < 4; i++) {
              // Serial.println("------------");
              UDP_Dryindex = UDP_REMAINING.indexOf('/');
              UDP_DRY_ARRAY[i] = UDP_REMAINING.substring(0, UDP_Dryindex);
              UDP_REMAINING = UDP_REMAINING.substring(UDP_Dryindex + 1);
            }

            UDPDryString = String(UDP_DRY_ARRAY[0]) + "/" + String(UDP_DRY_ARRAY[1]) + "/" + String(UDP_DRY_ARRAY[2]) + "/" + String(UDP_DRY_ARRAY[3]);

            packet.printf(UDPDryString.c_str(), packet.length());
            Serial.println(UDPDryString);
            EEPROMbusy = true;
            preferences.begin("my-app", false);
            preferences.putString("DRYA", UDPDryString);
            DRYA_ARRAY[0] = UDP_DRY_ARRAY[0];
            DRYA_ARRAY[1] = UDP_DRY_ARRAY[1];
            DRYA_ARRAY[2] = UDP_DRY_ARRAY[2];
            DRYA_ARRAY[3] = UDP_DRY_ARRAY[3];
            preferences.end();
            EEPROMbusy = false;
          }///////////////////////////////////////////////



          //SET DRYB /////////////////////////////////
          param_start2 = UDPRequest.indexOf("SETDRYB");
          param_end2 = UDPRequest.indexOf("#");
          if (param_start2 != -1 && param_end2 != -1) {
            UDPprocessor = UDPRequest.substring(param_start2 + 7, param_end2);

            UDP_STRING = UDPprocessor;
            UDP_REMAINING = UDPprocessor;
            UDP_Dryindex = 0;
            UDP_DRY_ARRAY[4];

            for (int i = 0; i < 4; i++) {
              // Serial.println("------------");
              UDP_Dryindex = UDP_REMAINING.indexOf('/');
              UDP_DRY_ARRAY[i] = UDP_REMAINING.substring(0, UDP_Dryindex);
              UDP_REMAINING = UDP_REMAINING.substring(UDP_Dryindex + 1);
            }

            UDPDryString = String(UDP_DRY_ARRAY[0]) + "/" + String(UDP_DRY_ARRAY[1]) + "/" + String(UDP_DRY_ARRAY[2]) + "/" + String(UDP_DRY_ARRAY[3]);

            packet.printf(UDPDryString.c_str(), packet.length());
            Serial.println(UDPDryString);
            EEPROMbusy = true;
            preferences.begin("my-app", false);
            preferences.putString("DRYB", UDPDryString);
            DRYB_ARRAY[0] = UDP_DRY_ARRAY[0];
            DRYB_ARRAY[1] = UDP_DRY_ARRAY[1];
            DRYB_ARRAY[2] = UDP_DRY_ARRAY[2];
            DRYB_ARRAY[3] = UDP_DRY_ARRAY[3];
            preferences.end();
            EEPROMbusy = false;
          }///////////////////////////////////////////////




          //SET DRYC /////////////////////////////////
          param_start2 = UDPRequest.indexOf("SETDRYC");
          param_end2 = UDPRequest.indexOf("#");
          if (param_start2 != -1 && param_end2 != -1) {
            UDPprocessor = UDPRequest.substring(param_start2 + 7, param_end2);

            UDP_STRING = UDPprocessor;
            UDP_REMAINING = UDPprocessor;
            UDP_Dryindex = 0;
            UDP_DRY_ARRAY[4];

            for (int i = 0; i < 4; i++) {
              // Serial.println("------------");
              UDP_Dryindex = UDP_REMAINING.indexOf('/');
              UDP_DRY_ARRAY[i] = UDP_REMAINING.substring(0, UDP_Dryindex);
              UDP_REMAINING = UDP_REMAINING.substring(UDP_Dryindex + 1);
            }

            UDPDryString = String(UDP_DRY_ARRAY[0]) + "/" + String(UDP_DRY_ARRAY[1]) + "/" + String(UDP_DRY_ARRAY[2]) + "/" + String(UDP_DRY_ARRAY[3]);

            packet.printf(UDPDryString.c_str(), packet.length());
            Serial.println(UDPDryString);
            EEPROMbusy = true;
            preferences.begin("my-app", false);
            preferences.putString("DRYC", UDPDryString);
            DRYC_ARRAY[0] = UDP_DRY_ARRAY[0];
            DRYC_ARRAY[1] = UDP_DRY_ARRAY[1];
            DRYC_ARRAY[2] = UDP_DRY_ARRAY[2];
            DRYC_ARRAY[3] = UDP_DRY_ARRAY[3];
            preferences.end();
            EEPROMbusy = false;
          }///////////////////////////////////////////////



          //SET DRYD /////////////////////////////////
          param_start2 = UDPRequest.indexOf("SETDRYD");
          param_end2 = UDPRequest.indexOf("#");
          if (param_start2 != -1 && param_end2 != -1) {
            UDPprocessor = UDPRequest.substring(param_start2 + 7, param_end2);

            UDP_STRING = UDPprocessor;
            UDP_REMAINING = UDPprocessor;
            UDP_Dryindex = 0;
            UDP_DRY_ARRAY[4];

            for (int i = 0; i < 4; i++) {
              // Serial.println("------------");
              UDP_Dryindex = UDP_REMAINING.indexOf('/');
              UDP_DRY_ARRAY[i] = UDP_REMAINING.substring(0, UDP_Dryindex);
              UDP_REMAINING = UDP_REMAINING.substring(UDP_Dryindex + 1);
            }

            UDPDryString = String(UDP_DRY_ARRAY[0]) + "/" + String(UDP_DRY_ARRAY[1]) + "/" + String(UDP_DRY_ARRAY[2]) + "/" + String(UDP_DRY_ARRAY[3]);

            packet.printf(UDPDryString.c_str(), packet.length());
            Serial.println(UDPDryString);
            EEPROMbusy = true;
            preferences.begin("my-app", false);
            preferences.putString("DRYD", UDPDryString);
            DRYD_ARRAY[0] = UDP_DRY_ARRAY[0];
            DRYD_ARRAY[1] = UDP_DRY_ARRAY[1];
            DRYD_ARRAY[2] = UDP_DRY_ARRAY[2];
            DRYD_ARRAY[3] = UDP_DRY_ARRAY[3];
            preferences.end();
            EEPROMbusy = false;
          }///////////////////////////////////////////////







          //SET SOC BMS////////////////////////////////
          param_start2 = UDPRequest.indexOf("SETSOC");
          param_end2 = UDPRequest.indexOf("#");
          if (param_start2 != -1 && param_end2 != -1) {
            UDPprocessor = UDPRequest.substring(param_start2 + 6, param_end2);
            SystemSOC = UDPprocessor.toInt();
            Serial.println(SystemSOC);
            SOC_CAL = true;
            packet.printf("soc is calibrated", packet.length());
          }///////////////////////////////////////////////


          //STARTUP TRIGGER////////////////////////////////
          param_start2 = UDPRequest.indexOf("STARTUP");
          param_end2 = UDPRequest.indexOf("#");
          if (param_start2 != -1 && param_end2 != -1) {
            SafetyStartUp2 = true;
            packet.printf("start up is triggered", packet.length());
          }///////////////////////////////////////////////





          //RESTART MODULE////////////////////////////////
          param_start2 = UDPRequest.indexOf("RESET_MODULE");
          param_end2 = UDPRequest.indexOf("#");
          if (param_start2 != -1 && param_end2 != -1) {
            packet.printf("Module will be reseted", packet.length());
            delay(1000);


            for (int i = 0; i < 100; i++ )  {
              if (!deleteFilesProgress) {
                ESP.restart();
                break;
              }
              delay(10);
            }
            UpdateNextRestart = false;
          }///////////////////////////////////////////////




          //          //UODATE FIRMWARE MODULE////////////////////////////////
          //          param_start2 = UDPRequest.indexOf("UPDATE_MODULE");
          //          param_end2 = UDPRequest.indexOf("#");
          //          if (param_start2 != -1 && param_end2 != -1) {
          //            packet.printf("Module will be reseted and update firmware", packet.length());
          //            delay(250);
          //            EEPROMbusy = true;
          //            preferences.begin("my-app", false);
          //            preferences.putBool("UNR", true);
          //            preferences.putBool("FBNR", true);
          //            preferences.putBool("BTNR", false);
          //            preferences.putBool("WDNR", false);
          //            preferences.putBool("UDPNR", true);
          //            preferences.end();
          //            EEPROMbusy = false;
          //            delay(250);
          //            Serial.println("It will restart to update firmware");
          //            for (int i = 0; i < 100; i++ )  {
          //              if (!deleteFilesProgress) {
          //                ESP.restart();
          //                break;
          //              }
          //              delay(10);
          //            }
          //          }///////////////////////////////////////////////



          //          //CHNGE MODE TO ONLINE MONITORING////////////////////////////////
          //          param_start2 = UDPRequest.indexOf("SET_ONLINEMONITORING");
          //          param_end2 = UDPRequest.indexOf("#");
          //          if (param_start2 != -1 && param_end2 != -1) {
          //            packet.printf("Module will be reseted ato chage mode (ONLINE MONITORING)", packet.length());
          //            EEPROMbusy = true;
          //            preferences.begin("my-app", false);
          //            preferences.putBool("FBNR", true);
          //            preferences.putBool("BTNR", false);
          //            preferences.putBool("WDNR", false);
          //            preferences.putBool("UDPNR", true);
          //            preferences.end();
          //            delay(250);
          //            Serial.println("It will restart to configure");
          //            EEPROMbusy = false;
          //
          //            for (int i = 0; i < 100; i++ )  {
          //              if (!deleteFilesProgress) {
          //                ESP.restart();
          //                break;
          //              }
          //              delay(10);
          //            }
          //
          //          }///////////////////////////////////////////////


          //          //CHNGE MODE TO WIFI DIRECT////////////////////////////////
          //          param_start2 = UDPRequest.indexOf("SET_WIFIDIRECT");
          //          param_end2 = UDPRequest.indexOf("#");
          //          if (param_start2 != -1 && param_end2 != -1) {
          //            packet.printf("Module will be reseted ato chage mode (WIFI DIRECT)", packet.length());
          //            EEPROMbusy = true;
          //            preferences.begin("my-app", false);
          //            preferences.putBool("FBNR", false);
          //            preferences.putBool("BTNR", false);
          //            preferences.putBool("WDNR", true);
          //            preferences.putBool("UDPNR", true);
          //            preferences.end();
          //            delay(250);
          //            Serial.println("It will restart to configure");
          //            EEPROMbusy = false;
          //
          //            delay(500);
          //            for (int i = 0; i < 100; i++ )  {
          //              if (!deleteFilesProgress) {
          //                ESP.restart();
          //                break;
          //              }
          //              delay(10);
          //            }
          //
          //          }///////////////////////////////////////////////



          //TOGGLE SAFETY////////////////////////////////
          param_start2 = UDPRequest.indexOf("TOGGLE_SAFETY");
          param_end2 = UDPRequest.indexOf("#");
          if (param_start2 != -1 && param_end2 != -1) {


            if (Safety) {
              SafetyTurnOffApproved = true;
              SafetyTurnOnApproved = false;
              packet.printf("safety option is OFF", packet.length());
            }
            else {


              if ((BMS.current - 30000) * 0.1 < 2 && (BMS.current - 30000) * 0.1 > -2) {
                SafetyTurnOnApproved = true;
                SafetyTurnOffApproved = false;
                packet.printf("safety option is ON", packet.length());
              }



            }

          }///////////////////////////////////////////////



          //SafetyTurnOnApproved

          //SET SLEEP BMS////////////////////////////////


          param_start2 = UDPRequest.indexOf("SETSLP");
          param_end2 = UDPRequest.indexOf("#");

          if (param_start2 != -1 && param_end2 != -1) {
            UDPprocessor = UDPRequest.substring(param_start2 + 6, param_end2);
            SLP = UDPprocessor.toInt();
            Serial.println(SLP);
            SLP_CAL = true;
            Serial.println("Sleep command is given");
            Serial.println(SLP_CAL);
            packet.printf("sleep timer is calibrated", packet.length());
          }///////////////////////////////////////////////


          //SET RTC BMS////////////////////////////////
          param_start2 = UDPRequest.indexOf("SETRTC");
          param_end2 = UDPRequest.indexOf("#");
          if (param_start2 != -1 && param_end2 != -1) {
            UDPprocessor = UDPRequest.substring(param_start2 + 6, param_end2);
            UDP_REMAINING = UDPprocessor;
            Serial.println(UDPprocessor);

            for (int i = 0; i < 6; i++) {
              Serial.println("------------");
              UDP_RTCindex = UDP_REMAINING.indexOf('/');
              Serial.println(UDP_RTCindex);
              UDP_RTC_ARRAY[i] = UDP_REMAINING.substring(0, UDP_RTCindex).toInt();
              UDP_REMAINING = UDP_REMAINING.substring(UDP_RTCindex + 1);
            }

            UDPprocessor = String(UDP_RTC_ARRAY[0]) + "/" + String(UDP_RTC_ARRAY[1]) + "/" + String(UDP_RTC_ARRAY[2]) + "/" + String(UDP_RTC_ARRAY[3]) + "/" + String(UDP_RTC_ARRAY[4]) + "/" + String(UDP_RTC_ARRAY[5]);
            yil = UDP_RTC_ARRAY[0];
            ayUInt = UDP_RTC_ARRAY[1];
            gunUInt = UDP_RTC_ARRAY[2];
            saatUInt = UDP_RTC_ARRAY[3];
            dakikaUInt = UDP_RTC_ARRAY[4];
            saniyeUInt = UDP_RTC_ARRAY[5];
            //RTCCongifured = true;
            SOC_CAL = true;
            packet.printf(UDPprocessor.c_str(), packet.length());

          }///////////////////////////////////////////////




          //GET SERIAL BMS////////////////////////////////
          //EN-02-048V0-0-3-3-0-0-23-A-00001
          param_start2 = UDPRequest.indexOf("GETSR");
          param_end2 = UDPRequest.indexOf("#");
          if (param_start2 != -1 && param_end2 != -1) {
            packet.printf(SerialNumber.c_str(), packet.length());
          }///////////////////////////////////////////////





          //GET FIRMWARE BMS////////////////////////////////
          param_start2 = UDPRequest.indexOf("GETFW");
          param_end2 = UDPRequest.indexOf("#");
          if (param_start2 != -1 && param_end2 != -1) {
            packet.printf(FirmwareVer.c_str(), packet.length());
          }///////////////////////////////////////////////

          //GET MEAS BMS////////////////////////////////
          param_start2 = UDPRequest.indexOf("GET_MEAS1");
          param_end2 = UDPRequest.indexOf("#");
          if (param_start2 != -1 && param_end2 != -1) {

            String MEAS1 = "TV" + String(BMS.sum_voltage * 0.1) +  "/TC" + String((BMS.current - 30000) * 0.1) + "/SOC" +  String(BMS.SOC * 0.1) + "/TEMP" +  String(BMS.max_cell_temp - 40) +
                           "/MAX" + String(BMS.max_cell_volt * 0.001) + "/MIN" + String(BMS. min_cell_volt * 0.001) +  "/STT" + String(BMS.state) +
                           "/CH" + String(BMS.charge) + "/DSH" + String(BMS.discharge) + "/LF" + String(BMS.bms_life) +
                           "/RC" + String(BMS.rem_cap);




            //            Serial.print("Measurement1 sent:");
            //            Serial.println(MEAS1);
            packet.printf(MEAS1.c_str(), packet.length());
          }///////////////////////////////////////////////




          //GET MEAS2 BMS////////////////////////////////
          param_start2 = UDPRequest.indexOf("GET_MEAS2");
          param_end2 = UDPRequest.indexOf("#");
          if (param_start2 != -1 && param_end2 != -1) {

            String MEAS2 = "/BL" + String(balancerState) + "/CHE" + String(ChargeEnergy, 3) + "/DHE" + String(DischargeEnergy) + "/TM" + TimeString + "/DT" + DateString +
                           "/FW" + FirmwareVer + "/DA" + String(DRY_VALUES[0]) + "/DB" + String(DRY_VALUES[1]) + "/DC" + String(DRY_VALUES[2]) + "/DD" + String(DRY_VALUES[3]) + "/SF" + String(Safety) +
                           "/CELLTEMP1" + String(CellTemp1) + "/CELLTEMP2" + String(CellTemp2) + "/SD" + String(control_sd) + "/CR" + String(canbusReady) + "#";

            //            Serial.print("Measurement2 sent:");
            //            Serial.println(MEAS2);
            packet.printf(MEAS2.c_str(), packet.length());
          }///////////////////////////////////////////////


          //GET INFO BMS////////////////////////////////
          param_start2 = UDPRequest.indexOf("GET_INFO");
          param_end2 = UDPRequest.indexOf("#");
          if (param_start2 != -1 && param_end2 != -1) {

            String INFO = "RV" + String(BMS.rated_volt * 0.001) + "/RC" + String(NominalVoltage * BMS.rated_cap * 0.001 * 0.001) + "/NV" + String(NominalVoltage) + "#";

            //            Serial.print("Measurement2 sent:");
            //            Serial.println(MEAS2);
            packet.printf(INFO.c_str(), packet.length());
          }///////////////////////////////////////////////



          //GET CELLS BMS////////////////////////////////
          param_start2 = UDPRequest.indexOf("GET_CELLS");
          param_end2 = UDPRequest.indexOf("#");
          if (param_start2 != -1 && param_end2 != -1) {

            String CellString = "";
            for (int i = 0; i < ReadCellMod; i++) {
              CellString = CellString + String(BMS.cell_voltage[i] * 0.001) + "/";
            }
            CellString = "CELLS" + CellString + "#";
            packet.printf(CellString.c_str(), packet.length());
          }///////////////////////////////////////////////


          //GET ALARMS BMS////////////////////////////////
          param_start2 = UDPRequest.indexOf("GET_ALARMS");
          param_end2 = UDPRequest.indexOf("#");
          if (param_start2 != -1 && param_end2 != -1) {

            String AlarmsString = "";
            for (int i = 0; i < 28; i++) {
              AlarmsString = AlarmsString + String(BMS.error[i]) + "/";
            }
            AlarmsString = "ALARMS" + AlarmsString + "#";
            packet.printf(AlarmsString.c_str(), packet.length());
          }///////////////////////////////////////////////


          //GET LIMITS BMS////////////////////////////////
          param_start2 = UDPRequest.indexOf("GET_LIMITS");
          param_end2 = UDPRequest.indexOf("#");
          if (param_start2 != -1 && param_end2 != -1) {
            //ReadAllParameters = true;

            String LIMITS = "DSCHI" + String(BMS.dischar_curr2) + "/" + "CHCHI" + String(BMS.charge_curr2) + "/" + "SUMVHI" + String(BMS.sumv_high2 * 0.1) + "/" +
                            "SUMVLO" + String(BMS.sumv_low2 * 0.1) + "/" + "CVHI" + String(BMS.cell_volthigh2 * 0.001) + "/" + "CVLO" + String(BMS.cell_voltlow2 * 0.001) + "/" +
                            "BV" + String(BMS.balance_volt * 0.001) + "/" + "BDV" + String(BMS.balance_volt_diff) + "/" + "CTHI" + String(BMS.charge_temp_high2) + "/" +
                            "CTLO" + String(BMS.charge_temp_low2) + "/" + "DTHI" + String(BMS.discharge_temp_high2) + "/" + "DTLO" + String(BMS.discharge_temp_low2) + "/" +
                            "VDF" + String(BMS.volt_diff2) + "/" + "SUMVHIEQU" + String(BMS.sumv_high2 / 10) + "/" + "SUMVLOEQU" + String(BMS.sumv_low2 / 10) + "/" +
                            "BLVEQU" + String(BMS.balance_volt * 0.001) + "/" + "BLDELTA" + String(BMS.balance_volt_diff) + "/" + "RTVOLT" + String(BMS.rated_volt * 0.001) + "/" +
                            "SOCHI" + String(BMS.SOC_high2) + "/" +  "SOCLO" + String(BMS.SOC_low2) + "/" +  "VOLTDIF" + String(BMS.volt_diff2) + "/" +  "SYSCAP" + String(BMS.rem_cap * 0.001 * 0.001 * 48) + "/" +
                            "SLPTM" + String(BMS.secondsleepbm) + "/" + "SLPTMEQU" + String(BMS.secondsleepbm) +   "/";









            LIMITS = "LIMITS" + LIMITS + "#";
            packet.printf(LIMITS.c_str(), packet.length());
          }///////////////////////////////////////////////


          //GET DRYCONTACTS BMS////////////////////////////////
          param_start2 = UDPRequest.indexOf("GET_DRY");
          param_end2 = UDPRequest.indexOf("#");
          if (param_start2 != -1 && param_start2 != -1) {
            String DRYString = "";

            DRYString = "DRYA" + DRYA_ARRAY[0] + "/" + DRYA_ARRAY[1] + "/" + DRYA_ARRAY[2] + "/" + DRYA_ARRAY[3] + "/?DRYB" + DRYB_ARRAY[0] + "/" + DRYB_ARRAY[1] + "/" + DRYB_ARRAY[2] + "/" + DRYB_ARRAY[3] + "/?DRYC" +
                        DRYC_ARRAY[0] + "/" + DRYC_ARRAY[1] + "/" + DRYC_ARRAY[2] + "/" + DRYC_ARRAY[3] + "/?DRYD" + DRYD_ARRAY[0] + "/" + DRYD_ARRAY[1] + "/" + DRYD_ARRAY[2] + "/" + DRYD_ARRAY[3] + "/?#";

            //              DRYA_ARRAY[0] = String(ContactTypeIndex);
            //              DRYA_ARRAY[1] = String(OperationTypeIndex);
            //              DRYA_ARRAY[2] = String(DryContactValueEnable);
            //              DRYA_ARRAY[3] = String(DryContactValueDisable);

            packet.printf(DRYString.c_str(), packet.length());
          }///////////////////////////////////////////////
        });
      }
      delay(100);
    }
  }
}


void FIREBASE_CODE( void * pvParameters ) {

  Serial.println("Firebase task started");
  while (WiFi.status() != WL_CONNECTED) {
    Serial.println(F("Firebase task is waiting for internet connection..."));
    delay(500);
  }

  Firebase.begin(FIREBASE_HOST, FIREBASE_AUTH);
  for (;;) {
    int a = OpMin;

    //   (ConvBinUnits(ESP.getFreeHeap(), 3).toFloat()>78.5)

    if (  ( (OpSec > 43 && OpSec < 45) ) && OpMin > 0 && a % 10 == 0) {
      //      String FirebaseTimeStamp = "/Monitoring/ENCAP2024_1/" + SerialNumber + "/YEAR" + String(BMS.year) + "/MONTH" + String(BMS.month) + "/DAY" + String(BMS.day) + "/HOUR" + String(BMS.when) + "/TimeStamp";
      //      String FirebaseFirmwareVersion = "/Monitoring/ENCAP2024_1/" + SerialNumber + "/YEAR" + String(BMS.year) + "/MONTH" + String(BMS.month) + "/DAY" + String(BMS.day) + "/HOUR" + String(BMS.when) + "/Firmware";
      //      String FirebaseTerminalVoltage = "/Monitoring/ENCAP2024_1/" + SerialNumber + "/YEAR" + String(BMS.year) + "/MONTH" + String(BMS.month) + "/DAY" + String(BMS.day) + "/HOUR" + String(BMS.when) + "/TerminalVoltage";
      //      String FirebaseTerminalCurrent = "/Monitoring/ENCAP2024_1/" + SerialNumber + "/YEAR" + String(BMS.year) + "/MONTH" + String(BMS.month) + "/DAY" + String(BMS.day) + "/HOUR" + String(BMS.when) + "/TerminalCurrent";
      //      String FirebaseCellTemp = "/Monitoring/ENCAP2024_1/" + SerialNumber + "/YEAR" + String(BMS.year) + "/MONTH" + String(BMS.month) + "/DAY" + String(BMS.day) + "/HOUR" + String(BMS.when) + "/CellTemp";
      //      String FirebaseStateOfCharge = "/Monitoring/ENCAP2024_1/" + SerialNumber + "/YEAR" + String(BMS.year) + "/MONTH" + String(BMS.month) + "/DAY" + String(BMS.day) + "/HOUR" + String(BMS.when) + "/StateOfCharge";
      //      String FirebaseMaxCell = "/Monitoring/ENCAP2024_1/" + SerialNumber + "/YEAR" + String(BMS.year) + "/MONTH" + String(BMS.month) + "/DAY" + String(BMS.day) + "/HOUR" + String(BMS.when) + "/MaxCell";
      //      String FirebaseMinCell = "/Monitoring/ENCAP2024_1/" + SerialNumber + "/YEAR" + String(BMS.year) + "/MONTH" + String(BMS.month) + "/DAY" + String(BMS.day) + "/HOUR" + String(BMS.when) + "/MinCell";
      //      String FirebaseChargeEnergy = "/Monitoring/ENCAP2024_1/" + SerialNumber + "/YEAR" + String(BMS.year) + "/MONTH" + String(BMS.month) + "/DAY" + String(BMS.day) + "/HOUR" + String(BMS.when) + "/ChargeEnergy";
      //      String FirebaseDischargeEnergy = "/Monitoring/ENCAP2024_1/" + SerialNumber + "/YEAR" + String(BMS.year) + "/MONTH" + String(BMS.month) + "/DAY" + String(BMS.day) + "/HOUR" + String(BMS.when) + "/DischargeEnergy";
      //      String FirebaseRestartCounter = "/Monitoring/ENCAP2024_1/" + SerialNumber + "/YEAR" + String(BMS.year) + "/MONTH" + String(BMS.month) + "/DAY" + String(BMS.day) + "/HOUR" + String(BMS.when) + "/RestartCounter";
      //      String FirebaseMaxCurrent = "/Monitoring/ENCAP2024_1/" + SerialNumber + "/YEAR" + String(BMS.year) + "/MONTH" + String(BMS.month) + "/DAY" + String(BMS.day) + "/HOUR" + String(BMS.when) + "/MaxCurrent";
      //      String FirebaseMinCurrent = "/Monitoring/ENCAP2024_1/" + SerialNumber + "/YEAR" + String(BMS.year) + "/MONTH" + String(BMS.month) + "/DAY" + String(BMS.day) + "/HOUR" + String(BMS.when) + "/MinCurrent";
      //      String FirebaseAlarm = "/Monitoring/ENCAP2024_1/" + SerialNumber + "/YEAR" + String(BMS.year) + "/MONTH" + String(BMS.month) + "/DAY" + String(BMS.day) + "/HOUR" + String(BMS.when) + "/Alarm";
      //      String FirebaseSD = "/Monitoring/ENCAP2024_1/" + SerialNumber + "/YEAR" + String(BMS.year) + "/MONTH" + String(BMS.month) + "/DAY" + String(BMS.day) + "/HOUR" + String(BMS.when) + "/SD";

      String FirebaseData1 = "/Monitoring/ENCAP2024_1/" + SerialNumber + "/YEAR" + String(BMS.year) + "/MONTH" + String(BMS.month) + "/DAY" + String(BMS.day) + "/HOUR" + String(BMS.when) + "/Data1";
      String FirebaseData2 = "/Monitoring/ENCAP2024_1/" + SerialNumber + "/YEAR" + String(BMS.year) + "/MONTH" + String(BMS.month) + "/DAY" + String(BMS.day) + "/HOUR" + String(BMS.when) + "/Data2";



      Serial.println(F("Firebase task is running.................................."));
      if (WiFi.status() == WL_CONNECTED) {



        String Data1 = "TM" + TimeString + "/FW" + FirmwareVer + "/TV" + String(BMS.sum_voltage * 0.1) + "/TC" + String((BMS.current - 30000) * 0.1) + "/TT" + String(BMS.max_cell_temp - 40) + "/SOC" + String(BMS.SOC * 0.1) + "/#";
        String Data2 = "MXC" + String(BMS.max_cell_volt * 0.001) + "/MNC" + String(BMS. min_cell_volt * 0.001) + "/CE" + String(ChargeEnergy) + "/DE" + String(DischargeEnergy) + "/AS" + String(AlarmStringLogPro) + "/SD" + String(control_sd) + "/LAT" + String(Latitude) + "/LON" + String(Longitude) + "/CN" + CountryName + "/#";



        //Data1////////////////////////
        Firebase.setString(FirebaseData1.c_str(), Data1);
        if (Firebase.failed()) {
          Serial.print("FirebaseData1 failed:");
          Serial.println(Firebase.error());
          firebaseReady = false;
        }
        else {
          Serial.println("FirebaseData1 Firebase successfull!--------------------------------------------------");
          firebaseReady = true;
        }
        ///////////////////////////////////////////////////////////


        //Data2////////////////////////
        Firebase.setString(FirebaseData2.c_str(), Data2);
        if (Firebase.failed()) {
          Serial.print("FirebaseData2 failed:");
          Serial.println(Firebase.error());
          firebaseReady = false;
        }
        else {
          Serial.println("FirebaseData2 Firebase successfull!--------------------------------------------------");
          firebaseReady = true;
        }
      }
      else {
        Serial.println("No Internet Connection");
      }
    }
    delay(500);

  }
}


//Touch Task execution________________________________________________________________________________________________________________
void TOUCH_SCREEN_CODE( void * pvParameters ) {
  int TouchArray[100];
  int TouchArray_vertical[100];
  int touch_iteration = 0;
  bool action_done = false;
  bool button_done = false;
  bool settings_page = false;







  // TOUCH SETUP=============================

  while (!ts.begin(18, 19, 40)) {
    Serial.println(F("Touch Screen trying to start...."));
    delay(500);
  }

  Serial.println(F("Touch Screen started.   !!!!!!!!!!!!!!!!!!!!!"));



  for (uint8_t i = 0; i < 100; i++) {
    TouchArray[i] = 0;
  }
  for (;;) {

    delay(10);
    if (ts.touched()) {
      delay(10);
      TS_Point p = ts.getPoint();
      delay(10);






      p.x = map(p.x, 0, 320, 0, 320);
      p.y = map(p.y, 0, 480, 0, 480);
      int y = tft.height() - p.x;
      int x = p.y;
      int cursor_after = p.y;
      int cursor_after_vertical = p.x;
      TouchArray[touch_iteration] = cursor_after;
      TouchArray_vertical[touch_iteration] = cursor_after_vertical;




      //      Serial.print(x);
      //      Serial.print("/");
      //      Serial.println(y);




      ///Touch wake up
      if (y >= 0 && y <= 320 && x >= 0 && x <= 480) {
        SleepCounter = 0;
      }

      if (!action_done && y >= 280 && y <= 320 && x >= 10 && x <= 190) {
        Serial.println("Alarm Records from footer");
        AlarmReadTrigger = true;
        FooterEvent = 17;
        action_done = true;
        PageNumber = 26;

        delay(250);
      }



      if (!action_done && y >= 280 && y <= 320 && x >= 200 && x <= 285) {

        Serial.println("SAFET-----------------------------------------------------------");
        Serial.println((BMS.current - 30000) * 0.1 < 2 );
        Serial.println((BMS.current - 30000) * 0.1 > -2);

        FooterEvent = 15;


        if (!Safety) {
          if ((BMS.current - 30000) * 0.1 < 2 && (BMS.current - 30000) * 0.1 > -2)
          {
            Safety = true;
            SafetyTurnOnApproved = true;
          }

          else {
            SafetyAlarm = true;

            FooterEvent = 1;
            Serial.println("Safety Alarm");
            Serial.println(abs(BMS.current - 30000) * 0.1);
          }
        }
        else {
          Safety = false;
          SafetyTurnOffApproved = true;
        }
        action_done = true;
        delay(250);
      }


      if (!action_done && y >= 270 && y <= 320 && x >= 295 && x <= 480) {

        Serial.println("footer multifunctional rectangle");
        FooterTouch = !FooterTouch;
        buzzerActive = true;
        action_done = true;
      }





      if (!action_done && y >= 0 && y <= 60 && x >= 0 && x <= 100) {

        Serial.println("Menu button touched");
        dashboard_page = 0;
        PageNumber = LastPageNumber;
        action_done = true;
        SafetyAlarm = false;
        AlarmLogStep = 0;
        buzzerActive = true;
      }


      if (!action_done && y >= 0 && y <= 60 && x >= 380 && x <= 480) {

        Serial.println("Back button touched");
        dashboard_page = 0;
        PageNumber = 0;
        dashboard_page = 0;
        action_done = true;
        SafetyAlarm = false;
        AlarmLogStep = 0;

      }

      if (PageNumber == 1) {
        //Menu size increased
        if (!action_done && y >= 70 && y <= 266 && x >= 447 && x <= 480) {
          PageNumber = 2;
          action_done = true;


        }

        if (!action_done && y >= 70 && y <= 266 && x >= 0 && x <= 64) {
          PageNumber = 0;
          action_done = true;

        }
      }

      if (PageNumber == 2) {
        if (!action_done && y >= 70 && y <= 266 && x >= 0 && x <= 64) {
          PageNumber = 1;
          action_done = true;

        }

        if (!action_done && y >= 70 && y <= 266 && x >= 447 && x <= 480) {
          PageNumber = 18;
          action_done = true;

        }
      }

      if (PageNumber == 18) {

        //Menu size increased
        if (!action_done && y >= 70 && y <= 266 && x >= 447 && x <= 480) {
          PageNumber = 29;
          action_done = true;

          Serial.println("menu size increased");
        }

        if (!action_done && y >= 70 && y <= 266 && x >= 0 && x <= 64) {
          PageNumber = 2;
          action_done = true;

        }
      }



      if (PageNumber == 29) {

        //Menu size increased


        if (!action_done && y >= 70 && y <= 266 && x >= 0 && x <= 64) {
          PageNumber = 18;
          action_done = true;

          Serial.println("menu size decreased");
        }
      }



      if (PageNumber == 0) {
        //1#panel touched
        if (!action_done && y >= 62 && y <= 222 && x >= 0 && x <= 239) {

          Serial.println("First panel touched");
          dashboard_page++;
          dashboard_page = dashboard_page % 11;
          action_done = true;
          Serial.println(dashboard_page);



        }
      }

      if (PageNumber == 2) {

        if (!action_done && y >= 62 && y <= 165 && x >= 36 && x <= 171) {
          Serial.println("Online Monitoring");
          FooterEvent = 2;
          action_done = true;
          EEPROMbusy = true;
          preferences.begin("my-app", false);
          preferences.putBool("FBNR", true);
          preferences.putBool("BTNR", false);
          preferences.putBool("WDNR", false);
          preferences.putBool("UDPNR", true);
          preferences.end();
          EEPROMbusy = false;

          delay(500);
          Serial.println("It will restart to configure");

          for (int i = 0; i < 100; i++ )  {
            if (!deleteFilesProgress) {
              ESP.restart();
              break;
            }
            delay(10);
          }
        }

        if (!action_done && y >= 167 && y <= 270 && x >= 64  && x <= 171) {
          Serial.println("Restart Module Page2");
          FooterEvent = 4;
          action_done = true;
          //
          SD.end();
          delay(500);

          for (int i = 0; i < 100; i++ )  {
            if (!deleteFilesProgress) {
              ESP.restart();
              break;
            }
            delay(10);
          }
        }

        if (!action_done && y >= 62 && y <= 165 && x >= 173 && x <= 308) {
          FooterEvent = 3;
          Serial.println("Wifi direct Monitoring");
          action_done = true;
          EEPROMbusy = true;

          preferences.begin("my-app", false);
          preferences.putBool("FBNR", false);
          preferences.putBool("BTNR", false);
          preferences.putBool("WDNR", true);
          preferences.putBool("UDPNR", true);
          preferences.end();
          EEPROMbusy = false;

          delay(500);
          Serial.println("It will restart to configure");

          for (int i = 0; i < 100; i++ )  {
            if (!deleteFilesProgress) {
              ESP.restart();
              break;
            }
            delay(10);
          }
        }

        if (!action_done && y >= 167 && y <= 270 && x >= 173 && x <= 308) {
          Serial.println("Firmware update");

          FooterEvent = 5;


          action_done = true;
          EEPROMbusy = true;
          preferences.begin("my-app", false);
          preferences.putBool("UNR", true);
          preferences.end();
          EEPROMbusy = false;

          delay(500);
          Serial.println("It will restart to update firmware");

          for (int i = 0; i < 100; i++ )  {
            if (!deleteFilesProgress) {
              ESP.restart();
              break;
            }
            delay(10);
          }
        }



        if (!action_done && y >= 167 && y <= 270 && x >= 310 && x <= 425) {
          Serial.println("QR Code for manual");
          FooterEvent = 6;
          action_done = true;
          PageNumber = 16;

          delay(500);

        }



        if (!action_done && y >= 61 && y <= 164 && x >= 310 && x <= 425) {
          FooterEvent = 7;
          Serial.println("Bluetooth Monitoring");
          action_done = true;
          EEPROMbusy = true;
          preferences.begin("my-app", false);
          preferences.putBool("FBNR", false);
          preferences.putBool("BTNR", true);
          preferences.putBool("WDNR", false);
          preferences.putBool("UDPNR", false);


          Serial.println("BTNR:" + String(preferences.getBool("BTNR", false)));


          preferences.end();
          EEPROMbusy = false;

          delay(500);
          Serial.println("It will restart to configure");

          for (int i = 0; i < 100; i++ )  {
            if (!deleteFilesProgress) {
              ESP.restart();
              break;
            }
            delay(10);
          }
        }

        //        if (!action_done && y >= 167 && y <= 270 && x >= 310 && x <= 425) {
        //          Serial.println("Product details");
        //          FooterEvent = 8;
        //          action_done = true;
        //        }

      }

      if (PageNumber == 1) {

        if (!action_done && y >= 62 && y <= 165 && x >= 64 && x <= 171) {
          Serial.println("Main Dashboard");
          FooterEvent = 8;
          action_done = true;

          PageNumber = 0;
        }

        if (!action_done && y >= 167 && y <= 270 && x >= 36  && x <= 171) {
          Serial.println("Cell Monitoring");
          FooterEvent = 9;
          PageNumber = 3;

        }


        if (!action_done && y >= 62 && y <= 165 && x >= 173  && x <= 308) {
          Serial.println("Network Settings");
          FooterEvent = 10;
          PageNumber = 4;

        }

        if (!action_done && y >= 167 && y <= 270 && x >= 173  && x <= 308) {
          Serial.println("Dry Contact Settings");
          FooterEvent = 11;
          PageNumber = 5;

        }

        if (!action_done && y >= 62 && y <= 165 && x >= 310  && x <= 425) {
          Serial.println("Daily Stats");
          FooterEvent = 12;
          PageNumber = 6;

        }


        if (!action_done && y >= 167 && y <= 270 && x >= 310 && x <= 425) {
          Serial.println("System Details");

          action_done = true;
          PageNumber = 17;
          ReadAllParameters = true;



        }

      }


      if (PageNumber == 18) {
        if (!action_done && y >= 62 && y <= 165 && x >= 36 && x <= 171) {
          Serial.println("System Setup");
          FooterEvent = 14;
          action_done = true;
          PageNumber = 19;

          delay(250);
        }

        if (!action_done && y >= 167 && y <= 270 && x >= 36 && x <= 171) {
          Serial.println("Set Simulation");
          FooterEvent = 65;
          action_done = true;
          PageNumber = 28;

          delay(250);
        }



        if (!action_done && y >= 62 && y <= 165 && x >= 173 && x <= 308) {
          Serial.println("Alarm Records");
          AlarmReadTrigger = true;
          FooterEvent = 17;
          action_done = true;
          PageNumber = 26;

          delay(250);
        }


        if (!action_done && y >= 280 && y <= 320 && x >= 10 && x <= 190) {
          Serial.println("Alarm Records from dashboard");
          FooterEvent = 17;
          action_done = true;
          PageNumber = 26;

          delay(250);
        }





        if (!action_done && y >= 62 && y <= 165 && x >= 310  && x <= 425) {
          Serial.println("MonitoringQR");
          FooterEvent = 50;
          PageNumber = 21;
          action_done = true;

        }

        if (!action_done && y >= 167 && y <= 270 && x >= 310  && x <= 425) {
          Serial.println("System Stats");
          FooterEvent = 55;
          PageNumber = 24;
          action_done = true;

        }

        if (!action_done && y >= 167 && y <= 270 && x >= 36  && x <= 171) {
          //          Serial.println("Setting Serial");
          //          FooterEvent = 30;
          //          SERIAL_CAL = true;
          //          action_done = true;
        }

        if (!action_done && y >= 167 && y <= 270 && x >= 173 && x <= 308) {
          Serial.println("Dry Contact Write");
          FooterEvent = 33;
          action_done = true;
          PageNumber = 22;
          delay(250);

        }
      }


      if (PageNumber == 22) {
        //UDPDryString = String(UDP_DRY_ARRAY[0]) + "/" + String(UDP_DRY_ARRAY[1]) + "/" + String(UDP_DRY_ARRAY[2]) + "/" + String(UDP_DRY_ARRAY[3]);


        String DRYSETTING = String(ContactFunctionListIndex) + "/" + String(OperationListIndex) + "/" + String(DryEnable) + "/" + String(DryDisable);

        if (!action_done && y >= 218 && y <= 268 && x >= 0  && x <= 480) {
          Serial.println(ContactNameListIndex);


          switch (ContactNameListIndex) {
            case 0:
              Serial.println(DRYSETTING);
              EEPROMbusy = true;
              preferences.begin("my-app", false);
              preferences.putString("DRYA", DRYSETTING);
              DRYA_ARRAY[0] = ContactFunctionListIndex;
              DRYA_ARRAY[1] = OperationListIndex;
              DRYA_ARRAY[2] = DryEnable.toFloat();
              DRYA_ARRAY[3] = DryDisable.toFloat();
              preferences.end();
              EEPROMbusy = false;
              break;

            case 1:
              Serial.println(DRYSETTING);
              EEPROMbusy = true;
              preferences.begin("my-app", false);
              preferences.putString("DRYB", DRYSETTING);
              DRYB_ARRAY[0] = ContactFunctionListIndex;
              DRYB_ARRAY[1] = OperationListIndex;
              DRYB_ARRAY[2] = DryEnable.toFloat();
              DRYB_ARRAY[3] = DryDisable.toFloat();
              preferences.end();
              EEPROMbusy = false;
              break;

            case 2:
              Serial.println(DRYSETTING);
              EEPROMbusy = true;
              preferences.begin("my-app", false);
              preferences.putString("DRYC", DRYSETTING);
              DRYC_ARRAY[0] = ContactFunctionListIndex;
              DRYC_ARRAY[1] = OperationListIndex;
              DRYC_ARRAY[2] = DryEnable.toFloat();
              DRYC_ARRAY[3] = DryDisable.toFloat();
              preferences.end();
              EEPROMbusy = false;
              break;

            case 3:
              Serial.println(DRYSETTING);
              EEPROMbusy = true;
              preferences.begin("my-app", false);
              preferences.putString("DRYD", DRYSETTING);
              DRYD_ARRAY[0] = ContactFunctionListIndex;
              DRYD_ARRAY[1] = OperationListIndex;
              DRYD_ARRAY[2] = DryEnable.toFloat();
              DRYD_ARRAY[3] = DryDisable.toFloat();
              preferences.end();
              EEPROMbusy = false;
              break;



            default:
              break;
          }


          Serial.println("saved");
          FooterEvent = 38;
          action_done = true;

        }

        if (!action_done && y >= 62 && y <= 112 && x >= 241  && x <= 480) {

          PageNumber = 23;
          Serial.println("numpadopen");
          action_done = true;
          DryEnableStatus = true;
          DryDisableStatus = false;
        }


        if (!action_done && y >= 114 && y <= 164 && x >= 241  && x <= 480) {

          PageNumber = 23;
          Serial.println("numpadopen");
          action_done = true;
          DryEnableStatus = false;
          DryDisableStatus = true;
        }

        if (!action_done && y >= 62 && y <= 112 && x >= 0  && x <= 239) {
          ContactNameListIndex++;
          ContactNameListIndex = ContactNameListIndex % 4;
          Serial.println("ContactName is changed");
          action_done = true;

        }



        if (!action_done && y >= 114 && y <= 164 && x >= 0  && x <= 239) {
          OperationListIndex++;
          OperationListIndex = OperationListIndex % 2;
          Serial.println("ContactCondition is changed");
          action_done = true;

        }

        if (!action_done && y >= 166 && y <= 216 && x >= 0  && x <= 480) {
          ContactFunctionListIndex++;
          ContactFunctionListIndex = ContactFunctionListIndex % 9;
          Serial.println("ContactFunction is changed");
          action_done = true;

        }
      }



      if (PageNumber == 19) {

        if (!action_done && y >= 218 && y <= 268 && x >= 0 && x <= 480) {
          Serial.println("Open Language List");
          FooterEvent = 61;
          PageNumber = 27;
          action_done = true;

        }




        if (!action_done && y >= 62 && y <= 112 && x >= 0 && x <= 239) {
          Serial.println("Buzzer Toggle");
          FooterEvent = 16;
          mute = !mute;

          EEPROMbusy = true;
          preferences.begin("my-app", false);
          delay(10);
          preferences.putBool("mute", mute);
          preferences.end();
          EEPROMbusy = false;




          action_done = true;
          delay(250);

        }

        if (!action_done && y >= 62 && y <= 122 && x >= 241 && x <= 480) {
          Serial.println("Safety Toggle");
          FooterEvent = 15;


          if (!Safety) {
            if ((BMS.current - 30000) * 0.1 < 2 && (BMS.current - 30000) * 0.1 > -2)
            {
              Safety = true;
              SafetyTurnOnApproved = true;
            }

            else {
              SafetyAlarm = true;

              FooterEvent = 1;
              Serial.println("Safety Alarm");
              Serial.println(abs(BMS.current - 30000) * 0.1);
            }
          }
          else {
            Safety = false;
            SafetyTurnOffApproved = true;
          }
          action_done = true;
          delay(250);
        }

        if (!action_done && y >= 124 && y <= 184 && x >= 0 && x <= 239) {
          Serial.println("OPEN CAN ID NUMPAD");

          FooterEvent = 54;
          action_done = true;
          PageNumber = 25;
        }

        if (!action_done && y >= 186 && y <= 236 && x >= 0 && x <= 480) {
          Serial.println("now it s setting can id");

          FooterEvent = 56;
          action_done = true;
          SetCANID = true;
        }





        //SetCANID



      }




      ///////////////////////////////////////////

      /////innnnnnnnn

      if (PageNumber == 30) {


        if (!action_done && y >= 62 && y <= 112 && x >= 0 && x <= 239) {
          Serial.println("Inverter Brands toggled");
          action_done = true;
          delay(250);


          if (indexINVERTERSArray < 16) {
            indexINVERTERSArray++;
          }
          else {
            indexINVERTERSArray = 0;
          }
        }

        if (!action_done && y >= 124 && y <= 184 && x >= 0 && x <= 239) {
          Serial.println("comm method toggled");

          action_done = true;

          if (indexCOMMArray < 1) {
            indexCOMMArray++;
          }
          else {
            indexCOMMArray = 0;
          }

        }


        if (!action_done && y >= 186 && y <= 236 && x >= 0 && x <= 480) {
          Serial.println("setting inverter comm");

          action_done = true;
          METHOD = indexCOMMArray;
          TYPE = indexINVERTERSArray;
          SetCOMM = true;


        }





      }


      ////////////////////////////////////////////





      if (PageNumber == 26) { // "AlarmLog"


        if (!action_done && y >= 200 && y <= 320 && x >= 440 && x <= 480) {
          Serial.println("table goes right");
          action_done = true;
          pulled = true;
        }


        if (!action_done && y >= 200 && y <= 320 && x >= 0 && x <= 40) {
          Serial.println("table goes left");
          action_done = true;
          pulled = false;

        }


        if (!action_done && y >= 60 && y <= 100 && x >= 40 && x <= 400) {
          Serial.println("table goes up");
          action_done = true;
          pulled2 = false;

        }

        if (!action_done && y >= 200 && y <= 320 && x >= 40 && x <= 400) {
          Serial.println("table goes down");
          action_done = true;
          pulled2 = true;

        }


      }

      if (PageNumber == 27) { // "Language Select"
        if (!action_done && y >= 62 && y <= 112 && x >= 80 && x <= 400) { //open numpad for SOC
          Serial.println("Turkish");

          action_done = true;
          indexLanguageArray = 1;
          FooterEvent = 63;
          PageNumber = 19;



          preferences.begin("my-app", false);
          preferences.putInt("indexLang", indexLanguageArray);
          preferences.end();

        }

        if (!action_done && y >= 114 && y <= 164 && x >= 80 && x <= 400) { //open numpad for SOC
          Serial.println("English");

          action_done = true;
          indexLanguageArray = 0;
          FooterEvent = 63;
          PageNumber = 19;

          preferences.begin("my-app", false);
          preferences.putInt("indexLang", indexLanguageArray);
          preferences.end();
        }

        if (!action_done && y >= 166 && y <= 216 && x >= 80 && x <= 400) { //open numpad for SOC
          Serial.println("German");

          action_done = true;
          indexLanguageArray = 2;
          FooterEvent = 63;
          PageNumber = 19;

          preferences.begin("my-app", false);
          preferences.putInt("indexLang", indexLanguageArray);
          preferences.end();
        }

        if (!action_done && y >= 218 && y <= 268 && x >= 80 && x <= 400) { //open numpad for SOC
          Serial.println("French");

          action_done = true;
          indexLanguageArray = 3;
          FooterEvent = 63;
          PageNumber = 19;


          preferences.begin("my-app", false);
          preferences.putInt("indexLang", indexLanguageArray);
          preferences.end();
        }
      }




      //Simulate
      if (PageNumber == 28) {
        if (!action_done && y >= 60 && y <= 160 && x >= 326 && x <= 480) { //open numpad for SOC
          Serial.println("Simulation Status toggled");

          SimulationStatus = !SimulationStatus;


          if (SimulationStatus) {
            FooterEvent = 66;


            DRYA_ARRAY_TEMP[0] = DRYA_ARRAY[0];
            DRYA_ARRAY_TEMP[1] = DRYA_ARRAY[1];
            DRYA_ARRAY_TEMP[2] = DRYA_ARRAY[2];
            DRYA_ARRAY_TEMP[3] = DRYA_ARRAY[3];


            DRYB_ARRAY_TEMP[0] = DRYB_ARRAY[0];
            DRYB_ARRAY_TEMP[1] = DRYB_ARRAY[1];
            DRYB_ARRAY_TEMP[2] = DRYB_ARRAY[2];
            DRYB_ARRAY_TEMP[3] = DRYB_ARRAY[3];


            DRYC_ARRAY_TEMP[0] = DRYC_ARRAY[0];
            DRYC_ARRAY_TEMP[1] = DRYC_ARRAY[1];
            DRYC_ARRAY_TEMP[2] = DRYC_ARRAY[2];
            DRYC_ARRAY_TEMP[3] = DRYC_ARRAY[3];


            DRYD_ARRAY_TEMP[0] = DRYD_ARRAY[0];
            DRYD_ARRAY_TEMP[1] = DRYD_ARRAY[1];
            DRYD_ARRAY_TEMP[2] = DRYD_ARRAY[2];
            DRYD_ARRAY_TEMP[3] = DRYD_ARRAY[3];




            ///////////////////////////


            DRYA_ARRAY[0] = "0";
            DRYA_ARRAY[1] = "1";
            DRYA_ARRAY[2] = "5";
            DRYA_ARRAY[3] = "20";

            DRYB_ARRAY[0] = "0";
            DRYB_ARRAY[1] = "0";
            DRYB_ARRAY[2] = "60";
            DRYB_ARRAY[3] = "70";

            DRYC_ARRAY[0] = "1";
            DRYC_ARRAY[1] = "0";
            DRYC_ARRAY[2] = "0";
            DRYC_ARRAY[3] = "100";

            DRYD_ARRAY[0] = "2";
            DRYD_ARRAY[1] = "1";
            DRYD_ARRAY[2] = "10";
            DRYD_ARRAY[3] = "5";

          }
          else {
            FooterEvent = 67;

            DRYA_ARRAY[0] = DRYA_ARRAY_TEMP[0];
            DRYA_ARRAY[1] = DRYA_ARRAY_TEMP[1];
            DRYA_ARRAY[2] = DRYA_ARRAY_TEMP[2];
            DRYA_ARRAY[3] = DRYA_ARRAY_TEMP[3];

            DRYB_ARRAY[0] = DRYB_ARRAY_TEMP[0];
            DRYB_ARRAY[1] = DRYB_ARRAY_TEMP[1];
            DRYB_ARRAY[2] = DRYB_ARRAY_TEMP[2];
            DRYB_ARRAY[3] = DRYB_ARRAY_TEMP[3];


            DRYC_ARRAY[0] = DRYC_ARRAY_TEMP[0];
            DRYC_ARRAY[1] = DRYC_ARRAY_TEMP[1];
            DRYC_ARRAY[2] = DRYC_ARRAY_TEMP[2];
            DRYC_ARRAY[3] = DRYC_ARRAY_TEMP[3];


            DRYD_ARRAY[0] = DRYD_ARRAY_TEMP[0];
            DRYD_ARRAY[1] = DRYD_ARRAY_TEMP[1];
            DRYD_ARRAY[2] = DRYD_ARRAY_TEMP[2];
            DRYD_ARRAY[3] = DRYD_ARRAY_TEMP[3];
          }
          action_done = true;
        }
      }





      if (PageNumber == 4) {
        if (!action_done && y >= 218 && y <= 268 && x >= 0 && x <= 480) { //open numpad for SOC
          Serial.println("resetNettwork");

          wifiManager.resetSettings();
          Serial.println("forget password");
          delay(1000);
          action_done = true;
        }
      }

      if (PageNumber == 6) {
        if (!action_done && y >= 218 && y <= 268 && x >= 0 && x <= 480) { //open numpad for SOC

          MaxTemp1 = 0;
          MaxTemp2 = 0;
          MaxVoltage = 0;
          MinVoltage = 500;
          MaxCurrent = -500;
          MinCurrent = 500;

          Serial.println("Statistics are resetted");

          delay(1000);
          action_done = true;
        }
      }

      if (PageNumber == 20) {
        if (!action_done && y >= 62 && y <= 164 && x >= 0 && x <= 102) { //open numpad for SOC
          Serial.println("calibration is increased");

          if (CalibrationValue <= 400) {
            CalibrationValue = CalibrationValue + 10;
          }
          action_done = true;
        }

        if (!action_done && y >= 166 && y <= 266 && x >= 0 && x <= 102) { //open numpad for SOC
          Serial.println("calibration is decreased");

          if (CalibrationValue >= -400) {
            CalibrationValue = CalibrationValue - 10;
          }
          action_done = true;
        }


        if (!action_done && y >= 62 && y <= 164 && x >= 378 && x <= 480) { //open numpad for SOC
          Serial.println("calibration is done");

          CURRENT_CAL = true;
          CalibrationStatus = 2;
          FooterEvent = 18;
          action_done = true;
        }

        if (!action_done && y >= 166 && y <= 266 && x >= 378 && x <= 480) { //open numpad for SOC
          CalibrationStatus = 2;

          CURRENT_ZERO_CAL = true;
          FooterEvent = 18;

          action_done = true;
        }

      }


      if (PageNumber == 23) {
        if (!action_done && y >= 65 && y <= 127 && x >= 390 && x <= 465 && numpadValue.length() < 9) { //7 BUTTON

          Serial.println("OK");
          action_done = true;



          if (DryEnableStatus) {
            DryEnable = numpadValue;
            DryEnableStatus = false;
            Serial.println("Enable:" + numpadValue);
            numpadValue = "";
          }

          else {
            DryDisable = numpadValue;
            DryDisableStatus = false;
            Serial.println("Disable:" + numpadValue);
            numpadValue = "";
          }

          PageNumber = 22;
        }

        if (!action_done && y >= 132 && y <= 194 && x >= 10 && x <= 65 && numpadValue.length() < 8 ) { //7 BUTTON

          Serial.println("1");
          action_done = true;
          numpadValue = numpadValue + "1";
        }

        if (!action_done && y >= 199 && y <= 261 && x >= 10 && x <= 65 && numpadValue.length() < 8 ) { //7 BUTTON
          Serial.println("6");

          action_done = true;
          numpadValue = numpadValue + "6";
        }


        if (!action_done && y >= 199 && y <= 261 && x >= 70 && x <= 125 && numpadValue.length() < 8) { //7 BUTTON
          Serial.println("7");

          action_done = true;
          numpadValue = numpadValue + "7";
        }

        if (!action_done && y >= 132 && y <= 194 && x >= 70 && x <= 125 && numpadValue.length() < 8) { //7 BUTTON
          Serial.println("2");
          action_done = true;

          numpadValue = numpadValue + "2";
        }

        if (!action_done && y >= 199 && y <= 261 && x >= 130 && x <= 185 && numpadValue.length() < 8) { //7 BUTTON
          Serial.println("8");

          action_done = true;
          numpadValue = numpadValue + "8";
        }


        if (!action_done && y >= 132 && y <= 194 && x >= 130 && x <= 185 && numpadValue.length() < 8) { //7 BUTTON
          Serial.println("3");
          action_done = true;

          numpadValue = numpadValue + "3";
        }

        if (!action_done && y >= 199 && y <= 261 && x >= 190 && x <= 245 && numpadValue.length() < 8) { //7 BUTTON
          Serial.println("9");
          action_done = true;

          numpadValue = numpadValue + "9";
        }

        if (!action_done && y >= 132 && y <= 194 && x >= 190 && x <= 245 && numpadValue.length() < 8) { //7 BUTTON
          Serial.println("4");
          action_done = true;

          numpadValue = numpadValue + "4";
        }


        if (!action_done && y >= 199 && y <= 261 && x >= 250 && x <= 305 && numpadValue.length() < 8) { //7 BUTTON
          Serial.println("0");
          action_done = true;

          numpadValue = numpadValue + "0";
        }

        if (!action_done && y >= 132 && y <= 194 && x >= 250 && x <= 305 && numpadValue.length() < 8) { //7 BUTTON
          Serial.println("5");
          action_done = true;

          numpadValue = numpadValue + "5";
        }

        if (!action_done && y >= 199 && y <= 261 && x >= 310 && x <= 385 ) { //7 BUTTON
          Serial.println(".");

          action_done = true;
          numpadValue = numpadValue + ".";
        }

        if (!action_done && y >= 132 && y <= 194 && x >= 310 && x <= 385 ) { //7 BUTTON
          Serial.println("BACK");

          action_done = true;
          numpadValue = "";
          PageNumber = 22;
        }


        if (!action_done && y >= 199 && y <= 261 && x >= 390 && x <= 465 ) { //7 BUTTON
          Serial.println("CLR");
          action_done = true;

          numpadValue = "";
        }

        if (!action_done && y >= 132 && y <= 194 && x >= 390 && x <= 465 ) { //7 BUTTON
          Serial.println("DEL");
          action_done = true;

          int numpadLength = numpadValue.length();
          numpadValue.remove(numpadLength - 1, 1);
        }
      }



      if (PageNumber == 25) {
        if (!action_done && y >= 65 && y <= 127 && x >= 390 && x <= 465 && numpadValue.length() < 9) { //7 BUTTON
          Serial.println("OK");
          action_done = true;

          SettedCANID = numpadValue.toInt();
          Serial.println("Setted CANID:" + numpadValue);
          numpadValue = "";

          PageNumber = 19;
        }

        if (!action_done && y >= 132 && y <= 194 && x >= 10 && x <= 65 && numpadValue.length() < 8 ) { //7 BUTTON
          Serial.println("1");

          action_done = true;
          numpadValue = numpadValue + "1";
        }

        if (!action_done && y >= 199 && y <= 261 && x >= 10 && x <= 65 && numpadValue.length() < 8 ) { //7 BUTTON
          Serial.println("6");
          action_done = true;

          numpadValue = numpadValue + "6";
        }


        if (!action_done && y >= 199 && y <= 261 && x >= 70 && x <= 125 && numpadValue.length() < 8) { //7 BUTTON
          Serial.println("7");

          action_done = true;
          numpadValue = numpadValue + "7";
        }

        if (!action_done && y >= 132 && y <= 194 && x >= 70 && x <= 125 && numpadValue.length() < 8) { //7 BUTTON
          Serial.println("2");
          action_done = true;

          numpadValue = numpadValue + "2";
        }

        if (!action_done && y >= 199 && y <= 261 && x >= 130 && x <= 185 && numpadValue.length() < 8) { //7 BUTTON
          Serial.println("8");
          action_done = true;

          numpadValue = numpadValue + "8";
        }


        if (!action_done && y >= 132 && y <= 194 && x >= 130 && x <= 185 && numpadValue.length() < 8) { //7 BUTTON
          Serial.println("3");

          action_done = true;
          numpadValue = numpadValue + "3";
        }

        if (!action_done && y >= 199 && y <= 261 && x >= 190 && x <= 245 && numpadValue.length() < 8) { //7 BUTTON
          Serial.println("9");
          action_done = true;

          numpadValue = numpadValue + "9";
        }

        if (!action_done && y >= 132 && y <= 194 && x >= 190 && x <= 245 && numpadValue.length() < 8) { //7 BUTTON
          Serial.println("4");
          action_done = true;

          numpadValue = numpadValue + "4";
        }


        if (!action_done && y >= 199 && y <= 261 && x >= 250 && x <= 305 && numpadValue.length() < 8) { //7 BUTTON
          Serial.println("0");
          action_done = true;

          numpadValue = numpadValue + "0";
        }

        if (!action_done && y >= 132 && y <= 194 && x >= 250 && x <= 305 && numpadValue.length() < 8) { //7 BUTTON
          Serial.println("5");
          action_done = true;

          numpadValue = numpadValue + "5";
        }

        if (!action_done && y >= 199 && y <= 261 && x >= 310 && x <= 385 ) { //7 BUTTON
          Serial.println(".");
          action_done = true;

          numpadValue = numpadValue + ".";
        }

        if (!action_done && y >= 132 && y <= 194 && x >= 310 && x <= 385 ) { //7 BUTTON
          Serial.println("BACK");
          action_done = true;

          numpadValue = "";
          PageNumber = 22;
        }


        if (!action_done && y >= 199 && y <= 261 && x >= 390 && x <= 465 ) { //7 BUTTON
          Serial.println("CLR");
          action_done = true;
          numpadValue = "";

        }

        if (!action_done && y >= 132 && y <= 194 && x >= 390 && x <= 465 ) { //7 BUTTON
          Serial.println("DEL");
          action_done = true;
          int numpadLength = numpadValue.length();
          numpadValue.remove(numpadLength - 1, 1);

        }
      }


      if (PageNumber == 29) {

        if (!action_done && y >= 62 && y <= 165 && x >= 36 && x <= 171) {
          Serial.println("Inverter Selection");
          FooterEvent = 2;
          action_done = true;

          PageNumber = 30;




        }
      }


      if (touch_iteration < 100)
        touch_iteration++;
    }
    else {
      touch_iteration = 0;
      action_done = false;
      for (int i = 0; i < 100; i++) {
        TouchArray[i] = 0;
      }
    }
  }
}



void loop() {




  //Serial.print("OpSec:");
  //Serial.println(ceil(OpSec));

  // Serial.print("OpMin:");
  // Serial.println(OpMin);


  //ENCONNECT 11000
  //BT 9000
  //FIREBASE 7000
  //BMS 6000
  //SDCAN 6500
  //TOUCH 4000
  //UDPTEST 6000

  //    Serial.println("ENCONNECT:" + String(uxTaskGetStackHighWaterMark(ENCONNECT) * 100 / 7000) + " %");
  //    Serial.println("BMS_COMM:" + String(uxTaskGetStackHighWaterMark(BMS_COMM) * 100 / 5000) + " %");
  //    Serial.println("SDCANBUS:" + String(uxTaskGetStackHighWaterMark(SDCANBUS) * 100 / 7500) + " %");
  //    Serial.println("UDPTEST:" + String(uxTaskGetStackHighWaterMark(UDPTEST) * 100 / 8000) + " %");
  //    Serial.println("FIREBASE:" + String(uxTaskGetStackHighWaterMark(FIREBASE) * 100 / 9000) + " %");
  //    Serial.println("TOUCH_SCREEN:" + String(uxTaskGetStackHighWaterMark(TOUCH_SCREEN) * 100 / 4000) + " %");
  //     Serial.println("BT:" + String(uxTaskGetStackHighWaterMark(BT) * 100 / 9000) + " %");
  //  Serial.print( "FreeRA");
  //  Serial.println(ConvBinUnits(ESP.getFreeHeap(), 3));


  Serial.print("FreeMemory:");
  Serial.print(ConvBinUnits(ESP.getFreeHeap(), 3));
  Serial.print(",");
  Serial.print("SD_Card_Status:");
  Serial.print(control_sd * 100);
  Serial.print(",");
  Serial.print("Firebase_Status:");
  Serial.print(firebaseReady * 100);
  Serial.print(",");
  Serial.print("CAN_Status:");
  Serial.print(canbusReady * 100);
  Serial.print(",");
  Serial.print("Seconds:");
  Serial.print(OpSec);
  Serial.print(",");
  Serial.print("Minutes:");
  Serial.print(OpMin);
  Serial.print(",");
  Serial.print("Hours:");
  Serial.print(OpHour);
  Serial.print(",");
  Serial.print("GPS:");
  Serial.println(GPSFetched);
  //  Serial.print("BMS_COUNTER:");
  //  Serial.print(bmsCounter);
  //  Serial.print("RTC_YEAR:");
  //  Serial.print((String(BMS.year).substring(2)).toInt());
  //  Serial.print("RTC_MONTH:");
  //  Serial.print(String(BMS.month).toInt());
  //  Serial.print("RTC_DAY:");
  //  Serial.print(String(BMS.day).toInt());
  //  Serial.print("RTC_HOUR:");
  //  Serial.print(String(BMS.when).toInt());
  //  Serial.print("RTC_MIN:");
  //  Serial.print(String(BMS.point).toInt());
  //  Serial.print("RTC_SEC:");
  //  Serial.println(String(BMS.second).toInt());







  //  Serial.println( "SDTimer:" + String(SDTimer));
  //  Serial.println( "FirebaseTimer:" + String(FirebaseTimer));
  //  Serial.println("Update:");
  //  Serial.println(UpdateNextRestart);
  //  Serial.print("SleepCounter:");
  //  Serial.println(SleepCounter);

  /////////////////////////////////
  if (WiFi.status() == WL_CONNECTED  && UpdateNextRestart) {
    Serial.println(F("System is preparing for update..."));

    ledcWrite(ledChannel, 255);
    drawBanner(TimeString, DateString, "Update", wifistatus, alarmNo, UpdateAvailable);
    drawUpload();
    drawFooter();


    delay(500);

    esp_task_wdt_init(60, false);
    //CHECK FOR UPDATE
    for (int i = 0; i < 3; i++) {
      //Serial.println("UNR Before Update:" + String(UpdateNextRestart));
      if (FirmwareVersionCheck()) {
        if (UpdateNextRestart) {
          delay(500);

          EEPROMbusy = true;
          preferences.begin("my-app", false);
          preferences.putBool("UNR", false);
          preferences.end();
          EEPROMbusy = false;
          Serial.println(F("Firmware will update"));
          //Serial.println("UNR After Update:" + String(UpdateNextRestart));
          delay(100);
          firmwareUpdate();
          UpdateNextRestart = false;
          PageNumber = 0;
        }
        else {
          Serial.println(F("Update Available"));
          UpdateAvailable = true;
          updateIcon = true;
        }
        break;
      }
    }

  }


  //UDP COMMAND MANAGER


  switch (UDP_COMMAND_INDEX) {

    case 0:
      //SET HCC&HCD BMS////////////////////////////////
      param_start2 = UDPRequest.indexOf("SETHCC");
      param_end2 = UDPRequest.indexOf("/");
      if (param_start2 != -1 && param_end2 != -1) {
        UDPprocessor = UDPRequest.substring(param_start2 + 6, param_end2);
        HCC = UDPprocessor.toInt();
      }///////////////////////////////////////////////

      param_start2 = UDPRequest.indexOf("SETHCD");
      param_end2 = UDPRequest.indexOf("#");
      if (param_start2 != -1 && param_end2 != -1) {
        UDPprocessor = UDPRequest.substring(param_start2 + 6, param_end2);
        HCD = UDPprocessor.toFloat();
        UDPRequest = "";
        Serial.print(HCC);
        Serial.print("/");
        Serial.println(HCD);
        HCC_HCD = true;
        UDP_COMMAND_INDEX = -1;
        Serial.print("*********************CURRENT******************");
      }///////////////////////////////////////////////
      break;




    case 1:

      //SET SUM_CALL BMS////////////////////////////////
      param_start2 = UDPRequest.indexOf("SETSUMHI");
      param_end2 = UDPRequest.indexOf("/");
      if (param_start2 != -1 && param_end2 != -1) {
        UDPprocessor = UDPRequest.substring(param_start2 + 8, param_end2);
        SUMHI = UDPprocessor.toFloat();


      }///////////////////////////////////////////////

      param_start2 = UDPRequest.indexOf("SETSUMLOW");
      param_end2 = UDPRequest.indexOf("#");
      if (param_start2 != -1 && param_end2 != -1) {
        UDPprocessor = UDPRequest.substring(param_start2 + 9, param_end2);
        SUMLOW = UDPprocessor.toFloat();
        UDPRequest = "";
        Serial.print(SUMHI);
        Serial.print("/");
        Serial.println(SUMLOW);
        SUM_CAL = true;
        UDP_COMMAND_INDEX = -1;
        Serial.print("*********************SUM******************");

      }///////////////////////////////////////////////

      break;


    case 2:

      //SET HVC_LVC BMS////////////////////////////////
      param_start2 = UDPRequest.indexOf("SETHVC");
      param_end2 = UDPRequest.indexOf("/");
      if (param_start2 != -1 && param_end2 != -1) {
        UDPprocessor = UDPRequest.substring(param_start2 + 6, param_end2);
        HVC = UDPprocessor.toFloat();
        //SerialBT.println(HCC);

      }///////////////////////////////////////////////


      param_start2 = UDPRequest.indexOf("SETLVC");
      param_end2 = UDPRequest.indexOf("#");
      if (param_start2 != -1 && param_end2 != -1) {
        UDPprocessor = UDPRequest.substring(param_start2 + 6, param_end2);
        LVC = UDPprocessor.toFloat();
        UDPRequest = "";
        Serial.print(HVC);
        Serial.print("/");
        Serial.println(LVC);
        HVC_LVC = true;
        UDP_COMMAND_INDEX = -1;
        Serial.print("*********************CELL******************");
      }///////////////////////////////////////////////

      break;


    case 3:

      //SET HTC_HTD BMS////////////////////////////////
      param_start2 = UDPRequest.indexOf("SETHTC");
      param_end2 = UDPRequest.indexOf("/");
      if (param_start2 != -1 && param_end2 != -1) {
        UDPprocessor = UDPRequest.substring(param_start2 + 6, param_end2);
        HTC = UDPprocessor.toInt();


      }///////////////////////////////////////////////


      param_start2 = UDPRequest.indexOf("SETHTD");
      param_end2 = UDPRequest.indexOf("#");
      if (param_start2 != -1 && param_end2 != -1) {
        UDPprocessor = UDPRequest.substring(param_start2 + 6, param_end2);
        HTD = UDPprocessor.toInt();
        UDPRequest = "";
        Serial.print(HTC);
        Serial.print("/");
        Serial.println(HTD);
        HTC_HTD = true;
        UDP_COMMAND_INDEX = -1;
        Serial.print("*********************TEMP******************");
      }///////////////////////////////////////////////
      break;



    case 4:

      //SET energy records////////////////////////////////
      param_start2 = UDPRequest.indexOf("SETCHARGE");
      param_end2 = UDPRequest.indexOf("/");
      if (param_start2 != -1 && param_end2 != -1) {
        UDPprocessor = UDPRequest.substring(param_start2 + 9, param_end2);
        SetCharge = UDPprocessor.toFloat();


      }///////////////////////////////////////////////


      param_start2 = UDPRequest.indexOf("SETDISCHARGE");
      param_end2 = UDPRequest.indexOf("#");
      if (param_start2 != -1 && param_end2 != -1) {
        UDPprocessor = UDPRequest.substring(param_start2 + 12, param_end2);
        SetDischarge = UDPprocessor.toFloat();


        UDPRequest = "";
        Serial.print(SetCharge);
        Serial.print("/");
        Serial.println(SetDischarge);
        SetEnergies = true;
        UDP_COMMAND_INDEX = -1;
        Serial.print("*********************ENERGY******************");
      }///////////////////////////////////////////////

      break;



    case 5:

      //SET SOC_ALARM BMS////////////////////////////////

      param_start2 = UDPRequest.indexOf("SETSHI");
      param_end2 = UDPRequest.indexOf("/");
      if (param_start2 != -1 && param_end2 != -1) {
        UDPprocessor = UDPRequest.substring(param_start2 + 6, param_end2);
        SOCHI = UDPprocessor.toFloat();
        //SerialBT.println(HCC);

      }///////////////////////////////////////////////

      param_start2 = UDPRequest.indexOf("SETSLOW");
      param_end2 = UDPRequest.indexOf("#");
      if (param_start2 != -1 && param_end2 != -1) {
        UDPprocessor = UDPRequest.substring(param_start2 + 7, param_end2);
        SOCLOW = UDPprocessor.toFloat();

        Serial.println("SOCALARM");
        Serial.println(SOCLOW);
        Serial.println(SOCHI);
        UDPRequest = "";
        Serial.print(SOCHI);
        Serial.print("/");
        Serial.println(SOCLOW);
        SOC_ALARM = true;
        UDP_COMMAND_INDEX = -1;
        Serial.print("*********************SOC******************");
      }///////////////////////////////////////////////
      break;



    case 6:
      //SET HCC&HCD BMS////////////////////////////////
      param_start2 = UDPRequest.indexOf("RESETBMS");
      param_end2 = UDPRequest.indexOf("#");
      if (param_start2 != -1 && param_end2 != -1) {

        UDPRequest = "";
        resetBMS = true;
        UDP_COMMAND_INDEX = -1;
        Serial.print("*********************RESET BMS******************");

      }///////////////////////////////////////////////
      break;


    default:
      //Serial.println("No UDP Command executed");
      break;

  }


  currentMillisUI = millis();

  // Serial.println("Loop Timer:" + String(currentMillisUI - previousMillisUI));

  // Serial.println("OpsEC:" + String(OpSec));

  OpSec = OpSec + (currentMillisUI - previousMillisUI) * 0.001;
  if (OpSec > 60) {
    OpSec = 0;
    OpMin = OpMin + 1;
    if (OpMin > 60) {
      OpMin = 0;
      OpHour = OpHour + 1;
      if (OpHour > 24) {
        OpHour = 0;
        OpDay = OpDay + 1;
      }
    }
  }





  if (currentMillisUI - previousMillisUI >= intervalUI) {



    //    Serial.print("SleepCounter:");
    //    Serial.println(SleepCounter);

    // BACKLIGHT COTROL
    if (SleepCounter < 3600) {
      SleepCounter++;
      if (SleepCounter == 5) {
        ledcWrite(ledChannel, 100);
      }
      if (SleepCounter == 59) {
        ledcWrite(ledChannel, 5);
      }

      if (SleepCounter == 300 && PageNumber != 15) {
        PageNumber = 0;
        dashboard_page = 0;
      }

      if (SleepCounter == 10) {
        SafetyAlarm = false;

      }
      else {

      }
    }

    else {
      SleepCounter = 0;
    }
    if (SleepCounter == 1) {
      ledcWrite(ledChannel, 255);
    }


    if (SimMode) {
      SIM_SOC = 95.4 + random(1, 10) * 0.1;
      SIM_MAXCELL = 3.7 + random(1, 10) * 0.01;
      SIM_MIN_CELL = 3.4 + random(1, 10) * 0.01;
      SIM_SUM = SimVolt + random(1, 10) * 0.01;
      SIM_CURRENT = 0;
      SIM_TEMP = 23 + random(1, 10) * 0.1;
    }






    //Page manager//

    switch (PageNumber) {

      case 0:
        LastPageNumber = 1;

        drawBanner(TimeString, DateString, "Dashboard", wifistatus, alarmNo, UpdateAvailable);


        if (SimMode) {
          drawDashboard(SIM_SOC, SIM_MAXCELL, SIM_MIN_CELL,
                        SIM_MAXCELL - SIM_MIN_CELL,
                        SIM_SUM, SIM_CURRENT, SIM_TEMP);
        }
        else {
          drawDashboard(BMS.SOC * 0.1, BMS.max_cell_volt * 0.001, BMS. min_cell_volt * 0.001,
                        BMS.max_cell_volt * 0.001 - BMS. min_cell_volt * 0.001, BMS.sum_voltage * 0.1,
                        (BMS.current - 30000) * 0.1, BMS.max_cell_temp - 40);
        }

        drawFooter();


        break;

      case 1:
        drawBanner(TimeString, DateString, "Settings", wifistatus, alarmNo, UpdateAvailable);
        LastPageNumber = 0  ;
        drawSettings();    //MainMenu
        drawFooter();
        break;

      case 2:
        drawBanner(TimeString, DateString, "Features", wifistatus, alarmNo, UpdateAvailable);
        LastPageNumber = 1;
        drawFeatures();  //second main menu
        drawFooter();
        break;

      case 3:
        drawBanner(TimeString, DateString, "Cell Monitoring", wifistatus, alarmNo, UpdateAvailable);
        drawCellSirius();
        LastPageNumber = 1;
        drawFooter();
        break;

      case 4:
        drawBanner(TimeString, DateString, "Network Settings", wifistatus, alarmNo, UpdateAvailable);
        drawNetwork();
        LastPageNumber = 1;
        drawFooter();
        break;

      case 5:
        drawBanner(TimeString, DateString, "Dry Contacts", wifistatus, alarmNo, UpdateAvailable);
        drawDryContact();
        LastPageNumber = 1;
        drawFooter();
        break;

      case 6 :
        drawBanner(TimeString, DateString, "Daily Statistics", wifistatus, alarmNo, UpdateAvailable);
        drawStatistics();
        LastPageNumber = 1;
        drawFooter();
        break;




      case 15:
        ledcWrite(ledChannel, 255);
        drawBanner(TimeString, DateString, "Update", wifistatus, alarmNo, UpdateAvailable);
        drawUpload();
        drawFooter();
        break;

      case 16:
        drawBanner(TimeString, DateString, "Product Manual", wifistatus, alarmNo, UpdateAvailable);
        drawBarcode();
        LastPageNumber = 2;
        drawFooter();
        break;

      case 17:
        drawBanner(TimeString, DateString, "Barcode", wifistatus, alarmNo, UpdateAvailable);
        drawSystemDetails();
        LastPageNumber = 1;
        drawFooter();
        break;

      case 18:
        drawBanner(TimeString, DateString, "System Setup Menu", wifistatus, alarmNo, UpdateAvailable);
        drawSystem(); //3 main menu
        LastPageNumber = 2;
        drawFooter();
        break;

      case 19:
        drawBanner(TimeString, DateString, "System Setup", wifistatus, alarmNo, UpdateAvailable);
        drawSystemSetup();
        LastPageNumber = 18;
        drawFooter();
        break;

      case 20:
        drawBanner(TimeString, DateString, "Current calibration", wifistatus, alarmNo, UpdateAvailable);
        // drawCurrentCalibration();
        drawFooter();
        LastPageNumber = 18;
        break;


      case 21:
        drawBanner(TimeString, DateString, "Barcode", wifistatus, alarmNo, UpdateAvailable);
        drawMonitoringBarcode();
        drawFooter();
        LastPageNumber = 18;
        break;

      case 22:
        drawBanner(TimeString, DateString, "Barcode", wifistatus, alarmNo, UpdateAvailable);
        drawWriteDry();
        LastPageNumber = 18;
        drawFooter();
        break;


      case 23:
        drawBanner(TimeString, DateString, "Barcode", wifistatus, alarmNo, UpdateAvailable);
        drawNumpadDry();
        LastPageNumber = 22;
        drawFooter();
        break;


      case 24:
        drawBanner(TimeString, DateString, "Barcode", wifistatus, alarmNo, UpdateAvailable);
        drawSystemStats();
        LastPageNumber = 18;
        drawFooter();
        break;

      case 25:
        drawBanner(TimeString, DateString, "Barcode", wifistatus, alarmNo, UpdateAvailable);
        drawNumpadDry();
        LastPageNumber = 19;
        drawFooter();
        break;

      case 26:
        drawBanner(TimeString, DateString, "Barcode", wifistatus, alarmNo, UpdateAvailable);
        drawAlarmLog();
        LastPageNumber = 18;
        drawFooter();
        break;

      case 27:
        drawBanner(TimeString, DateString, "Barcode", wifistatus, alarmNo, UpdateAvailable);
        //Serial.println("Page 27");
        drawLanguageList();
        LastPageNumber = 19;
        drawFooter();
        break;

      case 28:
        // Serial.println("Page 28");
        drawSimulation();
        LastPageNumber = 18;
        drawFooter();
        break;


      case 29:
        drawBanner(TimeString, DateString, "4. menu", wifistatus, alarmNo, UpdateAvailable);
        drawMenu4(); //4 main menu
        LastPageNumber = 3;
        drawFooter();
        break;


      case 30:
        drawBanner(TimeString, DateString, "InverterSetup", wifistatus, alarmNo, UpdateAvailable);
        drawInverterSetup();
        LastPageNumber = 29;
        drawFooter();
        break;


      default:
        PageNumber = 0;
        break;
    }


    previousMillisUI = currentMillisUI;
  }
  delay(100);

}


bool printLocalTime() {
  struct tm timeinfo;
  if (!getLocalTime(&timeinfo)) {
    Serial.println("Failed to obtain time");
    return false;
  }

  char timeHour[3];
  strftime(timeHour, 3, "%H", &timeinfo);
  strftime(timeMin, 3, "%M", &timeinfo);
  strftime(timeSec, 3, "%S", &timeinfo);
  strftime(timeDay, 3, "%d", &timeinfo);
  strftime(timeMonth, 3, "%m", &timeinfo);
  strftime(timeYear, 10, "%Y", &timeinfo);

  timeHourS = String(timeHour);
  timeMinS = String(timeMin);
  timeSecS = String(timeSec);

  timeDayS = String(timeDay);
  timeMonthS = String(timeMonth);
  timeYearS = String(timeYear);


}


void getBatteryParameters() {

  bmsCounter++;


  //

  uint8_t cellnumber [3] = {16, 0, 0};
  uint8_t tempnumber [3] = {1, 0, 0};
  //set_board_BMS_equ(uint8_t boardnum ,uint8_t *boardnumvolt,uint8_t *boardnumtemp,uint8_t *BMS_Set_equ)
  //bool set_board_BMS(uint8_t boardnum , uint8_t *boardnumvolt, uint8_t *boardnumtemp, uint8_t *BMS_Set)

  switch (CellMod) {

    case 0:
      cellnumber[0] = 13;
      break;

    case 1:
      cellnumber[0] = 16;
      break;

    case 2:
      cellnumber[0] = 8;
      break;

    case 3:
      cellnumber[0] = 4;
      break;


    case 4:
      cellnumber[0] = 7;
      Serial.println("7 CELLS");
      break;


    default:
      Serial.println("invalid no");
      break;
  }



  if (!boardSetted) {
    //    boardSetted = set_board_BMS_equ(1 , cellnumber, tempnumber, BMS_Set_equ);
    //    Serial.print("Board is setted FOR BMS EQU:" + String(boardSetted) + "-");
    //    Serial.println(cellnumber[0]);

    boardSetted = set_board_BMS(1 , cellnumber, tempnumber, BMS_Set);
    Serial.print("Board is setted FOR BMS:" + String(boardSetted) + "-");
    Serial.println(cellnumber[0]);
    boardSetted = true;
  }

  if (!ReadAllParameters) {

    if (!SafetyStartUp2) {

      if (RecoveryCounter > 60) {
        RecoveryCounter = 0;
        // Serial.println("Recovery is in the progresss......RecoveryCounter");
        resetBms();
        delay(500);
      }




      //bbbbbb






      //Auto Nominal Voltage

      if (BMS.sum_voltage * 0.1 > 40 && BMS.sum_voltage * 0.1 < 66) {
        NominalVoltage = 48;
      }
      else if (BMS.sum_voltage * 0.1 > 20 && BMS.sum_voltage * 0.1 < 29) {
        NominalVoltage = 24;
      }
      else if (BMS.sum_voltage * 0.1 > 7 && BMS.sum_voltage * 0.1 < 15) {
        NominalVoltage = 12;
      }
      else {
        NominalVoltage = 48;
      }





      switch (bmsCounter) {

        case 1:
          // Serial.println("ID 90 is requested...");
          BMS_recieve(0x90);
          delay(10);
          BMS_recieve(0x61);
          delay(10);
          Serial.println("rtc---------------------");
          Serial.println(BMS.year);
          Serial.println(BMS.month);
          Serial.println(BMS.day);
          Serial.println(BMS.when);
          Serial.println(BMS.point);
          Serial.println(BMS.second);

          Serial.println("rtc---------------------");


          // Serial.println("ID 90 is done...");

          break;

        case 2:

          // Serial.println("ID 92 is requested...");
          BMS_recieve(0x92);
          delay(10);
          //Serial.println("ID 92 is done...");
          break;


        case 3:
          // Serial.println("ID 93 is requested...");
          BMS_recieve(0x93);

          //          Serial.println("Charge:" + String(BMS.charge));
          //          Serial.println("Discharge:" + String(BMS.discharge));



          delay(10);
          // Serial.println("ID 93 is done...");
          break;


        case 4:
          // Serial.println("ID 98 is requested...");
          BMS_recieve(0x98);
          delay(10);
          // Serial.println("ID 98 is done...");
          break;


        case 5:
          // Serial.println("ID 53 is requested...");
          BMS_recieve(0x53);
          delay(10);
          //Serial.println("ID 53 is done...")
          break;


        case 6:
          //Serial.println("ID 91 is requested...");
          BMS_recieve(0x91);
          delay(10);
          //Serial.println("ID 91 is done...");
          break;


        case 7:
          //Serial.println("ID 95 is requested...");
          BMS_recieve(0x95);
          delay(10);
          //Serial.println("ID 95 is done...";



          //    for (int z = 0; z < 30; z++) {
          //      Serial.println("CellNo#" + String(z) + " Voltage:" + String(BMS.cell_voltage[z] * 0.001));
          //    }










          break;

        case 8:
          //Serial.println("ID 95 is requested...");
          BMS_recieve(0x52);
          delay(10);
          BMS_recieve(0x69);
          delay(10);
          //Serial.println("ID 95 is done...";



          //          Serial.println("CANBUS START");
          //          Serial.println(BMS.commname);
          //          Serial.println(BMS.commmeth);
          if (BMS.commname != "NONE") {
            canbusReady = true;
          }
          else {
            canbusReady = false;
          }
          //Serial.println("CANBUS END");

          //          Serial.println("cumilative_charge:");
          //          Serial.println(String(BMS.cumilative_charge));
          //
          //          Serial.println("cumilative_discharge:");
          //          Serial.println(String(BMS.cumilative_discharge));


          if (BMS.cumilative_charge != 0) {


            ////////////eeeeeeeeeeeeeeeeeeeeeeeeeee

            PreChargeEnergy = ChargeEnergy;
            PreDischargeEnergy = DischargeEnergy;

            //            Serial.println("");
            //            Serial.println("start");
            //            Serial.print("PreChargeEnergy:");
            //            Serial.println(PreChargeEnergy);
            //
            //            Serial.print("PreDischargeEnergy:");
            //            Serial.println(PreDischargeEnergy);


            ChargeEnergy = BMS.cumilative_charge  * 0.001 * NominalVoltage;
            DischargeEnergy = BMS.cumilative_discharge * 0.001 * NominalVoltage;
            ChargeFix = ChargeEnergy;
            DischargeFix = DischargeEnergy;

            //            Serial.print("ChargeEnergy:");
            //            Serial.println(ChargeEnergy);
            //
            //            Serial.print("DischargeEnergy:");
            //            Serial.println(DischargeEnergy);
            //            Serial.println("end");
            //            Serial.println("");


            if (PreChargeEnergy != ChargeEnergy && PreChargeEnergy != 0) {

              if (abs(ChargeEnergy - PreChargeEnergy) * 100 < 1000) {
                HourlyChargeEnergy = ((ChargeEnergy - PreChargeEnergy) * 100 + HourlyChargeEnergy);
              }
              else {
                // Serial.println("invalid!!!!!!!!!!!");
              }
            }



            if (PreDischargeEnergy != DischargeEnergy && PreDischargeEnergy != 0) {

              if (abs(DischargeEnergy - PreDischargeEnergy) * 100 < 1000) {
                HourlyDischargeEnergy = ((DischargeEnergy - PreDischargeEnergy) * 100 + HourlyDischargeEnergy);
              }
              else {
                //Serial.println("invalid!!!!!!!!!!!");
              }
            }


            //            HourlyDischargeEnergy = 1+ HourlyDischargeEnergy;


            //            Serial.print("Hourly Discharge Energy:");
            //            Serial.println(HourlyDischargeEnergy);
            //
            //            Serial.println("end");
            //            Serial.println("");

          }
          break;


        case 9:
          //Serial.println("ID 95 is requested...");
          BMS_recieve(0x50);
          delay(10);






          break;



        case 10:
          //Serial.println("ID 95 is requested...");
          BMS_recieve(0x51);
          delay(10);

          //          Serial.println("CellNumber1");
          //          Serial.println(String(BMS.board1cellnum));
          //
          //
          //          Serial.println("CellNumber2");
          //          Serial.println(String(BMS.board2cellnum));
          if (!SimMode) {
            ReadCellMod = BMS.board1cellnum + BMS.board2cellnum;
          }
          //          Serial.println("CellMod:" + String(ReadCellMod));
          //  NominalVoltage = BMS.rated_volt * 0.001 * ReadCellMod;

          //          Serial.println("rated cell voltage:" + String(BMS.rated_volt * 0.001));
          //          Serial.println("nominal voltage:" + String(NominalVoltage));
          //          Serial.println("rated capacity:" + String(NominalVoltage * BMS.rated_cap * 0.001 * 0.001));
          break;


        case 11:
          //Serial.println("ID 95 is requested...");


          CellTemp1 = getTemp_1(35);
          CellTemp2 = getTemp_2(34);
          delay(10);
          break;


        case 12:
          //Serial.println("BMU is requested...");

          for (int i = 0; i < 1; i++) {
            delay(1);
            BMS_recieve(0x57);
            delay(10);
            if (bmsserialnumber != "") {
              //Serial.println("BMS Serial no:" + bmsserialnumber);

              break;
            }
            else {
              //Serial.println("Searching for BMS Serial no: Trial:" + String(i));
            }
          }
          break;

        case 13:
          if (AlarmReadTrigger) {

            for (int p = 0; p < 10; p++) {



              BMSAlarmArray[p][0] = p;

              delay(100);
              BMS_recieve(0x64);

              //  Serial.println("row:" + String(i) + " Started");
              for (int i = 0; i < 22; i++) {
                Serial.print(recoder_fault[i]);
                Serial.print("-");
                BMSAlarmArray[p][i + 1] = recoder_fault[i];
              }
              Serial.println(" !#");
              // Serial.println("row:" + String(i) + " Ended");

            }
            closet = 1;
            send_read_BMS(0x64);
            delay(100);
            recieve_back(0x64);
          }
          AlarmReadTrigger = false;

          break;


        default:
          bmsCounter = 0;
          break;




      }

      if (buzzerActive) {

        if (!mute) {
          buzzer_on();
          delay(50);
          buzzer_off();

        }
        buzzerActive = false;



      }

      delay(10);
      FooterEvent = 20;
    }

    else {

      Serial.println("Startup commands are giving....");
      //rated capacity
      BMS_recieve(0x50);
      delay(50);
      //charge discharge high low
      BMS_recieve(0x5B);
      delay(50);
      //sum highlow
      BMS_recieve(0x5A);
      delay(50);
      //CHARGE DISCHARGE TEMP
      BMS_recieve(0x5C);
      delay(50);
      //SOC
      BMS_recieve(0x5D);
      delay(50);
      //CUMULATIVE
      BMS_recieve(0x52);
      delay(50);
      //vOLT DIFF
      BMS_recieve(0x5E);
      delay(50);
      //    BALANCE START
      BMS_recieve(0x5F);
      delay(50);
      //    cell volt high low
      BMS_recieve(0x59);
      delay(50);
      //      BMS_recieve_equ(0x5F);
      //      BMS_recieve_equ(0x5A);
      //      BMS_recieve_equ(0x59);
      //      BMS_recieve_equ(0x53);
      delay(50);

      SafetyStartUp2 = false;
      Serial.println("Startup finished successfully");
    }



    //    if (!mute) {
    //      delay(200);
    //      buzzer_on();
    //      delay(200);
    //      buzzer_off();
    //    }
  }

  else {
    loading = true;
    //rated capacity
    BMS_recieve(0x50);
    delay(50);
    //Serial.print("rated capacity:");
    // Serial.println(BMS.rated_cap);
    loadingPercentage = 10;
    //charge discharge high low
    BMS_recieve(0x5B);
    delay(50);
    loadingPercentage = 20;
    //sum highlow


    BMS_recieve(0x5A);
    delay(50);
    loadingPercentage = 30;
    //CHARGE DISCHARGE TEMP
    BMS_recieve(0x5C);
    delay(50);
    loadingPercentage = 40;
    //SOC
    BMS_recieve(0x5D);
    delay(50);
    loadingPercentage = 60;
    //CUMULATIVE
    BMS_recieve(0x52);
    delay(50);
    loadingPercentage = 70;
    //vOLT DIFF
    BMS_recieve(0x5E);
    delay(50);
    loadingPercentage = 80;
    //    BALANCE START
    BMS_recieve(0x5F);
    delay(50);
    loadingPercentage = 90;
    //    cell volt high low
    BMS_recieve(0x59);
    delay(50);
    loadingPercentage = 95;





    // BMS_recieve_equ(0x5F);

    loadingPercentage = 97;

    // BMS_recieve_equ(0x5A);

    loadingPercentage = 98;



    //BMS_recieve_equ(0x59);

    loadingPercentage = 99;



    // BMS_recieve_equ(0x53);
    delay(50);

    loadingPercentage = 100;
    delay(1000);
    loading = false;
    ReadAllParameters = false;
    Serial.println("All parameters are read");
  }
}



//DISPLAY FUNCTIONS///////////////////////////////////////



void drawBanner(String Time, String Date, String Page, bool wifistatus, int notification_no, bool UpdateAvailable) {





  BannerFrame.createSprite(409, 60);
  BannerFrame.fillSprite(TFT_BACKGROUND2);
  BannerFrame.setSwapBytes(true);
  BannerFrame.pushImage(0, 0, 409, 60, bannermiddle);
  BannerFrame.setTextColor(TFT_WHITE);
  BannerFrame.setTextDatum(TL_DATUM);





  if (CompanyMod == 2) {
    BannerIconMiddle.createSprite(245, 50);
    //Icon1.setSwapBytes(true);
    BannerIconMiddle.pushImage(0, 0, 245, 50, encaplogo);
    BannerIconMiddle.pushToSprite(&BannerFrame, 70, 2, TFT_BLACK);
  }


  else if (CompanyMod == 3) {
    BannerIconMiddle.createSprite(245, 50);
    // Icon1.setSwapBytes(true);
    BannerIconMiddle.pushImage(0, 0, 245, 50, ensiriuslogo);
    BannerIconMiddle.pushToSprite(&BannerFrame, 70, 2, TFT_BLACK);
  }


  else if (CompanyMod == 4) {
    BannerIconMiddle.createSprite(245, 50);
    // Icon1.setSwapBytes(true);
    BannerIconMiddle.pushImage(0, 0, 245, 50, ensegalogo);
    BannerIconMiddle.pushToSprite(&BannerFrame, 70, 2, TFT_BLACK);
  }


  else if (CompanyMod == 5) {
    BannerIconMiddle.createSprite(225, 48);
    //  Icon1.setSwapBytes(true);
    BannerIconMiddle.pushImage(0, 0, 225, 48, banner);
    BannerIconMiddle.pushToSprite(&BannerFrame, 70, 2, TFT_BLACK);
  }


  else if (CompanyMod == 6) {
    BannerIconMiddle.createSprite(245, 50);
    //  Icon1.setSwapBytes(true);
    BannerIconMiddle.pushImage(0, 0, 245, 50, enlight);
    BannerIconMiddle.pushToSprite(&BannerFrame, 70, 2, TFT_BLACK);
  }


  else if (CompanyMod == 8) {
    BannerIconMiddle.createSprite(245, 50);
    //  Icon1.setSwapBytes(true);
    BannerIconMiddle.pushImage(0, 0, 245, 50, ENWALL);
    BannerIconMiddle.pushToSprite(&BannerFrame, 70, 2, TFT_BLACK);
  }



  else {
    BannerIconMiddle.createSprite(245, 50);
    //Icon1.setSwapBytes(true);
    BannerIconMiddle.pushImage(0, 0, 245, 50, encaplogo);
    BannerIconMiddle.pushToSprite(&BannerFrame, 70, 2, TFT_BLACK);
  }
  BannerIconMiddle.deleteSprite();
  BannerFrame.pushSprite(36, 0);
  BannerFrame.deleteSprite();



  BannerFrame.createSprite(34, 60);
  BannerFrame.fillSprite(TFT_BACKGROUND2);
  BannerFrame.setSwapBytes(true);
  BannerFrame.pushImage(0, 0, 34, 60, bannerleft);

  BannerIconLeft.createSprite(24, 24);
  //Icon1.setSwapBytes(true);
  BannerIconLeft.pushImage(0, 0, 24, 24, menu1);
  BannerIconLeft.pushToSprite(&BannerFrame, 5, 20, TFT_BLACK);
  BannerFrame.pushSprite(0, 0, TFT_BLACK);
  BannerFrame.deleteSprite();
  BannerIconLeft.deleteSprite();



  BannerFrame.createSprite(34, 60);
  BannerFrame.fillSprite(TFT_BACKGROUND2);
  BannerFrame.setSwapBytes(true);
  BannerFrame.pushImage(0, 0, 34, 60, bannerright);

  BannerIconRight.createSprite(24, 24);
  //Icon1.setSwapBytes(true);
  BannerIconRight.pushImage(0, 0, 24, 24, back);
  BannerIconRight.pushToSprite(&BannerFrame, 5, 20, TFT_BLACK);
  BannerIconRight.deleteSprite();


  BannerFrame.pushSprite(447 , 0, TFT_BLACK);
  BannerFrame.deleteSprite();





}

void drawFooter() {


  Display.createSprite(480, 50);
  Display.fillSprite(TFT_BACKGROUND2);



  /////////////////////

  FooterFrame.createSprite(480, 50);
  FooterFrame.fillSprite(TFT_BACKGROUND2);
  FooterFrame.setSwapBytes(true);
  FooterFrame.pushImage(0, 0, 480, 50, footernew);
  FooterFrame.setTextColor(TFT_WHITE);



  ////////////////////


  FooterIconLeft.createSprite(24, 24);
  //FooterIconLeft.setSwapBytes(true);


  //  if (SimulationStatus) {
  //    AnimationBlink = !AnimationBlink;
  //    if (AnimationBlink) {
  //      FooterIconLeft.pushImage(0, 0, 24, 24, simulation);
  //
  //    }
  //  }

  FooterIconLeft.pushToSprite(&FooterFrame, 5, 10, TFT_BLACK);
  FooterIconLeft.deleteSprite();



  FooterFrame.fillRoundRect(295, 7, 180, 30, 10, TFT_BACKGROUND3); // functions backdrop
  FooterFrame.fillRoundRect(10, 7, 180, 30, 10, TFT_BACKGROUND3);// alarm backdrop

  FooterFrame.fillRoundRect(200, 7, 85, 30, 10, TFT_BACKGROUND3); //time string backdrop







  bool AlarmReady = false;

  //  FooterFrame.setFreeFont(&Orbitron_Medium_12);
  //  FooterFrame.setTextDatum(TL_DATUM);
  //  FooterFrame.drawString(TimeString , 215, 15);


  ///////////
  FooterIconLeft.createSprite(16, 16);
  FooterIconLeft.pushImage(0, 0, 16, 16, alarm_dash);
  FooterIconLeft.pushToSprite(&FooterFrame, 20, 15, TFT_BLACK);
  FooterFrame.setFreeFont(&Orbitron_Medium_12);
  FooterFrame.setTextDatum(TL_DATUM);


  for (int i = 0; i < 28; i++) {
    if (BMS.error[i] && (i != 0) && (i != 2) && (i != 4) && (i != 6)) {
      FooterFrame.drawString(AlarmStringLogPro , 50, 15);
      FooterFrame.fillCircle(35, 30, 3, TOLGA_RED);
      AlarmReady = true;
      break;
    }
  }

  if (!AlarmReady) {
    FooterFrame.drawString("No Alarm" , 50, 15);
    FooterFrame.fillCircle(35, 30, 3, TOLGA_GREEN);
  }


  FooterIconLeft.deleteSprite();

  //////////////////////////

  ///////////

  ///////////////////////////


  FooterIconLeft.createSprite(24, 24);
  FooterIconLeft.pushImage(0, 0, 24, 24, safe_dash);
  FooterIconLeft.pushToSprite(&FooterFrame, 230, 8, TFT_BLACK);
  FooterIconLeft.deleteSprite();


  if (Safety) {
    FooterFrame.fillCircle(250, 30, 3, TOLGA_GREEN);
  }
  else {
    FooterFrame.fillCircle(250, 30, 3, TOLGA_RED);
  }



  if (GPIO_READ[0]) {
    FooterIconLeft.createSprite(16, 16);
    FooterIconLeft.pushImage(0, 0, 16, 16, hazard);
    FooterIconLeft.pushToSprite(&FooterFrame, 260, 15, TFT_BLACK);
    FooterIconLeft.deleteSprite();
  }





  ///ttttttt
  if (!FooterTouch) {

    FooterIconLeft.createSprite(16, 16);
    FooterIconLeft.pushImage(0, 0, 16, 16, sd_dash);
    FooterIconLeft.pushToSprite(&FooterFrame, 440, 15, TFT_BLACK);
    FooterIconLeft.deleteSprite();


    if (control_sd) {
      FooterFrame.fillCircle(460, 30, 3, TOLGA_GREEN);
    }
    else {
      FooterFrame.fillCircle(460, 30, 3, TOLGA_RED);
    }



    ///////////
    FooterIconLeft.createSprite(16, 16);
    FooterIconLeft.pushImage(0, 0, 16, 16, wifi_dash);
    FooterIconLeft.pushToSprite(&FooterFrame, 410, 15, TFT_BLACK);
    FooterIconLeft.deleteSprite();


    if (wifistatus) {
      FooterFrame.fillCircle(430, 30, 3, TOLGA_GREEN);
    }
    else {
      FooterFrame.fillCircle(430, 30, 3, TOLGA_RED);
    }

    ///////////////////////////



    ///////////
    FooterIconLeft.createSprite(16, 16);
    FooterIconLeft.pushImage(0, 0, 16, 16, blue_dash);
    FooterIconLeft.pushToSprite(&FooterFrame, 380, 15, TFT_BLACK);
    FooterIconLeft.deleteSprite();

    if (bluetoothReady) {
      FooterFrame.fillCircle(400, 30, 3, TOLGA_GREEN);
    }
    else {
      FooterFrame.fillCircle(400, 30, 3, TOLGA_RED);
    }

    ///////////////////////////

    ///////////



    FooterIconLeft.createSprite(28, 12);
    FooterIconLeft.pushImage(0, 0, 28, 12, can_dash);
    FooterIconLeft.pushToSprite(&FooterFrame, 340, 15, TFT_BLACK);
    FooterIconLeft.deleteSprite();




    if (canbusReady) {
      FooterFrame.fillCircle(375, 30, 3, TOLGA_GREEN);
    }
    else {
      FooterFrame.fillCircle(375, 30, 3, TOLGA_RED);
    }


    ///////////////////////////

    ///////////
    FooterIconLeft.createSprite(16, 16);

    if (FBNextRestart) {
      FooterIconLeft.pushImage(0, 0, 16, 16, firebaseReadyIcon);
    }

    if (WDNextRestart || BTNextRestart) {
      FooterIconLeft.pushImage(0, 0, 16, 16, udp_dash);
    }

    FooterIconLeft.pushToSprite(&FooterFrame, 310, 15, TFT_BLACK);
    FooterIconLeft.deleteSprite();


    if (WDNextRestart) {
      if (wifistatus) {
        FooterFrame.fillCircle(330, 30, 3, TOLGA_GREEN);
      }
      else {
        FooterFrame.fillCircle(330, 30, 3, TOLGA_RED);
      }
    }
    else {
      if (firebaseReady) {
        FooterFrame.fillCircle(330, 30, 3, TOLGA_GREEN);
      }
      else {
        FooterFrame.fillCircle(330, 30, 3, TOLGA_RED);
      }
    }



    ///////////////////////////
  }
  else {

    ///bbbbbbbb
    ///////////

    FooterIconLeft.createSprite(16, 16);
    FooterIconLeft.pushImage(0, 0, 16, 16, calender);
    FooterIconLeft.pushToSprite(&FooterFrame, 310, 15, TFT_BLACK);
    FooterIconLeft.deleteSprite();




    if (String(BMS.year).toInt() >= 2001) {

      FooterFrame.fillCircle(330, 30, 3, TOLGA_GREEN);
      FooterFrame.setFreeFont(&Orbitron_Medium_12);
      FooterFrame.drawString( TimeString + " " + DateString , 335, 15);
    }
    else {
      FooterFrame.fillCircle(330, 30, 3, TOLGA_RED);
      FooterFrame.setFreeFont(&Orbitron_Medium_12);
      FooterFrame.drawString( "--" , 370, 15);


    }

    ///////////////////////////

  }



  if (ReadAllParameters) {

    Serial.println("read all parameters");
    FooterFrame.setTextColor(TOLGA_WHITE);
    FooterFrame.setTextDatum(MC_DATUM);
    FooterFrame.setTextColor(TOLGA_YELLOW);
    FooterFrame.setFreeFont(&Orbitron_Medium_12);
    //FooterFrame.drawString("Fetching Data From System %" + String(loadingPercentage) + "..." , 240, 20);
    //PageFrame.drawString("Fetching Data From System %" + String(loadingPercentage) + "..." , 240, 20);

  }


  FooterFrame.pushToSprite(&Display, 0, 0, TFT_BLACK);
  FooterFrame.deleteSprite();

  Display.pushSprite(0, 270);
  Display.deleteSprite();
}


void drawDryContact() {
  Display.createSprite(480, 210);
  Display.fillSprite(TFT_BLACK);
  PageFrame.createSprite(80, 40);
  PageFrame.fillSprite(TFT_BACKGROUND2);
  PageFrame.setTextColor(TOLGA_WHITE);
  PageFrame.setTextDatum(MC_DATUM);
  PageFrame.setTextColor(TFT_SILVER);
  PageFrame.setFreeFont(&Orbitron_Medium_12);
  PageFrame.drawString(Languages[indexLanguageArray][50] , 40, 20);
  PageFrame.setTextColor(TOLGA_WHITE);
  PageFrame.pushToSprite(&Display, 0, 2, TFT_BLACK);
  PageFrame.deleteSprite();

  PageFrame.createSprite(80, 38);
  PageFrame.fillSprite(TFT_BACKGROUND2);
  PageFrame.setTextColor(TOLGA_WHITE);
  PageFrame.setTextDatum(MC_DATUM);
  PageFrame.setTextColor(TOLGA_YELLOW);
  PageFrame.setFreeFont(&Orbitron_Medium_12);
  PageFrame.drawString("A" , 40, 20);
  PageFrame.setTextColor(TOLGA_WHITE);
  PageFrame.pushToSprite(&Display, 0, 44, TFT_BLACK);
  PageFrame.deleteSprite();

  PageFrame.createSprite(80, 40);
  PageFrame.fillSprite(TFT_BACKGROUND2);
  PageFrame.setTextColor(TOLGA_WHITE);
  PageFrame.setTextDatum(MC_DATUM);
  PageFrame.setTextColor(TOLGA_YELLOW);
  PageFrame.setFreeFont(&Orbitron_Medium_12);
  PageFrame.drawString("B" , 40, 20);
  PageFrame.setTextColor(TOLGA_WHITE);
  PageFrame.pushToSprite(&Display, 0, 84, TFT_BLACK);
  PageFrame.deleteSprite();

  PageFrame.createSprite(80, 40);
  PageFrame.fillSprite(TFT_BACKGROUND2);
  PageFrame.setTextColor(TOLGA_WHITE);
  PageFrame.setTextDatum(MC_DATUM);
  PageFrame.setTextColor(TOLGA_YELLOW);
  PageFrame.setFreeFont(&Orbitron_Medium_12);
  PageFrame.drawString("C" , 40, 20);
  PageFrame.setTextColor(TOLGA_WHITE);
  PageFrame.pushToSprite(&Display, 0, 126, TFT_BLACK);
  PageFrame.deleteSprite();

  PageFrame.createSprite(80, 40);
  PageFrame.fillSprite(TFT_BACKGROUND2);
  PageFrame.setTextColor(TOLGA_WHITE);
  PageFrame.setTextDatum(MC_DATUM);
  PageFrame.setTextColor(TOLGA_YELLOW);
  PageFrame.setFreeFont(&Orbitron_Medium_12);
  PageFrame.drawString("D" , 40, 20);
  PageFrame.setTextColor(TOLGA_WHITE);
  PageFrame.pushToSprite(&Display, 0, 168, TFT_BLACK);
  PageFrame.deleteSprite();





  //////////////////////////////////////////////////////


  PageFrame.createSprite(110, 40);
  PageFrame.fillSprite(TFT_BACKGROUND2);
  PageFrame.setTextColor(TOLGA_WHITE);
  PageFrame.setTextDatum(MC_DATUM);
  PageFrame.setTextColor(TFT_SILVER);
  PageFrame.setFreeFont(&Orbitron_Medium_12);
  PageFrame.drawString(Languages[indexLanguageArray][51] , 55, 20);
  PageFrame.setTextColor(TOLGA_WHITE);
  PageFrame.pushToSprite(&Display, 82, 2, TFT_BLACK);
  PageFrame.deleteSprite();


  PageFrame.createSprite(110, 38);
  PageFrame.fillSprite(TFT_BACKGROUND2);
  PageFrame.setTextDatum(MC_DATUM);
  PageFrame.setTextColor(TOLGA_WHITE);
  PageFrame.setFreeFont(&Orbitron_Medium_12);
  PageFrame.drawString(ContactFunctionList[DRYA_ARRAY[0].toInt()] , 55, 20);
  PageFrame.pushToSprite(&Display, 82, 44, TFT_BLACK);
  PageFrame.deleteSprite();


  PageFrame.createSprite(110, 40);
  PageFrame.fillSprite(TFT_BACKGROUND2);
  PageFrame.setTextDatum(MC_DATUM);
  PageFrame.setTextColor(TOLGA_WHITE);
  PageFrame.setFreeFont(&Orbitron_Medium_12);
  PageFrame.drawString(ContactFunctionList[DRYB_ARRAY[0].toInt()] , 55, 20);
  PageFrame.pushToSprite(&Display, 82, 84, TFT_BLACK);
  PageFrame.deleteSprite();


  PageFrame.createSprite(110, 40);
  PageFrame.fillSprite(TFT_BACKGROUND2);
  PageFrame.setTextDatum(MC_DATUM);
  PageFrame.setTextColor(TOLGA_WHITE);
  PageFrame.setFreeFont(&Orbitron_Medium_12);
  PageFrame.drawString(ContactFunctionList[DRYC_ARRAY[0].toInt()] , 55, 20);
  PageFrame.pushToSprite(&Display, 82, 126, TFT_BLACK);
  PageFrame.deleteSprite();

  PageFrame.createSprite(110, 40);
  PageFrame.fillSprite(TFT_BACKGROUND2);
  PageFrame.setTextDatum(MC_DATUM);
  PageFrame.setTextColor(TOLGA_WHITE);
  PageFrame.setFreeFont(&Orbitron_Medium_12);
  PageFrame.drawString(ContactFunctionList[DRYD_ARRAY[0].toInt()] , 55, 20);
  PageFrame.pushToSprite(&Display, 82, 168, TFT_BLACK);
  PageFrame.deleteSprite();





  //////////////////////////////////////////////////////
  PageFrame.createSprite(80, 40);
  PageFrame.fillSprite(TFT_BACKGROUND2);
  PageFrame.setTextColor(TOLGA_WHITE);
  PageFrame.setTextDatum(MC_DATUM);
  PageFrame.setTextColor(TFT_SILVER);
  PageFrame.setFreeFont(&Orbitron_Medium_12);
  PageFrame.drawString(Languages[indexLanguageArray][52] , 40, 20);
  PageFrame.setTextColor(TOLGA_WHITE);
  PageFrame.pushToSprite(&Display, 194, 2, TFT_BLACK);
  PageFrame.deleteSprite();



  PageFrame.createSprite(80, 38);
  PageFrame.fillSprite(TFT_BACKGROUND2);
  PageFrame.setTextDatum(MC_DATUM);
  PageFrame.setTextColor(TOLGA_YELLOW);
  PageFrame.setFreeFont(&Orbitron_Medium_12);
  PageFrame.drawString(OperationList[DRYA_ARRAY[1].toInt()] , 40, 20);
  PageFrame.pushToSprite(&Display, 194, 44, TFT_BLACK);
  PageFrame.deleteSprite();

  PageFrame.createSprite(80, 40);
  PageFrame.fillSprite(TFT_BACKGROUND2);
  PageFrame.setTextDatum(MC_DATUM);
  PageFrame.setTextColor(TOLGA_YELLOW);
  PageFrame.setFreeFont(&Orbitron_Medium_12);
  PageFrame.drawString(OperationList[DRYB_ARRAY[1].toInt()] , 40, 20);
  PageFrame.pushToSprite(&Display, 194, 84, TFT_BLACK);
  PageFrame.deleteSprite();

  PageFrame.createSprite(80, 40);
  PageFrame.fillSprite(TFT_BACKGROUND2);
  PageFrame.setTextDatum(MC_DATUM);
  PageFrame.setTextColor(TOLGA_YELLOW);
  PageFrame.setFreeFont(&Orbitron_Medium_12);
  PageFrame.drawString(OperationList[DRYC_ARRAY[1].toInt()] , 40, 20);
  PageFrame.pushToSprite(&Display, 194, 126, TFT_BLACK);
  PageFrame.deleteSprite();


  PageFrame.createSprite(80, 40);
  PageFrame.fillSprite(TFT_BACKGROUND2);
  PageFrame.setTextDatum(MC_DATUM);
  PageFrame.setTextColor(TOLGA_YELLOW);
  PageFrame.setFreeFont(&Orbitron_Medium_12);
  PageFrame.drawString(OperationList[DRYD_ARRAY[1].toInt()] , 40, 20);
  PageFrame.pushToSprite(&Display, 194, 168, TFT_BLACK);
  PageFrame.deleteSprite();


  //////////////////////////////////////////////////////

  PageFrame.createSprite(70, 40);
  PageFrame.fillSprite(TFT_BACKGROUND2);
  PageFrame.setTextColor(TOLGA_WHITE);
  PageFrame.setTextDatum(MC_DATUM);
  PageFrame.setTextColor(TFT_SILVER);
  PageFrame.setFreeFont(&Orbitron_Medium_12);
  PageFrame.drawString(Languages[indexLanguageArray][53] , 35, 20);
  PageFrame.setTextColor(TOLGA_WHITE);
  PageFrame.pushToSprite(&Display, 276, 2, TFT_BLACK);
  PageFrame.deleteSprite();


  PageFrame.createSprite(70, 38);
  PageFrame.fillSprite(TFT_BACKGROUND2);
  PageFrame.setTextColor(TOLGA_WHITE);
  PageFrame.setTextDatum(MC_DATUM);
  PageFrame.setTextColor(TOLGA_WHITE);
  PageFrame.setFreeFont(&Orbitron_Medium_12);
  PageFrame.drawString(DRYA_ARRAY[2] , 35, 20);
  PageFrame.setTextColor(TOLGA_WHITE);
  PageFrame.pushToSprite(&Display, 276, 44, TFT_BLACK);
  PageFrame.deleteSprite();


  PageFrame.createSprite(70, 40);
  PageFrame.fillSprite(TFT_BACKGROUND2);
  PageFrame.setTextColor(TOLGA_WHITE);
  PageFrame.setTextDatum(MC_DATUM);
  PageFrame.setTextColor(TOLGA_WHITE);
  PageFrame.setFreeFont(&Orbitron_Medium_12);
  PageFrame.drawString(DRYB_ARRAY[2] , 35, 20);
  PageFrame.setTextColor(TOLGA_WHITE);
  PageFrame.pushToSprite(&Display, 276, 84, TFT_BLACK);
  PageFrame.deleteSprite();


  PageFrame.createSprite(70, 40);
  PageFrame.fillSprite(TFT_BACKGROUND2);
  PageFrame.setTextColor(TOLGA_WHITE);
  PageFrame.setTextDatum(MC_DATUM);
  PageFrame.setTextColor(TOLGA_WHITE);
  PageFrame.setFreeFont(&Orbitron_Medium_12);
  PageFrame.drawString(DRYC_ARRAY[2] , 35, 20);
  PageFrame.setTextColor(TOLGA_WHITE);
  PageFrame.pushToSprite(&Display, 276, 126, TFT_BLACK);
  PageFrame.deleteSprite();

  PageFrame.createSprite(70, 40);
  PageFrame.fillSprite(TFT_BACKGROUND2);
  PageFrame.setTextColor(TOLGA_WHITE);
  PageFrame.setTextDatum(MC_DATUM);
  PageFrame.setTextColor(TOLGA_WHITE);
  PageFrame.setFreeFont(&Orbitron_Medium_12);
  PageFrame.drawString(DRYD_ARRAY[2] , 35, 20);
  PageFrame.setTextColor(TOLGA_WHITE);
  PageFrame.pushToSprite(&Display, 276, 168, TFT_BLACK);
  PageFrame.deleteSprite();


  //////////////////////////////////////////////////////


  PageFrame.createSprite(70, 40);
  PageFrame.fillSprite(TFT_BACKGROUND2);
  PageFrame.setTextColor(TOLGA_WHITE);
  PageFrame.setTextDatum(MC_DATUM);
  PageFrame.setTextColor(TFT_SILVER);
  PageFrame.setFreeFont(&Orbitron_Medium_12);
  PageFrame.drawString(Languages[indexLanguageArray][54] , 35, 20);
  PageFrame.setTextColor(TOLGA_WHITE);
  PageFrame.pushToSprite(&Display, 348, 2, TFT_BLACK);
  PageFrame.deleteSprite();


  PageFrame.createSprite(70, 38);
  PageFrame.fillSprite(TFT_BACKGROUND2);
  PageFrame.setTextColor(TOLGA_WHITE);
  PageFrame.setTextDatum(MC_DATUM);
  PageFrame.setTextColor(TOLGA_YELLOW);
  PageFrame.setFreeFont(&Orbitron_Medium_12);
  PageFrame.drawString(DRYA_ARRAY[3] , 35, 20);
  PageFrame.setTextColor(TOLGA_WHITE);
  PageFrame.pushToSprite(&Display, 348, 44, TFT_BLACK);
  PageFrame.deleteSprite();


  PageFrame.createSprite(70, 40);
  PageFrame.fillSprite(TFT_BACKGROUND2);
  PageFrame.setTextColor(TOLGA_WHITE);
  PageFrame.setTextDatum(MC_DATUM);
  PageFrame.setTextColor(TOLGA_YELLOW);
  PageFrame.setFreeFont(&Orbitron_Medium_12);
  PageFrame.drawString(DRYB_ARRAY[3] , 35, 20);
  PageFrame.setTextColor(TOLGA_WHITE);
  PageFrame.pushToSprite(&Display, 348, 84, TFT_BLACK);
  PageFrame.deleteSprite();

  PageFrame.createSprite(70, 40);
  PageFrame.fillSprite(TFT_BACKGROUND2);
  PageFrame.setTextColor(TOLGA_WHITE);
  PageFrame.setTextDatum(MC_DATUM);
  PageFrame.setTextColor(TOLGA_YELLOW);
  PageFrame.setFreeFont(&Orbitron_Medium_12);
  PageFrame.drawString(DRYC_ARRAY[3] , 35, 20);
  PageFrame.setTextColor(TOLGA_WHITE);
  PageFrame.pushToSprite(&Display, 348, 126, TFT_BLACK);
  PageFrame.deleteSprite();

  PageFrame.createSprite(70, 40);
  PageFrame.fillSprite(TFT_BACKGROUND2);
  PageFrame.setTextColor(TOLGA_WHITE);
  PageFrame.setTextDatum(MC_DATUM);
  PageFrame.setTextColor(TOLGA_YELLOW);
  PageFrame.setFreeFont(&Orbitron_Medium_12);
  PageFrame.drawString(DRYD_ARRAY[3] , 35, 20);
  PageFrame.setTextColor(TOLGA_WHITE);
  PageFrame.pushToSprite(&Display, 348, 168, TFT_BLACK);
  PageFrame.deleteSprite();


  //////////////////////////////////////////////////////



  PageFrame.createSprite(60, 40);
  PageFrame.fillSprite(TFT_BACKGROUND2);
  PageFrame.setTextColor(TOLGA_WHITE);
  PageFrame.setTextDatum(MC_DATUM);
  PageFrame.setTextColor(TFT_SILVER);
  PageFrame.setFreeFont(&Orbitron_Medium_12);
  PageFrame.drawString(Languages[indexLanguageArray][55] , 30, 20);
  PageFrame.setTextColor(TOLGA_WHITE);
  PageFrame.pushToSprite(&Display, 420, 2, TFT_BLACK);
  PageFrame.deleteSprite();




  PageFrame.createSprite(60, 38);
  PageFrame.fillSprite(TFT_BACKGROUND2);
  PageFrame.setTextColor(TOLGA_WHITE);
  PageFrame.setTextDatum(MC_DATUM);
  PageFrame.setFreeFont(&Orbitron_Medium_12);

  if (DRY_VALUES[0] == 0) {
    PageFrame.setTextColor(TOLGA_RED);
    PageFrame.drawString("OFF" , 30, 20);
  }
  else {
    PageFrame.setTextColor(TOLGA_GREEN);
    PageFrame.drawString("ON" , 30, 20);
  }

  PageFrame.pushToSprite(&Display, 420, 44, TFT_BLACK);
  PageFrame.deleteSprite();


  PageFrame.createSprite(60, 40);
  PageFrame.fillSprite(TFT_BACKGROUND2);
  PageFrame.setTextColor(TOLGA_WHITE);
  PageFrame.setTextDatum(MC_DATUM);
  PageFrame.setFreeFont(&Orbitron_Medium_12);

  if (DRY_VALUES[1] == 0) {
    PageFrame.setTextColor(TOLGA_RED);
    PageFrame.drawString("OFF" , 30, 20);
  }
  else {
    PageFrame.setTextColor(TOLGA_GREEN);
    PageFrame.drawString("ON" , 30, 20);
  }

  PageFrame.pushToSprite(&Display, 420, 84, TFT_BLACK);
  PageFrame.deleteSprite();


  PageFrame.createSprite(60, 40);
  PageFrame.fillSprite(TFT_BACKGROUND2);
  PageFrame.setTextColor(TOLGA_WHITE);
  PageFrame.setTextDatum(MC_DATUM);
  PageFrame.setFreeFont(&Orbitron_Medium_12);

  if (DRY_VALUES[2] == 0) {
    PageFrame.setTextColor(TOLGA_RED);
    PageFrame.drawString("OFF" , 30, 20);
  }
  else {
    PageFrame.setTextColor(TOLGA_GREEN);
    PageFrame.drawString("ON" , 30, 20);
  }

  PageFrame.pushToSprite(&Display, 420, 126, TFT_BLACK);
  PageFrame.deleteSprite();

  PageFrame.createSprite(60, 40);
  PageFrame.fillSprite(TFT_BACKGROUND2);
  PageFrame.setTextColor(TOLGA_WHITE);
  PageFrame.setTextDatum(MC_DATUM);
  PageFrame.setFreeFont(&Orbitron_Medium_12);

  if (DRY_VALUES[3] == 0) {
    PageFrame.setTextColor(TOLGA_RED);
    PageFrame.drawString("OFF" , 30, 20);
  }
  else {
    PageFrame.setTextColor(TOLGA_GREEN);
    PageFrame.drawString("ON" , 30, 20);
  }

  PageFrame.pushToSprite(&Display, 420, 168, TFT_BLACK);
  PageFrame.deleteSprite();






  //////////////////////////////////////////////////////



  Display.pushSprite(0, 60);
  Display.deleteSprite();


}

////////////////////////////////////////////////////

void drawNumpadDry() {

  Display.createSprite(480, 210);
  Display.fillSprite(TFT_BLACK);


  PageFrame.createSprite(375, 62);
  PageFrame.fillSprite(TOLGA_WHITE);
  PageFrame.setTextColor(TOLGA_WHITE);
  PageFrame.setTextDatum(TL_DATUM);
  PageFrame.setTextColor(TFT_BACKGROUND2);
  PageFrame.setFreeFont(&Orbitron_Bold_28);
  PageFrame.drawString(numpadValue, 30, 10, 7);
  PageFrame.pushToSprite(&Display, 10, 5, TFT_BLACK);
  PageFrame.deleteSprite();
  //Orbitron_Medium_24
  PageFrame.createSprite(75, 62);
  PageFrame.fillSprite(TFT_BACKGROUND2);
  PageFrame.setTextColor(TOLGA_WHITE);
  PageFrame.setTextDatum(MC_DATUM);
  PageFrame.setTextColor(TOLGA_YELLOW);
  PageFrame.setFreeFont(&Orbitron_Bold_28);
  PageFrame.drawString("OK" , 37, 30);
  PageFrame.pushToSprite(&Display, 390, 5, TFT_BLACK);
  PageFrame.deleteSprite();

  //



  PageFrame.createSprite(55, 62);
  PageFrame.fillSprite(TFT_BACKGROUND2);
  PageFrame.setTextColor(TOLGA_WHITE);
  PageFrame.setTextDatum(MC_DATUM);
  PageFrame.setTextColor(TOLGA_YELLOW);
  PageFrame.setFreeFont(&Orbitron_Bold_28);
  PageFrame.drawString("1" , 30, 30);
  PageFrame.pushToSprite(&Display, 10, 72, TFT_BLACK);
  PageFrame.deleteSprite();

  PageFrame.createSprite(55, 62);
  PageFrame.fillSprite(TFT_BACKGROUND2);
  PageFrame.setTextColor(TOLGA_WHITE);
  PageFrame.setTextDatum(MC_DATUM);
  PageFrame.setTextColor(TOLGA_YELLOW);
  PageFrame.setFreeFont(&Orbitron_Bold_28);
  PageFrame.drawString("2" , 30, 30);
  PageFrame.pushToSprite(&Display, 70, 72, TFT_BLACK);
  PageFrame.deleteSprite();


  PageFrame.createSprite(55, 62);
  PageFrame.fillSprite(TFT_BACKGROUND2);
  PageFrame.setTextColor(TOLGA_WHITE);
  PageFrame.setTextDatum(MC_DATUM);
  PageFrame.setTextColor(TOLGA_YELLOW);
  PageFrame.setFreeFont(&Orbitron_Bold_28);
  PageFrame.drawString("3" , 30, 30);
  PageFrame.pushToSprite(&Display, 130, 72, TFT_BLACK);
  PageFrame.deleteSprite();

  PageFrame.createSprite(55, 62);
  PageFrame.fillSprite(TFT_BACKGROUND2);
  PageFrame.setTextColor(TOLGA_WHITE);
  PageFrame.setTextDatum(MC_DATUM);
  PageFrame.setTextColor(TOLGA_YELLOW);
  PageFrame.setFreeFont(&Orbitron_Bold_28);
  PageFrame.drawString("4" , 30, 30);
  PageFrame.pushToSprite(&Display, 190, 72, TFT_BLACK);
  PageFrame.deleteSprite();

  PageFrame.createSprite(55, 62);
  PageFrame.fillSprite(TFT_BACKGROUND2);
  PageFrame.setTextColor(TOLGA_WHITE);
  PageFrame.setTextDatum(MC_DATUM);
  PageFrame.setTextColor(TOLGA_YELLOW);
  PageFrame.setFreeFont(&Orbitron_Bold_28);
  PageFrame.drawString("5" , 30, 30);
  PageFrame.pushToSprite(&Display, 250, 72, TFT_BLACK);
  PageFrame.deleteSprite();

  PageFrame.createSprite(75, 62);
  PageFrame.fillSprite(TFT_BACKGROUND2);
  PageFrame.setTextColor(TOLGA_WHITE);
  PageFrame.setTextDatum(MC_DATUM);
  PageFrame.setTextColor(TOLGA_YELLOW);
  PageFrame.setFreeFont(&Orbitron_Bold_28);
  PageFrame.drawString("BCK" , 37, 30);
  PageFrame.pushToSprite(&Display, 310, 72, TFT_BLACK);
  PageFrame.deleteSprite();

  PageFrame.createSprite(75, 62);
  PageFrame.fillSprite(TFT_BACKGROUND2);
  PageFrame.setTextColor(TOLGA_WHITE);
  PageFrame.setTextDatum(MC_DATUM);
  PageFrame.setTextColor(TOLGA_YELLOW);
  PageFrame.setFreeFont(&Orbitron_Bold_28);
  PageFrame.drawString("DEL" , 37, 30);
  PageFrame.pushToSprite(&Display, 390, 72, TFT_BLACK);
  PageFrame.deleteSprite();

  //

  PageFrame.createSprite(55, 62);
  PageFrame.fillSprite(TFT_BACKGROUND2);
  PageFrame.setTextColor(TOLGA_WHITE);
  PageFrame.setTextDatum(MC_DATUM);
  PageFrame.setTextColor(TOLGA_YELLOW);
  PageFrame.setFreeFont(&Orbitron_Bold_28);
  PageFrame.drawString("6" , 30, 30);
  PageFrame.pushToSprite(&Display, 10, 139, TFT_BLACK);
  PageFrame.deleteSprite();

  PageFrame.createSprite(55, 62);
  PageFrame.fillSprite(TFT_BACKGROUND2);
  PageFrame.setTextColor(TOLGA_WHITE);
  PageFrame.setTextDatum(MC_DATUM);
  PageFrame.setTextColor(TOLGA_YELLOW);
  PageFrame.setFreeFont(&Orbitron_Bold_28);
  PageFrame.drawString("7" , 30, 30);
  PageFrame.pushToSprite(&Display, 70, 139, TFT_BLACK);
  PageFrame.deleteSprite();


  PageFrame.createSprite(55, 62);
  PageFrame.fillSprite(TFT_BACKGROUND2);
  PageFrame.setTextColor(TOLGA_WHITE);
  PageFrame.setTextDatum(MC_DATUM);
  PageFrame.setTextColor(TOLGA_YELLOW);
  PageFrame.setFreeFont(&Orbitron_Bold_28);
  PageFrame.drawString("8" , 30, 30);
  PageFrame.pushToSprite(&Display, 130, 139, TFT_BLACK);
  PageFrame.deleteSprite();

  PageFrame.createSprite(55, 62);
  PageFrame.fillSprite(TFT_BACKGROUND2);
  PageFrame.setTextColor(TOLGA_WHITE);
  PageFrame.setTextDatum(MC_DATUM);
  PageFrame.setTextColor(TOLGA_YELLOW);
  PageFrame.setFreeFont(&Orbitron_Bold_28);
  PageFrame.drawString("9" , 30, 30);
  PageFrame.pushToSprite(&Display, 190, 139, TFT_BLACK);
  PageFrame.deleteSprite();

  PageFrame.createSprite(55, 62);
  PageFrame.fillSprite(TFT_BACKGROUND2);
  PageFrame.setTextColor(TOLGA_WHITE);
  PageFrame.setTextDatum(MC_DATUM);
  PageFrame.setTextColor(TOLGA_YELLOW);
  PageFrame.setFreeFont(&Orbitron_Bold_28);
  PageFrame.drawString("0" , 30, 30);
  PageFrame.pushToSprite(&Display, 250, 139, TFT_BLACK);
  PageFrame.deleteSprite();

  PageFrame.createSprite(75, 62);
  PageFrame.fillSprite(TFT_BACKGROUND2);
  PageFrame.setTextColor(TOLGA_WHITE);
  PageFrame.setTextDatum(MC_DATUM);
  PageFrame.setTextColor(TOLGA_YELLOW);
  PageFrame.setFreeFont(&Orbitron_Bold_28);
  PageFrame.drawString("." , 37, 30);
  PageFrame.pushToSprite(&Display, 310, 139, TFT_BLACK);
  PageFrame.deleteSprite();

  PageFrame.createSprite(75, 62);
  PageFrame.fillSprite(TFT_BACKGROUND2);
  PageFrame.setTextColor(TOLGA_WHITE);
  PageFrame.setTextDatum(MC_DATUM);
  PageFrame.setTextColor(TOLGA_YELLOW);
  PageFrame.setFreeFont(&Orbitron_Bold_28);
  PageFrame.drawString("CLR" , 37, 30);
  PageFrame.pushToSprite(&Display, 390, 139, TFT_BLACK);
  PageFrame.deleteSprite();





  Display.pushSprite(0, 60);
  Display.deleteSprite();


}
////////////////////////////////////////////////////

////////////////////////////////////////////////////

void drawWriteDry() {

  Display.createSprite(480, 210);
  Display.fillSprite(TFT_BLACK);


  PageFrame.createSprite(239, 50);
  PageFrame.fillSprite(TFT_BACKGROUND2);
  PageFrame.setTextColor(TOLGA_WHITE);
  Icon1.createSprite(24, 24);
  Icon1.setSwapBytes(true);
  Icon1.pushImage(0, 0, 24, 24, ip);
  Icon1.pushToSprite(&PageFrame, 20, 20, TFT_BLACK);
  PageFrame.setTextDatum(TL_DATUM);
  PageFrame.setTextColor(TFT_SILVER);
  PageFrame.setFreeFont(&Orbitron_Medium_12);
  PageFrame.drawString(Languages[indexLanguageArray][97] , 60, 5);
  PageFrame.setTextColor(TOLGA_WHITE);
  PageFrame.setFreeFont(&Orbitron_Bold_18);
  PageFrame.drawString(ContactNameList[ContactNameListIndex], 60, 25);
  PageFrame.pushToSprite(&Display, 0, 2, TFT_BLACK);
  PageFrame.deleteSprite();
  Icon1.deleteSprite();


  PageFrame.createSprite(239, 50);
  PageFrame.fillSprite(TFT_BACKGROUND2);
  PageFrame.setTextColor(TOLGA_WHITE);
  Icon1.createSprite(24, 24);
  Icon1.setSwapBytes(true);
  Icon1.pushImage(0, 0, 24, 24, port);
  Icon1.pushToSprite(&PageFrame, 20, 20, TFT_BLACK);
  PageFrame.setTextDatum(TL_DATUM);
  PageFrame.setTextColor(TFT_SILVER);
  PageFrame.setFreeFont(&Orbitron_Medium_12);
  PageFrame.drawString(Languages[indexLanguageArray][98] , 60, 5);
  PageFrame.setTextColor(TOLGA_WHITE);
  PageFrame.setFreeFont(&Orbitron_Bold_18);
  PageFrame.drawString(OperationList[OperationListIndex] , 60, 25);
  PageFrame.pushToSprite(&Display, 0, 54, TFT_BLACK);
  PageFrame.deleteSprite();
  Icon1.deleteSprite();


  PageFrame.createSprite(480, 50);
  PageFrame.fillSprite(TFT_BACKGROUND2);
  PageFrame.setTextColor(TOLGA_WHITE);
  Icon1.createSprite(24, 24);
  Icon1.setSwapBytes(true);
  Icon1.pushImage(0, 0, 24, 24, mac);
  Icon1.pushToSprite(&PageFrame, 20, 20, TFT_BLACK);
  PageFrame.setTextDatum(TL_DATUM);
  PageFrame.setTextColor(TFT_SILVER);
  PageFrame.setFreeFont(&Orbitron_Medium_12);
  PageFrame.drawString(Languages[indexLanguageArray][101] , 60, 5);
  PageFrame.setTextColor(TOLGA_WHITE);
  PageFrame.setFreeFont(&Orbitron_Bold_18);
  PageFrame.drawString(ContactFunctionList[ContactFunctionListIndex] , 60, 25);
  PageFrame.pushToSprite(&Display, 0, 106, TFT_BLACK);
  PageFrame.deleteSprite();
  Icon1.deleteSprite();


  PageFrame.createSprite(239, 50);
  PageFrame.fillSprite(TFT_BACKGROUND2);
  PageFrame.setTextColor(TOLGA_WHITE);
  Icon1.createSprite(24, 24);
  Icon1.setSwapBytes(true);
  Icon1.pushImage(0, 0, 24, 24, protocol);
  Icon1.pushToSprite(&PageFrame, 20, 20, TFT_BLACK);
  PageFrame.setTextDatum(TL_DATUM);
  PageFrame.setTextColor(TFT_SILVER);
  PageFrame.setFreeFont(&Orbitron_Medium_12);
  PageFrame.drawString(Languages[indexLanguageArray][99] , 60, 5);
  PageFrame.setTextColor(TOLGA_WHITE);
  PageFrame.setFreeFont(&Orbitron_Bold_18);
  PageFrame.drawString(DryEnable , 60, 25);
  PageFrame.pushToSprite(&Display, 241, 2, TFT_BLACK);
  PageFrame.deleteSprite();
  Icon1.deleteSprite();


  PageFrame.createSprite(239, 50);
  PageFrame.fillSprite(TFT_BACKGROUND2);
  PageFrame.setTextColor(TOLGA_WHITE);
  Icon1.createSprite(24, 24);
  Icon1.setSwapBytes(true);
  Icon1.pushImage(0, 0, 24, 24, ssid);
  Icon1.pushToSprite(&PageFrame, 20, 20, TFT_BLACK);
  PageFrame.setTextDatum(TL_DATUM);
  PageFrame.setTextColor(TFT_SILVER);
  PageFrame.setFreeFont(&Orbitron_Medium_12);
  PageFrame.drawString(Languages[indexLanguageArray][100] , 60, 5);
  PageFrame.setTextColor(TOLGA_WHITE);
  PageFrame.setFreeFont(&Orbitron_Bold_18);
  PageFrame.drawString(DryDisable, 60, 25);
  PageFrame.pushToSprite(&Display, 241, 54, TFT_BLACK);
  PageFrame.deleteSprite();
  Icon1.deleteSprite();



  PageFrame.createSprite(480, 50);
  PageFrame.fillSprite(TFT_BACKGROUND2);
  PageFrame.setTextColor(TOLGA_WHITE);
  PageFrame.setTextDatum(MC_DATUM);
  PageFrame.setTextColor(TOLGA_RED);
  PageFrame.setFreeFont(&Orbitron_Bold_18);
  PageFrame.drawString(Languages[indexLanguageArray][102] , 240, 20);
  PageFrame.pushToSprite(&Display, 0, 158, TFT_BLACK);
  PageFrame.deleteSprite();

  Display.pushSprite(0, 60);
  Display.deleteSprite();


}
////////////////////////////////////////////////////



void drawNetwork() {

  Display.createSprite(480, 210);
  Display.fillSprite(TFT_BLACK);


  PageFrame.createSprite(239, 50);
  PageFrame.fillSprite(TFT_BACKGROUND2);
  PageFrame.setTextColor(TOLGA_WHITE);
  Icon1.createSprite(24, 24);
  Icon1.setSwapBytes(true);
  Icon1.pushImage(0, 0, 24, 24, ip);
  Icon1.pushToSprite(&PageFrame, 20, 20, TFT_BLACK);
  PageFrame.setTextDatum(TL_DATUM);
  PageFrame.setTextColor(TFT_SILVER);
  PageFrame.setFreeFont(&Orbitron_Medium_12);
  PageFrame.drawString(Languages[indexLanguageArray][34] , 60, 5);
  PageFrame.setTextColor(TOLGA_WHITE);
  PageFrame.setFreeFont(&Orbitron_Bold_18);
  PageFrame.drawString(WiFi.localIP().toString().c_str(), 60, 25);
  PageFrame.pushToSprite(&Display, 0, 2, TFT_BLACK);
  PageFrame.deleteSprite();
  Icon1.deleteSprite();


  PageFrame.createSprite(239, 50);
  PageFrame.fillSprite(TFT_BACKGROUND2);
  PageFrame.setTextColor(TOLGA_WHITE);
  Icon1.createSprite(24, 24);
  Icon1.setSwapBytes(true);
  Icon1.pushImage(0, 0, 24, 24, port);
  Icon1.pushToSprite(&PageFrame, 20, 20, TFT_BLACK);
  PageFrame.setTextDatum(TL_DATUM);
  PageFrame.setTextColor(TFT_SILVER);
  PageFrame.setFreeFont(&Orbitron_Medium_12);
  PageFrame.drawString(Languages[indexLanguageArray][35] , 60, 5);
  PageFrame.setTextColor(TOLGA_WHITE);
  PageFrame.setFreeFont(&Orbitron_Bold_18);
  PageFrame.drawString("2001 " , 60, 25);
  PageFrame.pushToSprite(&Display, 0, 54, TFT_BLACK);
  PageFrame.deleteSprite();
  Icon1.deleteSprite();


  PageFrame.createSprite(480, 50);
  PageFrame.fillSprite(TFT_BACKGROUND2);
  PageFrame.setTextColor(TOLGA_WHITE);
  Icon1.createSprite(24, 24);
  Icon1.setSwapBytes(true);
  Icon1.pushImage(0, 0, 24, 24, mac);
  Icon1.pushToSprite(&PageFrame, 20, 20, TFT_BLACK);
  PageFrame.setTextDatum(TL_DATUM);
  PageFrame.setTextColor(TFT_SILVER);
  PageFrame.setFreeFont(&Orbitron_Medium_12);
  PageFrame.drawString(Languages[indexLanguageArray][38] , 60, 5);
  PageFrame.setTextColor(TOLGA_WHITE);
  PageFrame.setFreeFont(&Orbitron_Bold_18);
  PageFrame.drawString(WiFi.macAddress().c_str() , 60, 25);
  PageFrame.pushToSprite(&Display, 0, 106, TFT_BLACK);
  PageFrame.deleteSprite();
  Icon1.deleteSprite();


  PageFrame.createSprite(239, 50);
  PageFrame.fillSprite(TFT_BACKGROUND2);
  PageFrame.setTextColor(TOLGA_WHITE);
  Icon1.createSprite(24, 24);
  Icon1.setSwapBytes(true);
  Icon1.pushImage(0, 0, 24, 24, protocol);
  Icon1.pushToSprite(&PageFrame, 20, 20, TFT_BLACK);
  PageFrame.setTextDatum(TL_DATUM);
  PageFrame.setTextColor(TFT_SILVER);
  PageFrame.setFreeFont(&Orbitron_Medium_12);
  PageFrame.drawString(Languages[indexLanguageArray][36] , 60, 5);
  PageFrame.setTextColor(TOLGA_WHITE);
  PageFrame.setFreeFont(&Orbitron_Bold_18);
  PageFrame.drawString("UDP" , 60, 25);
  PageFrame.pushToSprite(&Display, 241, 2, TFT_BLACK);
  PageFrame.deleteSprite();
  Icon1.deleteSprite();


  PageFrame.createSprite(239, 50);
  PageFrame.fillSprite(TFT_BACKGROUND2);
  PageFrame.setTextColor(TOLGA_WHITE);
  Icon1.createSprite(24, 24);
  Icon1.setSwapBytes(true);
  Icon1.pushImage(0, 0, 24, 24, ssid);
  Icon1.pushToSprite(&PageFrame, 20, 20, TFT_BLACK);
  PageFrame.setTextDatum(TL_DATUM);
  PageFrame.setTextColor(TFT_SILVER);
  PageFrame.setFreeFont(&Orbitron_Medium_12);
  PageFrame.drawString(Languages[indexLanguageArray][37] , 60, 5);
  PageFrame.setTextColor(TOLGA_WHITE);
  PageFrame.setFreeFont(&Orbitron_Bold_18);
  PageFrame.drawString(WiFi.SSID().c_str() , 60, 25);
  PageFrame.pushToSprite(&Display, 241, 54, TFT_BLACK);
  PageFrame.deleteSprite();
  Icon1.deleteSprite();



  PageFrame.createSprite(480, 50);
  PageFrame.fillSprite(TFT_BACKGROUND2);
  PageFrame.setTextColor(TOLGA_WHITE);
  PageFrame.setTextDatum(MC_DATUM);
  PageFrame.setTextColor(TOLGA_RED);
  PageFrame.setFreeFont(&Orbitron_Bold_18);
  PageFrame.drawString(Languages[indexLanguageArray][39] , 240, 20);
  PageFrame.pushToSprite(&Display, 0, 158, TFT_BLACK);
  PageFrame.deleteSprite();

  Display.pushSprite(0, 60);
  Display.deleteSprite();

}




void drawSimulation() {

  Display.createSprite(480, 210);
  Display.fillSprite(TFT_BLACK);

  PageFrame.setFreeFont(&Orbitron_Bold_16);
  PageFrame.createSprite(80, 25);
  PageFrame.fillSprite(TFT_BACKGROUND2);
  PageFrame.setTextColor(TOLGA_WHITE);
  PageFrame.setTextDatum(MC_DATUM);
  PageFrame.setTextColor(TFT_SILVER);
  PageFrame.drawString( "Contact", 40, 10);
  PageFrame.pushToSprite(&Display, 0, 2, TFT_BLACK);
  PageFrame.deleteSprite();
  Icon1.deleteSprite();

  PageFrame.createSprite(160, 25);
  PageFrame.fillSprite(TFT_BACKGROUND2);
  PageFrame.setTextColor(TOLGA_WHITE);
  PageFrame.setTextDatum(MC_DATUM);
  PageFrame.setTextColor(TFT_SILVER);
  PageFrame.drawString( "Functionality", 80, 10);
  PageFrame.pushToSprite(&Display, 82, 2, TFT_BLACK);
  PageFrame.deleteSprite();
  Icon1.deleteSprite();


  PageFrame.createSprite(80, 25);
  PageFrame.fillSprite(TFT_BACKGROUND2);
  PageFrame.setTextColor(TOLGA_WHITE);
  PageFrame.setTextDatum(MC_DATUM);
  PageFrame.setTextColor(TFT_SILVER);
  PageFrame.drawString( "Enable", 40, 10);
  PageFrame.pushToSprite(&Display, 244, 2, TFT_BLACK);
  PageFrame.deleteSprite();
  Icon1.deleteSprite();



  ////////////////////////////////

  PageFrame.createSprite(80, 40);
  PageFrame.fillSprite(TFT_BACKGROUND2);
  PageFrame.setTextColor(TOLGA_WHITE);
  PageFrame.setTextDatum(MC_DATUM);
  PageFrame.setTextColor(TOLGA_YELLOW);
  PageFrame.drawString( "A", 40, 20);
  PageFrame.pushToSprite(&Display, 0, 29, TFT_BLACK);
  PageFrame.deleteSprite();
  Icon1.deleteSprite();

  PageFrame.createSprite(160, 40);
  PageFrame.fillSprite(TFT_BACKGROUND2);
  PageFrame.setTextColor(TOLGA_WHITE);
  PageFrame.setTextDatum(MC_DATUM);
  PageFrame.setTextColor(TOLGA_GREEN);
  PageFrame.drawString( "High Voltage", 80, 20);
  PageFrame.pushToSprite(&Display, 82, 29, TFT_BLACK);
  PageFrame.deleteSprite();
  Icon1.deleteSprite();


  PageFrame.createSprite(80, 40);
  PageFrame.fillSprite(TFT_BACKGROUND2);
  PageFrame.setTextColor(TOLGA_WHITE);
  PageFrame.setTextDatum(MC_DATUM);
  PageFrame.setTextColor(TOLGA_RED);
  PageFrame.drawString( "30V", 40, 20);
  PageFrame.pushToSprite(&Display, 244, 29, TFT_BLACK);
  PageFrame.deleteSprite();
  Icon1.deleteSprite();


  //////////////////////////


  PageFrame.createSprite(80, 45);
  PageFrame.fillSprite(TFT_BACKGROUND2);
  PageFrame.setTextColor(TOLGA_WHITE);
  PageFrame.setTextDatum(MC_DATUM);
  PageFrame.setTextColor(TOLGA_YELLOW);
  PageFrame.drawString( "B", 40, 20);
  PageFrame.pushToSprite(&Display, 0, 71, TFT_BLACK);
  PageFrame.deleteSprite();
  Icon1.deleteSprite();


  PageFrame.createSprite(160, 45);
  PageFrame.fillSprite(TFT_BACKGROUND2);
  PageFrame.setTextColor(TOLGA_WHITE);
  PageFrame.setTextDatum(MC_DATUM);
  PageFrame.setTextColor(TOLGA_GREEN);
  PageFrame.drawString( "Low Voltage", 80, 20);
  PageFrame.pushToSprite(&Display, 82, 71, TFT_BLACK);
  PageFrame.deleteSprite();
  Icon1.deleteSprite();



  PageFrame.createSprite(80, 45);
  PageFrame.fillSprite(TFT_BACKGROUND2);
  PageFrame.setTextColor(TOLGA_WHITE);
  PageFrame.setTextDatum(MC_DATUM);
  PageFrame.setTextColor(TOLGA_RED);
  PageFrame.drawString( "60V", 40, 20);
  PageFrame.pushToSprite(&Display, 244, 71, TFT_BLACK);
  PageFrame.deleteSprite();
  Icon1.deleteSprite();

  /////////////////////////////


  PageFrame.createSprite(80, 45);
  PageFrame.fillSprite(TFT_BACKGROUND2);
  PageFrame.setTextColor(TOLGA_WHITE);
  PageFrame.setTextDatum(MC_DATUM);
  PageFrame.setTextColor(TOLGA_YELLOW);
  PageFrame.drawString( "C", 40, 20);
  PageFrame.pushToSprite(&Display, 0, 118, TFT_BLACK);
  PageFrame.deleteSprite();
  Icon1.deleteSprite();


  PageFrame.createSprite(160, 45);
  PageFrame.fillSprite(TFT_BACKGROUND2);
  PageFrame.setTextColor(TOLGA_WHITE);
  PageFrame.setTextDatum(MC_DATUM);
  PageFrame.setTextColor(TOLGA_GREEN);
  PageFrame.drawString( "High Current", 80, 20);
  PageFrame.pushToSprite(&Display, 82, 118, TFT_BLACK);
  PageFrame.deleteSprite();
  Icon1.deleteSprite();



  PageFrame.createSprite(80, 45);
  PageFrame.fillSprite(TFT_BACKGROUND2);
  PageFrame.setTextColor(TOLGA_WHITE);
  PageFrame.setTextDatum(MC_DATUM);
  PageFrame.setTextColor(TOLGA_RED);
  PageFrame.drawString( "0A", 40, 20);
  PageFrame.pushToSprite(&Display, 244, 118, TFT_BLACK);
  PageFrame.deleteSprite();
  Icon1.deleteSprite();



  /////////////////////////////


  PageFrame.createSprite(80, 43);
  PageFrame.fillSprite(TFT_BACKGROUND2);
  PageFrame.setTextColor(TOLGA_WHITE);
  PageFrame.setTextDatum(MC_DATUM);
  PageFrame.setTextColor(TOLGA_YELLOW);
  PageFrame.drawString( "D", 40, 20);
  PageFrame.pushToSprite(&Display, 0, 165, TFT_BLACK);
  PageFrame.deleteSprite();
  Icon1.deleteSprite();


  PageFrame.createSprite(160, 43);
  PageFrame.fillSprite(TFT_BACKGROUND2);
  PageFrame.setTextColor(TOLGA_WHITE);
  PageFrame.setTextDatum(MC_DATUM);
  PageFrame.setTextColor(TOLGA_GREEN);
  PageFrame.drawString( "High Temp.", 80, 20);
  PageFrame.pushToSprite(&Display, 82, 165, TFT_BLACK);
  PageFrame.deleteSprite();
  Icon1.deleteSprite();



  PageFrame.createSprite(80, 43);
  PageFrame.fillSprite(TFT_BACKGROUND2);
  PageFrame.setTextColor(TOLGA_WHITE);
  PageFrame.setTextDatum(MC_DATUM);
  PageFrame.setTextColor(TOLGA_RED);
  PageFrame.drawString( "10C", 40, 20);
  PageFrame.pushToSprite(&Display, 244, 165, TFT_BLACK);
  PageFrame.deleteSprite();
  Icon1.deleteSprite();

  //////////////////////////////////////




  PageFrame.createSprite(154, 100);
  PageFrame.fillSprite(TFT_BACKGROUND2);
  PageFrame.setTextColor(TOLGA_WHITE);
  PageFrame.setTextDatum(MC_DATUM);
  PageFrame.setTextColor(TOLGA_GREEN);
  PageFrame.setFreeFont(&Orbitron_Bold_18);
  PageFrame.drawString( "Simulate", 77, 50);
  PageFrame.pushToSprite(&Display, 326, 2, TFT_BLACK);
  PageFrame.deleteSprite();
  Icon1.deleteSprite();



  PageFrame.createSprite(154, 104);
  PageFrame.fillSprite(TFT_BACKGROUND2);
  PageFrame.setTextColor(TOLGA_WHITE);
  PageFrame.setTextDatum(MC_DATUM);
  PageFrame.setTextColor(TFT_SILVER);
  PageFrame.setFreeFont(&Orbitron_Medium_12);
  PageFrame.drawString( "Simulation Status", 77, 30);
  PageFrame.setTextColor(TFT_WHITE);
  PageFrame.setFreeFont(&Orbitron_Bold_18);

  if (SimulationStatus) {
    PageFrame.drawString( "ENABLED", 77, 60);
  }
  else {
    PageFrame.drawString( "DISABLED", 77, 60);
  }
  PageFrame.pushToSprite(&Display, 326, 104, TFT_BLACK); \
  PageFrame.deleteSprite();
  Icon1.deleteSprite();



  Display.pushSprite(0, 60);
  Display.deleteSprite();

}










void drawLanguageList() {

  Serial.println("language list written");

  Display.createSprite(480, 210);
  Display.fillSprite(TFT_BLACK);

  PageFrame.createSprite(80, 206);
  PageFrame.fillSprite(TFT_BACKGROUND2);
  PageFrame.pushToSprite(&Display, 0, 2, TFT_BLACK);
  PageFrame.deleteSprite();

  PageFrame.createSprite(316, 50);
  PageFrame.setFreeFont(&Orbitron_Bold_18);
  PageFrame.fillSprite(TFT_BACKGROUND2);
  PageFrame.setTextDatum(MC_DATUM);
  PageFrame.drawString("Turkce" , 120, 25);

  Icon1.createSprite(24, 24);
  Icon1.setSwapBytes(true);
  Icon1.pushImage(0, 0, 24, 24, turkish);
  Icon1.pushToSprite(&PageFrame, 200, 15, TFT_BLACK);
  Icon1.deleteSprite();

  PageFrame.pushToSprite(&Display, 82, 2, TFT_BLACK);
  PageFrame.deleteSprite();


  PageFrame.createSprite(316, 50);
  PageFrame.setFreeFont(&Orbitron_Bold_18);
  PageFrame.fillSprite(TFT_BACKGROUND2);
  PageFrame.setTextDatum(MC_DATUM);
  PageFrame.drawString("English" , 120, 25);

  Icon1.createSprite(24, 24);
  Icon1.setSwapBytes(true);
  Icon1.pushImage(0, 0, 24, 24, english);
  Icon1.pushToSprite(&PageFrame, 200, 15, TFT_BLACK);
  Icon1.deleteSprite();

  PageFrame.pushToSprite(&Display, 82, 54, TFT_BLACK);
  PageFrame.deleteSprite();

  PageFrame.createSprite(316, 50);
  PageFrame.setFreeFont(&Orbitron_Bold_18);
  PageFrame.fillSprite(TFT_BACKGROUND2);
  PageFrame.setTextDatum(MC_DATUM);
  PageFrame.drawString("Deutsch" , 120, 25);

  Icon1.createSprite(24, 24);
  Icon1.setSwapBytes(true);
  Icon1.pushImage(0, 0, 24, 24, german);
  Icon1.pushToSprite(&PageFrame, 200, 15, TFT_BLACK);
  Icon1.deleteSprite();

  PageFrame.pushToSprite(&Display, 82, 106, TFT_BLACK);
  PageFrame.deleteSprite();

  PageFrame.createSprite(316, 50);
  PageFrame.setFreeFont(&Orbitron_Bold_18);
  PageFrame.fillSprite(TFT_BACKGROUND2);
  PageFrame.setTextDatum(MC_DATUM);
  PageFrame.drawString("Francais" , 120, 25);

  Icon1.createSprite(24, 24);
  Icon1.setSwapBytes(true);
  Icon1.pushImage(0, 0, 24, 24, french);
  Icon1.pushToSprite(&PageFrame, 200, 15, TFT_BLACK);
  Icon1.deleteSprite();

  PageFrame.pushToSprite(&Display, 82, 158, TFT_BLACK);
  PageFrame.deleteSprite();


  PageFrame.createSprite(80, 206);
  PageFrame.fillSprite(TFT_BACKGROUND2);
  PageFrame.pushToSprite(&Display, 400, 2, TFT_BLACK);
  PageFrame.deleteSprite();

  Display.pushSprite(0, 60);
  Display.deleteSprite();
}


/////////////

void drawAlarmLog() {



  String headerArray[9] = {Languages[indexLanguageArray][161], Languages[indexLanguageArray][162], Languages[indexLanguageArray][163], Languages[indexLanguageArray][164], Languages[indexLanguageArray][165], Languages[indexLanguageArray][166], Languages[indexLanguageArray][167], Languages[indexLanguageArray][168], Languages[indexLanguageArray][169]};
  int widthArray[9] = {35, 160, 110, 70, 80, 60, 60, 65, 65};
  int location = 0;
  String BMSAlarmList[256] = {"HSumVolt#1", "LSumVolt#1", "HSOC#1", "LSOC#1", "HCHC#1", "HDSC#1", "HDT#1", "LDT#1", "HCV#1", "LCV#1", "HCVD#1", "HTDiff#1",
                              "HSumVolt#2", "LSumVolt#2", "HSOC#2", "LSOC#2", "HCHC#2", "HDSC#2", "HCV#2", "LCV#2", "HCVD#2", "HDT#2", "LDT#2", "HTDiff#2",
                              "LCT#1", "LCT#2", "HCT#1", "HCT#2", "DContact", "CContact", "PCContact", "ROL", "HDM", "HCM", "DMM", "CMM,", "DCA", "CCS", "DCO", "CCO", "PF",
                              "UOC", "DCIS", "AFE", "SCD", "STSM", "EEPROMF", "RICM"
                             };
  BMSAlarmList[59] = "ChargingState";
  BMSAlarmList[60] = "DischargingState";
  BMSAlarmList[255] = "Undefined";






  for (int c = 0; c < 10; c++) {
    BMSAlarmArrayString[c][0] = String(BMSAlarmArray[c][0]);
    BMSAlarmArrayString[c][1] = String(BMSAlarmArray[c][1]) + "/" + String(BMSAlarmArray[c][2]) + "/" + String(BMSAlarmArray[c][3]) + " " + String(BMSAlarmArray[c][4]) + ":" + String(BMSAlarmArray[c][5]) + ":" + String(BMSAlarmArray[c][6]);
    BMSAlarmArrayString[c][2] =  BMSAlarmList[BMSAlarmArray[c][7]];
    BMSAlarmArrayString[c][3] = String(BMSAlarmArray[c][9] * 0.1) + "V";
    BMSAlarmArrayString[c][4] = String((BMSAlarmArray[c][10] * 0.1) - 3000) + "A";
    BMSAlarmArrayString[c][5] = String((BMSAlarmArray[c][11] * 0.1)) + "%";
    BMSAlarmArrayString[c][6] = String((BMSAlarmArray[c][18])) + "C";
    BMSAlarmArrayString[c][7] = String((BMSAlarmArray[c][14] * 0.001)) + "V";
    BMSAlarmArrayString[c][8] = String((BMSAlarmArray[c][16] * 0.001)) + "V";
  }











  Display.createSprite(720, 210);
  Display.fillSprite(TFT_BLACK);

  //header-------------------------------------
  ContentFrame.createSprite(720, 24);
  ContentFrame.fillSprite(TFT_BACKGROUND2);
  ContentFrame.setTextColor(TFT_WHITE);

  location = 0;
  for (int i = 0; i < 9; i++) {

    ContentFrame.drawLine(location, 0, location, 24, TFT_BACKGROUND);
    ContentFrame.drawString(headerArray[i], location + 10, 7, 2);
    location = location + widthArray[i];

  }

  if (!pulled) {
    ContentFrame.pushToSprite(&Display, 0, 2, TFT_BLACK);
  }
  else {
    ContentFrame.pushToSprite(&Display, -220, 2, TFT_BLACK);
  }
  ContentFrame.deleteSprite();
  //---------------------------------------

  //TABLE
  ContentFrame.createSprite(720, 180);
  ContentFrame.fillSprite(TFT_WHITE);
  ContentFrame.setTextColor(TFT_BLACK);

  location = 0;

  for (int i = 0; i < 9; i++) {
    ContentFrame.drawLine(location, 0, location, 180, TFT_BACKGROUND2);
    location = location + widthArray[i];
  }

  for (int p = 0; p < 5; p++) {
    ContentFrame.drawLine(0, 30 + 30 * p, 720, 30 + 30 * p, TFT_BACKGROUND2);
  }





  for (int x = 0; x < 5; x++) {
    location = 0;

    for (int y = 0; y < 9; y++) {


      if (!pulled2) {
        //Serial.println("unpulled");
        ContentFrame.drawString(BMSAlarmArrayString[x][y], 10 + location, 10 + x * 30, 2);
      }
      else {
        // Serial.println("pulled");
        ContentFrame.drawString(BMSAlarmArrayString[x + 5][y], 10 + location, 10 + x * 30, 2);
      }




      location = location + widthArray[y];
    }

  }









  if (!pulled) {
    //Serial.println("unpulled");
    ContentFrame.pushToSprite(&Display, 0, 26, TFT_BLACK);
  }
  else {
    //  Serial.println("pulled");
    ContentFrame.pushToSprite(&Display, -220, 26, TFT_BLACK);
  }
  ContentFrame.deleteSprite();
  //------




  //horizontal slider------------------
  ContentFrame.createSprite(600, 32);
  ContentFrame.fillSprite(TFT_BACKGROUND2);
  ContentFrame.setTextColor(TFT_SILVER);

  Icon1.createSprite(24, 24);
  Icon1.setSwapBytes(true);
  Icon1.pushImage(0, 0, 24, 24, rightScroll);
  Icon1.pushToSprite(&ContentFrame, 450, 4, TFT_BLACK);
  Icon1.deleteSprite();

  Icon1.createSprite(24, 24);
  Icon1.setSwapBytes(true);
  Icon1.pushImage(0, 0, 24, 24, leftScroll);
  Icon1.pushToSprite(&ContentFrame, 6, 4, TFT_BLACK);
  Icon1.deleteSprite();



  Icon1.createSprite(120, 15);
  Icon1.setSwapBytes(true);
  Icon1.pushImage(0, 0, 120, 15, scrollbar);


  if (!pulled) {
    Icon1.pushToSprite(&ContentFrame, 40, 8, TFT_BLACK);
  }
  else {
    Icon1.pushToSprite(&ContentFrame, 320, 8, TFT_BLACK);

  }

  Icon1.deleteSprite();

  ContentFrame.pushToSprite(&Display, 0, 177, TFT_BLACK);

  ContentFrame.deleteSprite();

  //-------------------------------------



  Display.pushSprite(0, 60);



  Display.deleteSprite();



}






//////////

void drawStatistics() {

  Display.createSprite(480, 210);
  Display.fillSprite(TFT_BLACK);




  PageFrame.createSprite(239, 50);
  PageFrame.fillSprite(TFT_BACKGROUND2);
  PageFrame.setTextColor(TOLGA_WHITE);
  Icon1.createSprite(24, 24);
  Icon1.setSwapBytes(true);
  Icon1.pushImage(0, 0, 24, 24, voltage);
  Icon1.pushToSprite(&PageFrame, 20, 20, TFT_BLACK);
  PageFrame.setTextDatum(TL_DATUM);
  PageFrame.setTextColor(TFT_SILVER);
  PageFrame.setFreeFont(&Orbitron_Medium_12);
  PageFrame.drawString(Languages[indexLanguageArray][40], 60, 5);
  PageFrame.setTextColor(TOLGA_WHITE);
  PageFrame.setFreeFont(&Orbitron_Bold_18);
  PageFrame.drawString(String(MaxVoltage) + " V", 60, 25);
  PageFrame.pushToSprite(&Display, 0, 2, TFT_BLACK);
  PageFrame.deleteSprite();
  Icon1.deleteSprite();


  PageFrame.createSprite(239, 50);
  PageFrame.fillSprite(TFT_BACKGROUND2);
  PageFrame.setTextColor(TOLGA_WHITE);
  Icon1.createSprite(24, 24);
  Icon1.setSwapBytes(true);
  Icon1.pushImage(0, 0, 24, 24, voltage);
  Icon1.pushToSprite(&PageFrame, 20, 20, TFT_BLACK);
  PageFrame.setTextDatum(TL_DATUM);
  PageFrame.setTextColor(TFT_SILVER);
  PageFrame.setFreeFont(&Orbitron_Medium_12);
  PageFrame.drawString(Languages[indexLanguageArray][41] , 60, 5);
  PageFrame.setTextColor(TOLGA_WHITE);
  PageFrame.setFreeFont(&Orbitron_Bold_18);
  PageFrame.drawString(String(MinVoltage) + " V" , 60, 25);
  PageFrame.pushToSprite(&Display, 241, 2, TFT_BLACK);
  PageFrame.deleteSprite();
  Icon1.deleteSprite();

  PageFrame.createSprite(239, 50);
  PageFrame.fillSprite(TFT_BACKGROUND2);
  PageFrame.setTextColor(TOLGA_WHITE);
  Icon1.createSprite(24, 24);
  Icon1.setSwapBytes(true);
  Icon1.pushImage(0, 0, 24, 24, current);
  Icon1.pushToSprite(&PageFrame, 20, 20, TFT_BLACK);
  PageFrame.setTextDatum(TL_DATUM);
  PageFrame.setTextColor(TFT_SILVER);
  PageFrame.setFreeFont(&Orbitron_Medium_12);
  PageFrame.drawString(Languages[indexLanguageArray][42], 60, 5);
  PageFrame.setTextColor(TOLGA_WHITE);
  PageFrame.setFreeFont(&Orbitron_Bold_18);
  PageFrame.drawString(String(MaxCurrent) + " A" , 60, 25);
  PageFrame.pushToSprite(&Display, 0, 54, TFT_BLACK);
  PageFrame.deleteSprite();
  Icon1.deleteSprite();


  PageFrame.createSprite(239, 50);
  PageFrame.fillSprite(TFT_BACKGROUND2);
  PageFrame.setTextColor(TOLGA_WHITE);
  Icon1.createSprite(24, 24);
  Icon1.setSwapBytes(true);
  Icon1.pushImage(0, 0, 24, 24, current);
  Icon1.pushToSprite(&PageFrame, 20, 20, TFT_BLACK);
  PageFrame.setTextDatum(TL_DATUM);
  PageFrame.setTextColor(TFT_SILVER);
  PageFrame.setFreeFont(&Orbitron_Medium_12);
  PageFrame.drawString(Languages[indexLanguageArray][43] , 60, 5);
  PageFrame.setTextColor(TOLGA_WHITE);
  PageFrame.setFreeFont(&Orbitron_Bold_18);
  PageFrame.drawString(String(MinCurrent) + " A" , 60, 25);
  PageFrame.pushToSprite(&Display, 241, 54, TFT_BLACK);
  PageFrame.deleteSprite();
  Icon1.deleteSprite();




  PageFrame.createSprite(239, 50);
  PageFrame.fillSprite(TFT_BACKGROUND2);
  PageFrame.setTextColor(TOLGA_WHITE);
  Icon1.createSprite(24, 24);
  Icon1.setSwapBytes(true);
  Icon1.pushImage(0, 0, 24, 24, temp);
  Icon1.pushToSprite(&PageFrame, 20, 20, TFT_BLACK);
  PageFrame.setTextDatum(TL_DATUM);
  PageFrame.setTextColor(TFT_SILVER);
  PageFrame.setFreeFont(&Orbitron_Medium_12);
  PageFrame.drawString(Languages[indexLanguageArray][44] , 60, 5);
  PageFrame.setTextColor(TOLGA_WHITE);
  PageFrame.setFreeFont(&Orbitron_Bold_18);
  PageFrame.drawString(String(MaxTemp1) + " C", 60, 25);
  PageFrame.pushToSprite(&Display, 0, 106, TFT_BLACK);
  PageFrame.deleteSprite();
  Icon1.deleteSprite();


  PageFrame.createSprite(239, 50);
  PageFrame.fillSprite(TFT_BACKGROUND2);
  PageFrame.setTextColor(TOLGA_WHITE);
  Icon1.createSprite(24, 24);
  Icon1.setSwapBytes(true);
  Icon1.pushImage(0, 0, 24, 24, temp);
  Icon1.pushToSprite(&PageFrame, 20, 20, TFT_BLACK);
  PageFrame.setTextDatum(TL_DATUM);
  PageFrame.setTextColor(TFT_SILVER);
  PageFrame.setFreeFont(&Orbitron_Medium_12);
  PageFrame.drawString(Languages[indexLanguageArray][45] , 60, 5);
  PageFrame.setTextColor(TOLGA_WHITE);
  PageFrame.setFreeFont(&Orbitron_Bold_18);
  PageFrame.drawString(String(MaxTemp2) + " C", 60, 25);
  PageFrame.pushToSprite(&Display, 241, 106, TFT_BLACK);
  PageFrame.deleteSprite();
  Icon1.deleteSprite();



  PageFrame.createSprite(480, 50);
  PageFrame.fillSprite(TFT_BACKGROUND2);
  PageFrame.setTextColor(TOLGA_WHITE);
  PageFrame.setTextDatum(MC_DATUM);
  PageFrame.setTextColor(TOLGA_RED);
  PageFrame.setFreeFont(&Orbitron_Bold_18);
  PageFrame.drawString(Languages[indexLanguageArray][46] , 240, 20);
  PageFrame.pushToSprite(&Display, 0, 158, TFT_BLACK);
  PageFrame.deleteSprite();

  Display.pushSprite(0, 60);
  Display.deleteSprite();


}



void drawSystemSetup() {
  Display.createSprite(480, 210);
  Display.fillSprite(TFT_BLACK);

  PageFrame.createSprite(239, 50);
  PageFrame.fillSprite(TFT_BACKGROUND2);
  PageFrame.setTextColor(TOLGA_WHITE);
  Icon1.createSprite(36, 36);
  Icon1.setSwapBytes(true);

  if (mute) {
    Icon1.pushImage(0, 0, 36, 36, novolume);
  }

  else {
    Icon1.pushImage(0, 0, 36, 36, volume);
  }

  Icon1.pushToSprite(&PageFrame, 20, 6, TFT_BLACK);
  PageFrame.setTextDatum(TL_DATUM);
  PageFrame.setTextColor(TFT_SILVER);
  PageFrame.setFreeFont(&Orbitron_Medium_12);
  PageFrame.drawString(Languages[indexLanguageArray][87] , 80, 5);
  PageFrame.setTextColor(TOLGA_WHITE);
  PageFrame.setFreeFont(&Orbitron_Bold_18);
  PageFrame.setTextDatum(TL_DATUM);

  if (mute) {
    PageFrame.drawString("Disabled" , 80, 20);
  }
  else {
    PageFrame.drawString("Enabled" , 80, 20);
  }

  PageFrame.pushToSprite(&Display, 0, 2, TFT_BLACK);
  PageFrame.deleteSprite();
  Icon1.deleteSprite();



  PageFrame.createSprite(239, 50);
  PageFrame.fillSprite(TFT_BACKGROUND2);
  PageFrame.setTextColor(TOLGA_WHITE);
  Icon1.createSprite(36, 36);
  Icon1.setSwapBytes(true);

  if (Safety) {
    Icon1.pushImage(0, 0, 36, 36, safety);
  }

  else {
    Icon1.pushImage(0, 0, 36, 36, nosafety);
  }


  Icon1.pushToSprite(&PageFrame, 20, 6, TFT_BLACK);
  PageFrame.setTextDatum(TL_DATUM);
  PageFrame.setTextColor(TFT_SILVER);
  PageFrame.setFreeFont(&Orbitron_Medium_12);
  PageFrame.drawString(Languages[indexLanguageArray][88] , 80, 5);
  PageFrame.setTextColor(TOLGA_WHITE);
  PageFrame.setFreeFont(&Orbitron_Bold_18);
  PageFrame.setTextDatum(TL_DATUM);

  if (Safety) {
    PageFrame.drawString("Enabled" , 80, 20);
  }
  else {
    PageFrame.drawString("Disabled" , 80, 20);
  }

  PageFrame.pushToSprite(&Display, 241, 2, TFT_BLACK);
  PageFrame.deleteSprite();
  Icon1.deleteSprite();





  //////////////


  PageFrame.createSprite(239, 50);
  PageFrame.fillSprite(TFT_BACKGROUND2);
  PageFrame.setTextColor(TOLGA_WHITE);

  Icon1.createSprite(36, 36);
  Icon1.setSwapBytes(true);
  Icon1.pushImage(0, 0, 36, 36, canid);
  Icon1.pushToSprite(&PageFrame, 20, 6, TFT_BLACK);

  PageFrame.setTextDatum(TL_DATUM);
  PageFrame.setTextColor(TFT_SILVER);
  PageFrame.setFreeFont(&Orbitron_Medium_12);
  PageFrame.drawString(Languages[indexLanguageArray][89] , 80, 5);
  PageFrame.setTextColor(TOLGA_WHITE);
  PageFrame.setFreeFont(&Orbitron_Bold_18);
  PageFrame.setTextDatum(TL_DATUM);
  PageFrame.drawString(String(SettedCANID) , 80, 20);

  PageFrame.pushToSprite(&Display, 0, 54, TFT_BLACK);
  PageFrame.deleteSprite();
  Icon1.deleteSprite();


  PageFrame.createSprite(239, 50);
  PageFrame.fillSprite(TFT_BACKGROUND2);
  PageFrame.setTextColor(TOLGA_WHITE);

  Icon1.createSprite(36, 36);
  Icon1.setSwapBytes(true);
  Icon1.pushImage(0, 0, 36, 36, canid);
  Icon1.pushToSprite(&PageFrame, 20, 6, TFT_BLACK);

  PageFrame.setTextDatum(TL_DATUM);
  PageFrame.setTextColor(TFT_SILVER);
  PageFrame.setFreeFont(&Orbitron_Medium_12);
  PageFrame.drawString(Languages[indexLanguageArray][90] , 80, 5);
  PageFrame.setTextColor(TOLGA_WHITE);
  PageFrame.setFreeFont(&Orbitron_Bold_18);
  PageFrame.setTextDatum(TL_DATUM);
  PageFrame.drawString(String(BMS.board_number), 80, 20);

  PageFrame.pushToSprite(&Display, 241, 54, TFT_BLACK);
  PageFrame.deleteSprite();
  Icon1.deleteSprite();

  ///////////////

  PageFrame.createSprite(480, 50);
  PageFrame.fillSprite(TFT_BACKGROUND2);
  PageFrame.setTextColor(TOLGA_WHITE);
  PageFrame.setTextDatum(MC_DATUM);
  PageFrame.setTextColor(TOLGA_RED);
  PageFrame.setFreeFont(&Orbitron_Bold_18);
  PageFrame.drawString(Languages[indexLanguageArray][91] , 240, 25);
  PageFrame.setTextColor(TOLGA_WHITE);
  PageFrame.pushToSprite(&Display, 0, 106, TFT_BLACK);
  PageFrame.deleteSprite();

  PageFrame.createSprite(480, 50);
  PageFrame.fillSprite(TFT_BACKGROUND2);
  PageFrame.setTextColor(TOLGA_WHITE);

  PageFrame.setTextDatum(MC_DATUM);
  PageFrame.setTextColor(TOLGA_RED);
  PageFrame.setFreeFont(&Orbitron_Bold_18);
  PageFrame.drawString(Languages[indexLanguageArray][92] , 240, 25);
  PageFrame.setTextColor(TOLGA_WHITE);

  Icon1.createSprite(24, 24);
  Icon1.setSwapBytes(true);


  switch (indexLanguageArray) {

    case 0:
      Icon1.pushImage(0, 0, 24, 24, english);
      break;

    case 1:
      Icon1.pushImage(0, 0, 24, 24, turkish);
      break;

    case 2:
      Icon1.pushImage(0, 0, 24, 24, german);
      break;
    case 3:
      Icon1.pushImage(0, 0, 24, 24, french);
      break;
    default:
      break;

  }


  Icon1.pushToSprite(&PageFrame, 360, 18, TFT_BLACK);


  PageFrame.pushToSprite(&Display, 0, 158, TFT_BLACK);
  PageFrame.deleteSprite();
  Icon1.deleteSprite();






  Display.pushSprite(0, 60);
  Display.deleteSprite();
}
////////////////////////////////////////////////////////////////////////

void drawInverterSetup() {
  Display.createSprite(480, 210);
  Display.fillSprite(TFT_BLACK);

  PageFrame.createSprite(239, 50);
  PageFrame.fillSprite(TFT_BACKGROUND2);
  PageFrame.setTextColor(TOLGA_WHITE);
  Icon1.createSprite(24, 24);
  Icon1.setSwapBytes(true);
  Icon1.pushImage(0, 0, 24, 24, inverter);
  Icon1.pushToSprite(&PageFrame, 20, 16, TFT_BLACK);
  PageFrame.setTextDatum(TL_DATUM);
  PageFrame.setTextColor(TFT_SILVER);
  PageFrame.setFreeFont(&Orbitron_Medium_12);
  PageFrame.drawString(Languages[indexLanguageArray][153] , 80, 5);
  PageFrame.setTextColor(TOLGA_WHITE);
  PageFrame.setFreeFont(&Orbitron_Bold_18);
  PageFrame.setTextDatum(TL_DATUM);
  PageFrame.drawString( INVERTERS[indexINVERTERSArray], 80, 20);
  PageFrame.pushToSprite(&Display, 0, 2, TFT_BLACK);
  PageFrame.deleteSprite();
  Icon1.deleteSprite();



  PageFrame.createSprite(239, 50);
  PageFrame.fillSprite(TFT_BACKGROUND2);
  PageFrame.setTextColor(TOLGA_WHITE);
  Icon1.createSprite(24, 24);
  Icon1.setSwapBytes(true);
  Icon1.pushImage(0, 0, 24, 24, inverter);
  Icon1.pushToSprite(&PageFrame, 20, 16, TFT_BLACK);
  PageFrame.setTextDatum(TL_DATUM);
  PageFrame.setTextColor(TFT_SILVER);
  PageFrame.setFreeFont(&Orbitron_Medium_12);
  PageFrame.drawString(Languages[indexLanguageArray][155] , 80, 5);
  PageFrame.setTextColor(TOLGA_WHITE);
  PageFrame.setFreeFont(&Orbitron_Bold_18);
  PageFrame.setTextDatum(TL_DATUM);
  PageFrame.drawString(BMS.commname , 80, 20);
  PageFrame.pushToSprite(&Display, 241, 2, TFT_BLACK);
  PageFrame.deleteSprite();
  Icon1.deleteSprite();

  //Serial.println(BMS.commname);
  // Serial.println(BMS.commmeth);



  //////////////


  PageFrame.createSprite(239, 50);
  PageFrame.fillSprite(TFT_BACKGROUND2);
  PageFrame.setTextColor(TOLGA_WHITE);
  Icon1.createSprite(24, 24);
  Icon1.setSwapBytes(true);
  Icon1.pushImage(0, 0, 24, 24, ethernet);
  Icon1.pushToSprite(&PageFrame, 20, 16, TFT_BLACK);
  PageFrame.setTextDatum(TL_DATUM);
  PageFrame.setTextColor(TFT_SILVER);
  PageFrame.setFreeFont(&Orbitron_Medium_12);
  PageFrame.drawString(Languages[indexLanguageArray][154] , 80, 5);
  PageFrame.setTextColor(TOLGA_WHITE);
  PageFrame.setFreeFont(&Orbitron_Bold_18);
  PageFrame.setTextDatum(TL_DATUM);
  PageFrame.drawString(COMM[indexCOMMArray] , 80, 20);
  PageFrame.pushToSprite(&Display, 0, 54, TFT_BLACK);
  PageFrame.deleteSprite();
  Icon1.deleteSprite();


  PageFrame.createSprite(239, 50);
  PageFrame.fillSprite(TFT_BACKGROUND2);
  PageFrame.setTextColor(TOLGA_WHITE);

  Icon1.createSprite(24, 24);
  Icon1.setSwapBytes(true);
  Icon1.pushImage(0, 0, 24, 24, ethernet);
  Icon1.pushToSprite(&PageFrame, 20, 16, TFT_BLACK);
  PageFrame.setTextDatum(TL_DATUM);
  PageFrame.setTextColor(TFT_SILVER);
  PageFrame.setFreeFont(&Orbitron_Medium_12);
  PageFrame.drawString(Languages[indexLanguageArray][156] , 80, 5);
  PageFrame.setTextColor(TOLGA_WHITE);
  PageFrame.setFreeFont(&Orbitron_Bold_18);
  PageFrame.setTextDatum(TL_DATUM);
  PageFrame.drawString(BMS.commmeth, 80, 20);
  PageFrame.pushToSprite(&Display, 241, 54, TFT_BLACK);
  PageFrame.deleteSprite();
  Icon1.deleteSprite();

  ///////////////

  PageFrame.createSprite(480, 50);
  PageFrame.fillSprite(TFT_BACKGROUND2);
  PageFrame.setTextColor(TOLGA_WHITE);
  PageFrame.setTextDatum(MC_DATUM);
  PageFrame.setTextColor(TOLGA_RED);
  PageFrame.setFreeFont(&Orbitron_Bold_18);
  PageFrame.drawString(Languages[indexLanguageArray][157] , 240, 25);
  PageFrame.setTextColor(TOLGA_WHITE);
  PageFrame.pushToSprite(&Display, 0, 106, TFT_BLACK);
  PageFrame.deleteSprite();

  PageFrame.createSprite(480, 50);
  PageFrame.fillSprite(TFT_BACKGROUND2);
  PageFrame.setTextColor(TOLGA_WHITE);
  PageFrame.setTextDatum(MC_DATUM);
  PageFrame.setTextColor(TOLGA_RED);
  PageFrame.setFreeFont(&Orbitron_Bold_18);
  PageFrame.setTextColor(TOLGA_WHITE);
  PageFrame.pushToSprite(&Display, 0, 158, TFT_BLACK);
  PageFrame.deleteSprite();

  Display.pushSprite(0, 60);
  Display.deleteSprite();
}









////////////////////////
void drawSystemStats() {

  Display.createSprite(480, 210);
  Display.fillSprite(TFT_BLACK);
  PageFrame.createSprite(239, 50);
  PageFrame.fillSprite(TFT_BACKGROUND2);
  PageFrame.setTextColor(TOLGA_WHITE);
  Icon1.createSprite(24, 24);
  Icon1.setSwapBytes(true);
  Icon1.pushImage(0, 0, 24, 24, charge);
  Icon1.pushToSprite(&PageFrame, 20, 20, TFT_BLACK);
  PageFrame.setTextDatum(TL_DATUM);
  PageFrame.setTextColor(TFT_SILVER);
  PageFrame.setFreeFont(&Orbitron_Medium_12);
  PageFrame.drawString(Languages[indexLanguageArray][103], 60, 5);
  PageFrame.setTextColor(TOLGA_WHITE);
  PageFrame.setFreeFont(&Orbitron_Bold_18);
  PageFrame.drawString(String(ChargeFix) + " kWh", 60, 25);
  PageFrame.pushToSprite(&Display, 0, 2, TFT_BLACK);
  PageFrame.deleteSprite();
  Icon1.deleteSprite();


  PageFrame.createSprite(239, 50);
  PageFrame.fillSprite(TFT_BACKGROUND2);
  PageFrame.setTextColor(TOLGA_WHITE);
  Icon1.createSprite(24, 24);
  Icon1.setSwapBytes(true);
  Icon1.pushImage(0, 0, 24, 24, discharge);
  Icon1.pushToSprite(&PageFrame, 20, 20, TFT_BLACK);
  PageFrame.setTextDatum(TL_DATUM);
  PageFrame.setTextColor(TFT_SILVER);
  PageFrame.setFreeFont(&Orbitron_Medium_12);
  PageFrame.drawString(Languages[indexLanguageArray][104] , 60, 5);
  PageFrame.setTextColor(TOLGA_WHITE);
  PageFrame.setFreeFont(&Orbitron_Bold_18);
  PageFrame.drawString(String(DischargeFix) + " kWh" , 60, 25);
  PageFrame.pushToSprite(&Display, 241, 2, TFT_BLACK);
  PageFrame.deleteSprite();
  Icon1.deleteSprite();

  PageFrame.createSprite(239, 50);
  PageFrame.fillSprite(TFT_BACKGROUND2);
  PageFrame.setTextColor(TOLGA_WHITE);
  Icon1.createSprite(24, 24);
  Icon1.setSwapBytes(true);
  Icon1.pushImage(0, 0, 24, 24, current);
  Icon1.pushToSprite(&PageFrame, 20, 20, TFT_BLACK);
  PageFrame.setTextDatum(TL_DATUM);
  PageFrame.setTextColor(TFT_SILVER);
  PageFrame.setFreeFont(&Orbitron_Medium_12);
  PageFrame.drawString(Languages[indexLanguageArray][105] , 60, 5);
  PageFrame.setTextColor(TOLGA_WHITE);
  PageFrame.setFreeFont(&Orbitron_Bold_18);
  PageFrame.drawString(String(MaxCurrentP) + " A" , 60, 25);
  PageFrame.pushToSprite(&Display, 0, 54, TFT_BLACK);
  PageFrame.deleteSprite();
  Icon1.deleteSprite();


  PageFrame.createSprite(239, 50);
  PageFrame.fillSprite(TFT_BACKGROUND2);
  PageFrame.setTextColor(TOLGA_WHITE);
  Icon1.createSprite(24, 24);
  Icon1.setSwapBytes(true);
  Icon1.pushImage(0, 0, 24, 24, current);
  Icon1.pushToSprite(&PageFrame, 20, 20, TFT_BLACK);
  PageFrame.setTextDatum(TL_DATUM);
  PageFrame.setTextColor(TFT_SILVER);
  PageFrame.setFreeFont(&Orbitron_Medium_12);
  PageFrame.drawString(Languages[indexLanguageArray][106] , 60, 5);
  PageFrame.setTextColor(TOLGA_WHITE);
  PageFrame.setFreeFont(&Orbitron_Bold_18);
  PageFrame.drawString(String(MinCurrentP) + " A" , 60, 25);
  PageFrame.pushToSprite(&Display, 241, 54, TFT_BLACK);
  PageFrame.deleteSprite();
  Icon1.deleteSprite();




  PageFrame.createSprite(480, 24);
  PageFrame.fillSprite(TFT_BACKGROUND2);
  PageFrame.setTextColor(TOLGA_WHITE);

  PageFrame.setTextDatum(TL_DATUM);
  PageFrame.setTextColor(TFT_SILVER);
  PageFrame.setFreeFont(&Orbitron_Medium_12);
  PageFrame.drawString(Languages[indexLanguageArray][107] , 60, 5);
  PageFrame.setTextColor(TOLGA_WHITE);
  PageFrame.setFreeFont(&Orbitron_Medium_12);
  PageFrame.drawString(String(OpDay) + " Day " + String(OpHour) + " Hour " + String(OpMin) + " Min. " + String(OpSec, 0) + " Sec. ", 200, 5);
  PageFrame.pushToSprite(&Display, 0, 106, TFT_BLACK);
  PageFrame.deleteSprite();
  Icon1.deleteSprite();









  PageFrame.createSprite(480, 24);
  PageFrame.fillSprite(TFT_BACKGROUND2);
  PageFrame.setTextColor(TOLGA_WHITE);
  PageFrame.setTextDatum(TL_DATUM);

  PageFrame.setTextColor(TFT_SILVER);
  PageFrame.setFreeFont(&Orbitron_Medium_12);
  PageFrame.drawString(Languages[indexLanguageArray][108] , 60, 5);
  PageFrame.setTextColor(TOLGA_WHITE);
  PageFrame.setFreeFont(&Orbitron_Medium_12);
  PageFrame.drawString(SerialNumber, 170, 5);

  PageFrame.pushToSprite(&Display, 0, 132, TFT_BLACK);
  PageFrame.deleteSprite();



  PageFrame.createSprite(480, 24);
  PageFrame.fillSprite(TFT_BACKGROUND2);
  PageFrame.setTextColor(TOLGA_WHITE);
  PageFrame.setTextDatum(TL_DATUM);



  PageFrame.setTextColor(TFT_SILVER);
  PageFrame.setFreeFont(&Orbitron_Medium_12);
  PageFrame.drawString(Languages[indexLanguageArray][109] , 60, 5);
  PageFrame.setTextColor(TOLGA_WHITE);
  PageFrame.setFreeFont(&Orbitron_Medium_12);
  PageFrame.drawString(bmsserialnumber, 170, 5);
  PageFrame.pushToSprite(&Display, 0, 158, TFT_BLACK);
  PageFrame.deleteSprite();

  PageFrame.createSprite(480, 24);
  PageFrame.fillSprite(TFT_BACKGROUND2);
  PageFrame.setTextColor(TOLGA_WHITE);
  PageFrame.setTextDatum(TL_DATUM);

  PageFrame.setTextColor(TFT_SILVER);
  PageFrame.setFreeFont(&Orbitron_Medium_12);
  PageFrame.drawString("Memory (kb)" , 60, 5);
  PageFrame.setTextColor(TOLGA_WHITE);
  PageFrame.setFreeFont(&Orbitron_Medium_12);
  PageFrame.drawString(String(ConvBinUnits(ESP.getFreeHeap(), 3)), 170, 5);
  PageFrame.pushToSprite(&Display, 0, 184, TFT_BLACK);
  PageFrame.deleteSprite();





  //bmsserialnumber






  Display.pushSprite(0, 60);
  Display.deleteSprite();
}


////////////////


void drawSystemDetails() {

  Display.createSprite(480, 210);
  Display.fillSprite(TFT_BLACK);


  PageFrame.createSprite(239, 50);
  PageFrame.fillSprite(TFT_BACKGROUND2);
  PageFrame.setTextColor(TOLGA_WHITE);
  Icon1.createSprite(24, 24);
  Icon1.setSwapBytes(true);
  Icon1.pushImage(0, 0, 24, 24, current);
  Icon1.pushToSprite(&PageFrame, 20, 20, TFT_BLACK);
  PageFrame.setTextDatum(TL_DATUM);
  PageFrame.setTextColor(TFT_SILVER);
  PageFrame.setFreeFont(&Orbitron_Medium_12);
  PageFrame.drawString(Languages[indexLanguageArray][56] , 60, 5);
  PageFrame.setTextColor(TOLGA_WHITE);
  PageFrame.setFreeFont(&Orbitron_Bold_18);
  PageFrame.drawString(String(BMS.dischar_curr2) + " A", 60, 25);
  PageFrame.pushToSprite(&Display, 0, 2, TFT_BLACK);
  PageFrame.deleteSprite();
  Icon1.deleteSprite();


  PageFrame.createSprite(239, 50);
  PageFrame.fillSprite(TFT_BACKGROUND2);
  PageFrame.setTextColor(TOLGA_WHITE);
  Icon1.createSprite(24, 24);
  Icon1.setSwapBytes(true);
  Icon1.pushImage(0, 0, 24, 24, current);
  Icon1.pushToSprite(&PageFrame, 20, 20, TFT_BLACK);
  PageFrame.setTextDatum(TL_DATUM);
  PageFrame.setTextColor(TFT_SILVER);
  PageFrame.setFreeFont(&Orbitron_Medium_12);
  PageFrame.drawString(Languages[indexLanguageArray][57] , 60, 5);
  PageFrame.setTextColor(TOLGA_WHITE);
  PageFrame.setFreeFont(&Orbitron_Bold_18);
  PageFrame.drawString(String(BMS.charge_curr2) + " A" , 60, 25);
  PageFrame.pushToSprite(&Display, 241, 2, TFT_BLACK);
  PageFrame.deleteSprite();
  Icon1.deleteSprite();

  PageFrame.createSprite(239, 50);
  PageFrame.fillSprite(TFT_BACKGROUND2);
  PageFrame.setTextColor(TOLGA_WHITE);
  Icon1.createSprite(24, 24);
  Icon1.setSwapBytes(true);
  Icon1.pushImage(0, 0, 24, 24, voltage);
  Icon1.pushToSprite(&PageFrame, 20, 20, TFT_BLACK);
  PageFrame.setTextDatum(TL_DATUM);
  PageFrame.setTextColor(TFT_SILVER);
  PageFrame.setFreeFont(&Orbitron_Medium_12);
  PageFrame.drawString(Languages[indexLanguageArray][58] , 60, 5);
  PageFrame.setTextColor(TOLGA_WHITE);
  PageFrame.setFreeFont(&Orbitron_Bold_18);
  PageFrame.drawString(String(BMS.sumv_high2 / 10) + " V" , 60, 25);
  PageFrame.pushToSprite(&Display, 0, 54, TFT_BLACK);
  PageFrame.deleteSprite();
  Icon1.deleteSprite();


  PageFrame.createSprite(239, 50);
  PageFrame.fillSprite(TFT_BACKGROUND2);
  PageFrame.setTextColor(TOLGA_WHITE);
  Icon1.createSprite(24, 24);
  Icon1.setSwapBytes(true);
  Icon1.pushImage(0, 0, 24, 24, voltage);
  Icon1.pushToSprite(&PageFrame, 20, 20, TFT_BLACK);
  PageFrame.setTextDatum(TL_DATUM);
  PageFrame.setTextColor(TFT_SILVER);
  PageFrame.setFreeFont(&Orbitron_Medium_12);
  PageFrame.drawString(Languages[indexLanguageArray][59] , 60, 5);
  PageFrame.setTextColor(TOLGA_WHITE);
  PageFrame.setFreeFont(&Orbitron_Bold_18);
  PageFrame.drawString(String(BMS.sumv_low2 / 10) + " V" , 60, 25);
  PageFrame.pushToSprite(&Display, 241, 54, TFT_BLACK);
  PageFrame.deleteSprite();
  Icon1.deleteSprite();




  PageFrame.createSprite(239, 50);
  PageFrame.fillSprite(TFT_BACKGROUND2);
  PageFrame.setTextColor(TOLGA_WHITE);
  Icon1.createSprite(24, 24);
  Icon1.setSwapBytes(true);
  Icon1.pushImage(0, 0, 24, 24, voltage);
  Icon1.pushToSprite(&PageFrame, 20, 20, TFT_BLACK);
  PageFrame.setTextDatum(TL_DATUM);
  PageFrame.setTextColor(TFT_SILVER);
  PageFrame.setFreeFont(&Orbitron_Medium_12);
  PageFrame.drawString(Languages[indexLanguageArray][60] , 60, 5);
  PageFrame.setTextColor(TOLGA_WHITE);
  PageFrame.setFreeFont(&Orbitron_Bold_18);
  PageFrame.drawString(String(BMS.cell_volthigh2 * 0.001, 3) + " V", 60, 25);
  PageFrame.pushToSprite(&Display, 0, 106, TFT_BLACK);
  PageFrame.deleteSprite();
  Icon1.deleteSprite();


  PageFrame.createSprite(239, 50);
  PageFrame.fillSprite(TFT_BACKGROUND2);
  PageFrame.setTextColor(TOLGA_WHITE);
  Icon1.createSprite(24, 24);
  Icon1.setSwapBytes(true);
  Icon1.pushImage(0, 0, 24, 24, voltage);
  Icon1.pushToSprite(&PageFrame, 20, 20, TFT_BLACK);
  PageFrame.setTextDatum(TL_DATUM);
  PageFrame.setTextColor(TFT_SILVER);
  PageFrame.setFreeFont(&Orbitron_Medium_12);
  PageFrame.drawString(Languages[indexLanguageArray][61] , 60, 5);
  PageFrame.setTextColor(TOLGA_WHITE);
  PageFrame.setFreeFont(&Orbitron_Bold_18);
  PageFrame.drawString(String(BMS.cell_voltlow2 * 0.001, 3) + " V", 60, 25);
  PageFrame.pushToSprite(&Display, 241, 106, TFT_BLACK);
  PageFrame.deleteSprite();
  Icon1.deleteSprite();




  PageFrame.createSprite(239, 50);
  PageFrame.fillSprite(TFT_BACKGROUND2);
  PageFrame.setTextColor(TOLGA_WHITE);
  Icon1.createSprite(24, 24);
  Icon1.setSwapBytes(true);
  Icon1.pushImage(0, 0, 24, 24, temp);
  Icon1.pushToSprite(&PageFrame, 20, 20, TFT_BLACK);
  PageFrame.setTextDatum(TL_DATUM);
  PageFrame.setTextColor(TFT_SILVER);
  PageFrame.setFreeFont(&Orbitron_Medium_12);
  PageFrame.drawString(Languages[indexLanguageArray][62] , 60, 5);
  PageFrame.setTextColor(TOLGA_WHITE);
  PageFrame.setFreeFont(&Orbitron_Bold_18);
  PageFrame.drawString(String(BMS.charge_temp_high2) + " C", 60, 25);
  PageFrame.pushToSprite(&Display, 0, 158, TFT_BLACK);
  PageFrame.deleteSprite();
  Icon1.deleteSprite();


  PageFrame.createSprite(239, 50);
  PageFrame.fillSprite(TFT_BACKGROUND2);
  PageFrame.setTextColor(TOLGA_WHITE);
  Icon1.createSprite(24, 24);
  Icon1.setSwapBytes(true);
  Icon1.pushImage(0, 0, 24, 24, temp);
  Icon1.pushToSprite(&PageFrame, 20, 20, TFT_BLACK);
  PageFrame.setTextDatum(TL_DATUM);
  PageFrame.setTextColor(TFT_SILVER);
  PageFrame.setFreeFont(&Orbitron_Medium_12);
  PageFrame.drawString(Languages[indexLanguageArray][63], 60, 5);
  PageFrame.setTextColor(TOLGA_WHITE);
  PageFrame.setFreeFont(&Orbitron_Bold_18);
  PageFrame.drawString(String(BMS.discharge_temp_high2) + " C", 60, 25);
  PageFrame.pushToSprite(&Display, 241, 158, TFT_BLACK);
  PageFrame.deleteSprite();
  Icon1.deleteSprite();
  Display.pushSprite(0, 60);
  Display.deleteSprite();
}

//void drawCurrentCalibration() {
//
//  Display.createSprite(480, 210);
//  Display.fillSprite(TFT_BLACK);
//
//  PageFrame.createSprite(102, 102);
//  PageFrame.fillSprite(TFT_BACKGROUND2);
// PageFrame.setTextColor(TOLGA_WHITE);
//  Icon1.createSprite(64, 64);
//  Icon1.setSwapBytes(true);
//  Icon1.pushImage(0, 0, 64, 64, upCurrent);
//  Icon1.pushToSprite(&PageFrame, 19, 19, TFT_BLACK);
//  PageFrame.pushToSprite(&Display, 0, 2, TFT_BLACK);
//
//  PageFrame.deleteSprite();
//  Icon1.deleteSprite();
//
//  PageFrame.createSprite(102, 102);
//  PageFrame.fillSprite(TFT_BACKGROUND2);
// PageFrame.setTextColor(TOLGA_WHITE);
//  Icon1.createSprite(64, 64);
//  Icon1.setSwapBytes(true);
//  Icon1.pushImage(0, 0, 64, 64, downCurrent);
//  Icon1.pushToSprite(&PageFrame, 19, 19, TFT_BLACK);
//  PageFrame.pushToSprite(&Display, 0, 106, TFT_BLACK);
//
//  PageFrame.deleteSprite();
//  Icon1.deleteSprite();
//
//
//
//  PageFrame.createSprite(272, 102);
//  PageFrame.fillSprite(TFT_BACKGROUND2);
// PageFrame.setTextColor(TOLGA_WHITE);
//
//  PageFrame.setTextDatum(TL_DATUM);
//  PageFrame.setTextColor(TFT_SILVER);
//  PageFrame.setFreeFont(&Orbitron_Bold_18);
//  PageFrame.drawString("Terminal Current" , 10, 15);
// PageFrame.setTextColor(TOLGA_WHITE);
//  PageFrame.setFreeFont(&Orbitron_Bold_28);
//  PageFrame.drawString(String((BMS.current - 30000) * 0.1) , 10, 40);
//  PageFrame.pushToSprite(&Display, 104, 2, TFT_BLACK);
//  PageFrame.deleteSprite();
//
//
//
//  PageFrame.createSprite(272, 102);
//  PageFrame.fillSprite(TFT_BACKGROUND2);
// PageFrame.setTextColor(TOLGA_WHITE);
//
//
//  PageFrame.setTextDatum(TL_DATUM);
//  PageFrame.setTextColor(TFT_SILVER);
//  PageFrame.setFreeFont(&Orbitron_Bold_18);
//  PageFrame.drawString("Calibrated Current" , 10, 15);
//  PageFrame.setTextColor(TOLGA_GREEN);
//  PageFrame.setFreeFont(&Orbitron_Bold_28);
//  PageFrame.drawString(String(CalibrationValue) , 10, 40);
//  PageFrame.pushToSprite(&Display, 104, 106, TFT_BLACK);
//  PageFrame.deleteSprite();
//  PageFrame.createSprite(102, 102);
//  PageFrame.fillSprite(TFT_BACKGROUND2);
// PageFrame.setTextColor(TOLGA_WHITE);
//  Icon1.createSprite(48, 48);
//  Icon1.setSwapBytes(true);
//  Icon1.pushImage(0, 0, 48, 48, calibrateIcon);
//  Icon1.pushToSprite(&PageFrame, 27, 14, TFT_BLACK);
//  PageFrame.setTextDatum(MC_DATUM);
//  PageFrame.setTextColor(TFT_SILVER);
//  PageFrame.setFreeFont(&Orbitron_Medium_12);
//  PageFrame.drawString("Calibrate" , 51, 75);
//  PageFrame.pushToSprite(&Display, 378, 2, TFT_BLACK);
//
//  PageFrame.deleteSprite();
//  Icon1.deleteSprite();
//
//  PageFrame.createSprite(102, 102);
//  PageFrame.fillSprite(TFT_BACKGROUND2);
// PageFrame.setTextColor(TOLGA_WHITE);
//  Icon1.createSprite(48, 48);
//  Icon1.setSwapBytes(true);
//  Icon1.pushImage(0, 0, 48, 48, calibrateIcon);
//  Icon1.pushToSprite(&PageFrame, 27, 14, TFT_BLACK);
//  PageFrame.setTextDatum(MC_DATUM);
//  PageFrame.setTextColor(TFT_SILVER);
//  PageFrame.setFreeFont(&Orbitron_Medium_12);
//  PageFrame.drawString("Zero" , 51, 73);
//  PageFrame.drawString("Calibrate" , 51, 88);
//
//  PageFrame.pushToSprite(&Display, 378, 106, TFT_BLACK);
//
//  PageFrame.deleteSprite();
//  Icon1.deleteSprite();
//
//  Display.pushSprite(0, 60);
//  Display.deleteSprite();
//}






void drawFeatures() {


  Display.createSprite(480, 210);
  Display.fillSprite(TFT_BLACK);
  PageFrame.createSprite(135, 103);
  PageFrame.fillSprite(TFT_BACKGROUND2);
  PageFrame.setTextColor(TOLGA_WHITE);
  Icon1.createSprite(24, 24);
  Icon1.setSwapBytes(true);
  Icon1.pushImage(0, 0, 24, 24, onlineMonitoring);
  Icon1.pushToSprite(&PageFrame, 55, 15, TFT_BLACK);
  PageFrame.setTextDatum(MC_DATUM);
  PageFrame.setTextColor(TFT_SILVER);
  PageFrame.setFreeFont(&Orbitron_Bold_18);
  PageFrame.drawString(Languages[indexLanguageArray][64] , 67, 55);
  PageFrame.drawString(Languages[indexLanguageArray][65] , 67, 75);
  PageFrame.pushToSprite(&Display, 36, 2, TFT_BLACK);
  PageFrame.deleteSprite();
  Icon1.deleteSprite();


  PageFrame.createSprite(135, 103);
  PageFrame.fillSprite(TFT_BACKGROUND2);
  PageFrame.setTextColor(TOLGA_WHITE);
  Icon1.createSprite(24, 24);
  Icon1.setSwapBytes(true);
  Icon1.pushImage(0, 0, 24, 24, wifidirect);
  Icon1.pushToSprite(&PageFrame, 55, 15, TFT_BLACK);
  PageFrame.setTextDatum(MC_DATUM);
  PageFrame.setTextColor(TFT_SILVER);
  PageFrame.setFreeFont(&Orbitron_Bold_18);
  PageFrame.drawString(Languages[indexLanguageArray][66] , 67, 55);
  PageFrame.drawString(Languages[indexLanguageArray][67] , 67, 75);
  PageFrame.pushToSprite(&Display, 173, 2, TFT_BLACK);
  PageFrame.deleteSprite();
  Icon1.deleteSprite();


  PageFrame.createSprite(135, 103);
  PageFrame.fillSprite(TFT_BACKGROUND2);
  PageFrame.setTextColor(TOLGA_WHITE);
  Icon1.createSprite(24, 24);
  Icon1.setSwapBytes(true);
  Icon1.pushImage(0, 0, 24, 24, bluetooth);
  Icon1.pushToSprite(&PageFrame, 55, 15, TFT_BLACK);
  PageFrame.setTextDatum(MC_DATUM);
  PageFrame.setTextColor(TFT_SILVER);
  PageFrame.setFreeFont(&Orbitron_Bold_18);
  PageFrame.drawString(Languages[indexLanguageArray][68] , 67, 55);
  PageFrame.drawString(Languages[indexLanguageArray][69] , 67, 75);
  PageFrame.pushToSprite(&Display, 310, 2, TFT_BLACK);
  PageFrame.deleteSprite();
  Icon1.deleteSprite();


  PageFrame.createSprite(135, 101);
  PageFrame.fillSprite(TFT_BACKGROUND2);
  PageFrame.setTextColor(TOLGA_WHITE);
  Icon1.createSprite(24, 24);
  Icon1.setSwapBytes(true);
  Icon1.pushImage(0, 0, 24, 24, restart);
  Icon1.pushToSprite(&PageFrame, 55, 15, TFT_BLACK);
  PageFrame.setTextDatum(MC_DATUM);
  PageFrame.setTextColor(TFT_SILVER);
  PageFrame.setFreeFont(&Orbitron_Bold_18);
  PageFrame.drawString(Languages[indexLanguageArray][70] , 67, 55);
  PageFrame.drawString(Languages[indexLanguageArray][71] , 67, 75);
  PageFrame.pushToSprite(&Display, 36, 107, TFT_BLACK);
  PageFrame.deleteSprite();
  Icon1.deleteSprite();

  PageFrame.createSprite(135, 101);
  PageFrame.fillSprite(TFT_BACKGROUND2);
  PageFrame.setTextColor(TOLGA_WHITE);
  Icon1.createSprite(24, 24);
  Icon1.setSwapBytes(true);
  Icon1.pushImage(0, 0, 24, 24, updateFirmware);
  Icon1.pushToSprite(&PageFrame, 55, 15, TFT_BLACK);
  PageFrame.setTextDatum(MC_DATUM);
  PageFrame.setTextColor(TFT_SILVER);
  PageFrame.setFreeFont(&Orbitron_Bold_18);
  PageFrame.drawString(Languages[indexLanguageArray][72] , 67, 55);
  PageFrame.drawString(Languages[indexLanguageArray][73] , 67, 75);
  PageFrame.pushToSprite(&Display, 173, 107, TFT_BLACK);
  PageFrame.deleteSprite();
  Icon1.deleteSprite();


  PageFrame.createSprite(135, 101);
  PageFrame.fillSprite(TFT_BACKGROUND2);
  PageFrame.setTextColor(TOLGA_WHITE);
  Icon1.createSprite(24, 24);
  Icon1.setSwapBytes(true);
  Icon1.pushImage(0, 0, 24, 24, manuals);
  Icon1.pushToSprite(&PageFrame, 55, 15, TFT_BLACK);
  PageFrame.setTextDatum(MC_DATUM);
  PageFrame.setTextColor(TFT_SILVER);
  PageFrame.setFreeFont(&Orbitron_Bold_18);
  PageFrame.drawString(Languages[indexLanguageArray][74] , 67, 55);
  PageFrame.drawString(Languages[indexLanguageArray][75] , 67, 75);
  PageFrame.pushToSprite(&Display, 310, 107, TFT_BLACK);
  PageFrame.deleteSprite();
  Icon1.deleteSprite();



  PageFrame.createSprite(34, 206);
  PageFrame.fillSprite(TFT_BACKGROUND2);
  Icon1.createSprite(24, 24);
  Icon1.setSwapBytes(true);
  Icon1.pushImage(0, 0, 24, 24, left);
  Icon1.pushToSprite(&PageFrame, 5, 89, TFT_BLACK);
  Icon1.deleteSprite();
  PageFrame.pushToSprite(&Display, 0, 2, TFT_BLACK);
  PageFrame.deleteSprite();
  PageFrame.createSprite(34, 206);
  PageFrame.fillSprite(TFT_BACKGROUND2);
  Icon1.createSprite(24, 24);
  Icon1.setSwapBytes(true);
  Icon1.pushImage(0, 0, 24, 24, right);
  Icon1.pushToSprite(&PageFrame, 5, 89, TFT_BLACK);
  Icon1.deleteSprite();
  PageFrame.pushToSprite(&Display, 447, 2, TFT_BLACK);
  PageFrame.deleteSprite();
  Display.pushSprite(0, 60);


}

void drawSystem() {

  Display.createSprite(480, 210);
  Display.fillSprite(TFT_BLACK);
  PageFrame.createSprite(135, 103);
  PageFrame.fillSprite(TFT_BACKGROUND2);
  PageFrame.setTextColor(TOLGA_WHITE);
  Icon1.createSprite(24, 24);
  Icon1.setSwapBytes(true);
  Icon1.pushImage(0, 0, 24, 24, dashboard);
  Icon1.pushToSprite(&PageFrame, 55, 15, TFT_BLACK);
  PageFrame.setTextDatum(MC_DATUM);
  PageFrame.setTextColor(TFT_SILVER);
  PageFrame.setFreeFont(&Orbitron_Bold_18);
  PageFrame.drawString(Languages[indexLanguageArray][77], 67, 55);
  PageFrame.drawString(Languages[indexLanguageArray][78] , 67, 75);
  PageFrame.pushToSprite(&Display, 36, 2, TFT_BLACK);
  PageFrame.deleteSprite();
  Icon1.deleteSprite();


  PageFrame.createSprite(135, 103);
  PageFrame.fillSprite(TFT_BACKGROUND2);
  PageFrame.setTextColor(TOLGA_WHITE);
  Icon1.createSprite(24, 24);
  Icon1.setSwapBytes(true);
  Icon1.pushImage(0, 0, 24, 24, alarmlog);
  Icon1.pushToSprite(&PageFrame, 55, 15, TFT_BLACK);
  PageFrame.setTextDatum(MC_DATUM);
  PageFrame.setTextColor(TFT_SILVER);
  PageFrame.setFreeFont(&Orbitron_Bold_18);
  PageFrame.drawString(Languages[indexLanguageArray][79] , 67, 55);
  PageFrame.drawString(Languages[indexLanguageArray][80] , 67, 75);
  PageFrame.pushToSprite(&Display, 173, 2, TFT_BLACK);
  PageFrame.deleteSprite();
  Icon1.deleteSprite();


  PageFrame.createSprite(135, 103);
  PageFrame.fillSprite(TFT_BACKGROUND2);
  PageFrame.setTextColor(TOLGA_WHITE);
  Icon1.createSprite(24, 24);
  Icon1.setSwapBytes(true);
  Icon1.pushImage(0, 0, 24, 24, monitoringqr);
  Icon1.pushToSprite(&PageFrame, 55, 15, TFT_BLACK);
  PageFrame.setTextDatum(MC_DATUM);
  PageFrame.setTextColor(TFT_SILVER);
  PageFrame.setFreeFont(&Orbitron_Bold_18);
  PageFrame.drawString(Languages[indexLanguageArray][81] , 67, 55);
  PageFrame.drawString(Languages[indexLanguageArray][82] , 67, 75);
  PageFrame.pushToSprite(&Display, 310, 2, TFT_BLACK);
  PageFrame.deleteSprite();
  Icon1.deleteSprite();


  PageFrame.createSprite(135, 101);
  PageFrame.fillSprite(TFT_BACKGROUND2);
  PageFrame.setTextColor(TOLGA_WHITE);
  Icon1.createSprite(24, 24);
  Icon1.setSwapBytes(true);
  Icon1.pushImage(0, 0, 24, 24, simulation);
  Icon1.pushToSprite(&PageFrame, 55, 15, TFT_BLACK);
  PageFrame.setTextDatum(MC_DATUM);
  PageFrame.setTextColor(TFT_SILVER);
  PageFrame.setFreeFont(&Orbitron_Bold_18);
  PageFrame.drawString(Languages[indexLanguageArray][158] , 67, 55);
  PageFrame.drawString(Languages[indexLanguageArray][159] , 67, 75);
  PageFrame.pushToSprite(&Display, 36, 107, TFT_BLACK);
  PageFrame.deleteSprite();
  Icon1.deleteSprite();

  PageFrame.createSprite(135, 101);
  PageFrame.fillSprite(TFT_BACKGROUND2);
  PageFrame.setTextColor(TOLGA_WHITE);
  Icon1.createSprite(24, 24);
  Icon1.setSwapBytes(true);
  Icon1.pushImage(0, 0, 24, 24, drycontact);
  Icon1.pushToSprite(&PageFrame, 55, 15, TFT_BLACK);
  PageFrame.setTextDatum(MC_DATUM);
  PageFrame.setTextColor(TFT_SILVER);
  PageFrame.setFreeFont(&Orbitron_Bold_18);
  PageFrame.drawString(Languages[indexLanguageArray][83] , 67, 55);
  PageFrame.drawString(Languages[indexLanguageArray][84] , 67, 75);
  PageFrame.pushToSprite(&Display, 173, 107, TFT_BLACK);
  PageFrame.deleteSprite();
  Icon1.deleteSprite();


  PageFrame.createSprite(135, 101);
  PageFrame.fillSprite(TFT_BACKGROUND2);
  PageFrame.setTextColor(TOLGA_WHITE);
  Icon1.createSprite(24, 24);
  Icon1.setSwapBytes(true);
  Icon1.pushImage(0, 0, 24, 24, SYSTEMSTAT);
  Icon1.pushToSprite(&PageFrame, 55, 15, TFT_BLACK);
  PageFrame.setTextDatum(MC_DATUM);
  PageFrame.setTextColor(TFT_SILVER);
  PageFrame.setFreeFont(&Orbitron_Bold_18);
  PageFrame.drawString(Languages[indexLanguageArray][85] , 67, 55);
  PageFrame.drawString(Languages[indexLanguageArray][86] , 67, 75);
  PageFrame.pushToSprite(&Display, 310, 107, TFT_BLACK);
  PageFrame.deleteSprite();
  Icon1.deleteSprite();


  PageFrame.createSprite(34, 206);
  PageFrame.fillSprite(TFT_BACKGROUND2);
  Icon1.createSprite(24, 24);
  Icon1.setSwapBytes(true);
  Icon1.pushImage(0, 0, 24, 24, left);
  Icon1.pushToSprite(&PageFrame, 5, 89, TFT_BLACK);
  Icon1.deleteSprite();
  PageFrame.pushToSprite(&Display, 0, 2, TFT_BLACK);
  PageFrame.deleteSprite();
  PageFrame.createSprite(34, 206);
  PageFrame.fillSprite(TFT_BACKGROUND2);
  Icon1.createSprite(24, 24);
  Icon1.setSwapBytes(true);
  Icon1.pushImage(0, 0, 24, 24, right);
  Icon1.pushToSprite(&PageFrame, 5, 89, TFT_BLACK);
  Icon1.deleteSprite();
  PageFrame.pushToSprite(&Display, 447, 2, TFT_BLACK);
  PageFrame.deleteSprite();
  Display.pushSprite(0, 60);



}


void drawSettings() {

  Display.createSprite(480, 210);
  Display.fillSprite(TFT_BLACK);
  PageFrame.createSprite(135, 103);
  PageFrame.fillSprite(TFT_BACKGROUND2);
  PageFrame.setTextColor(TOLGA_WHITE);
  Icon1.createSprite(24, 24);
  Icon1.setSwapBytes(true);
  Icon1.pushImage(0, 0, 24, 24, dashboard);
  Icon1.pushToSprite(&PageFrame, 55, 15, TFT_BLACK);
  PageFrame.setTextDatum(MC_DATUM);
  PageFrame.setTextColor(TFT_SILVER);
  PageFrame.setFreeFont(&Orbitron_Bold_18);
  PageFrame.drawString(Languages[indexLanguageArray][22], 67, 55);
  PageFrame.drawString(Languages[indexLanguageArray][23] , 67, 75);
  PageFrame.pushToSprite(&Display, 36, 2, TFT_BLACK);
  PageFrame.deleteSprite();
  Icon1.deleteSprite();


  PageFrame.createSprite(135, 103);
  PageFrame.fillSprite(TFT_BACKGROUND2);
  PageFrame.setTextColor(TOLGA_WHITE);
  Icon1.createSprite(24, 24);
  Icon1.setSwapBytes(true);
  Icon1.pushImage(0, 0, 24, 24, networksettings);
  Icon1.pushToSprite(&PageFrame, 55, 15, TFT_BLACK);
  PageFrame.setTextDatum(MC_DATUM);
  PageFrame.setTextColor(TFT_SILVER);
  PageFrame.setFreeFont(&Orbitron_Bold_18);
  PageFrame.drawString(Languages[indexLanguageArray][24] , 67, 55);
  PageFrame.drawString(Languages[indexLanguageArray][25] , 67, 75);
  PageFrame.pushToSprite(&Display, 173, 2, TFT_BLACK);
  PageFrame.deleteSprite();
  Icon1.deleteSprite();


  PageFrame.createSprite(135, 103);
  PageFrame.fillSprite(TFT_BACKGROUND2);
  PageFrame.setTextColor(TOLGA_WHITE);
  Icon1.createSprite(24, 24);
  Icon1.setSwapBytes(true);
  Icon1.pushImage(0, 0, 24, 24, alarms);
  Icon1.pushToSprite(&PageFrame, 55, 15, TFT_BLACK);
  PageFrame.setTextDatum(MC_DATUM);
  PageFrame.setTextColor(TFT_SILVER);
  PageFrame.setFreeFont(&Orbitron_Bold_18);
  PageFrame.drawString(Languages[indexLanguageArray][26] , 67, 55);
  PageFrame.drawString(Languages[indexLanguageArray][27] , 67, 75);
  PageFrame.pushToSprite(&Display, 310, 2, TFT_BLACK);
  PageFrame.deleteSprite();
  Icon1.deleteSprite();


  PageFrame.createSprite(135, 101);
  PageFrame.fillSprite(TFT_BACKGROUND2);
  PageFrame.setTextColor(TOLGA_WHITE);
  Icon1.createSprite(24, 24);
  Icon1.setSwapBytes(true);
  Icon1.pushImage(0, 0, 24, 24, cell);
  Icon1.pushToSprite(&PageFrame, 55, 15, TFT_BLACK);
  PageFrame.setTextDatum(MC_DATUM);
  PageFrame.setTextColor(TFT_SILVER);
  PageFrame.setFreeFont(&Orbitron_Bold_18);
  PageFrame.drawString(Languages[indexLanguageArray][28] , 67, 55);
  PageFrame.drawString(Languages[indexLanguageArray][29] , 67, 75);
  PageFrame.pushToSprite(&Display, 36, 107, TFT_BLACK);
  PageFrame.deleteSprite();
  Icon1.deleteSprite();

  PageFrame.createSprite(135, 101);
  PageFrame.fillSprite(TFT_BACKGROUND2);
  PageFrame.setTextColor(TOLGA_WHITE);
  Icon1.createSprite(24, 24);
  Icon1.setSwapBytes(true);
  Icon1.pushImage(0, 0, 24, 24, drycontact);
  Icon1.pushToSprite(&PageFrame, 55, 15, TFT_BLACK);
  PageFrame.setTextDatum(MC_DATUM);
  PageFrame.setTextColor(TFT_SILVER);
  PageFrame.setFreeFont(&Orbitron_Bold_18);
  PageFrame.drawString(Languages[indexLanguageArray][30] , 67, 55); -
  PageFrame.drawString(Languages[indexLanguageArray][31] , 67, 75);
  PageFrame.pushToSprite(&Display, 173, 107, TFT_BLACK);
  PageFrame.deleteSprite();
  Icon1.deleteSprite();


  PageFrame.createSprite(135, 101);
  PageFrame.fillSprite(TFT_BACKGROUND2);
  PageFrame.setTextColor(TOLGA_WHITE);
  Icon1.createSprite(24, 24);
  Icon1.setSwapBytes(true);
  Icon1.pushImage(0, 0, 24, 24, settings);
  Icon1.pushToSprite(&PageFrame, 55, 15, TFT_BLACK);
  PageFrame.setTextDatum(MC_DATUM);
  PageFrame.setTextColor(TFT_SILVER);
  PageFrame.setFreeFont(&Orbitron_Bold_18);
  PageFrame.drawString(Languages[indexLanguageArray][32] , 67, 55);
  PageFrame.drawString(Languages[indexLanguageArray][33] , 67, 75);
  PageFrame.pushToSprite(&Display, 310, 107, TFT_BLACK);
  PageFrame.deleteSprite();
  Icon1.deleteSprite();


  PageFrame.createSprite(34, 206);
  PageFrame.fillSprite(TFT_BACKGROUND2);
  Icon1.createSprite(24, 24);
  Icon1.setSwapBytes(true);
  Icon1.pushImage(0, 0, 24, 24, left);
  Icon1.pushToSprite(&PageFrame, 5, 89, TFT_BLACK);
  Icon1.deleteSprite();
  PageFrame.pushToSprite(&Display, 0, 2, TFT_BLACK);
  PageFrame.deleteSprite();
  PageFrame.createSprite(34, 206);
  PageFrame.fillSprite(TFT_BACKGROUND2);
  Icon1.createSprite(24, 24);
  Icon1.setSwapBytes(true);
  Icon1.pushImage(0, 0, 24, 24, right);
  Icon1.pushToSprite(&PageFrame, 5, 89, TFT_BLACK);
  Icon1.deleteSprite();
  PageFrame.pushToSprite(&Display, 447, 2, TFT_BLACK);
  PageFrame.deleteSprite();
  Display.pushSprite(0, 60);

}


//44444444
void drawMenu4() {

  Display.createSprite(480, 210);
  Display.fillSprite(TFT_BLACK);

  PageFrame.createSprite(135, 103);
  PageFrame.fillSprite(TFT_BACKGROUND2);
  PageFrame.setTextColor(TOLGA_WHITE);
  Icon1.createSprite(24, 24);
  Icon1.setSwapBytes(true);
  Icon1.pushImage(0, 0, 24, 24, inverter);
  Icon1.pushToSprite(&PageFrame, 55, 15, TFT_BLACK);
  PageFrame.setTextDatum(MC_DATUM);
  PageFrame.setTextColor(TFT_SILVER);
  PageFrame.setFreeFont(&Orbitron_Bold_18);
  PageFrame.drawString(Languages[indexLanguageArray][151], 67, 55);
  PageFrame.drawString(Languages[indexLanguageArray][152] , 67, 75);
  PageFrame.pushToSprite(&Display, 36, 2, TFT_BLACK);
  PageFrame.deleteSprite();
  Icon1.deleteSprite();


  PageFrame.createSprite(135, 103);
  PageFrame.fillSprite(TFT_BACKGROUND2);
  PageFrame.setTextColor(TOLGA_WHITE);
  Icon1.createSprite(24, 24);
  Icon1.setSwapBytes(true);
  Icon1.pushImage(0, 0, 24, 24, inverter);
  Icon1.pushToSprite(&PageFrame, 55, 15, TFT_BLACK);
  PageFrame.setTextDatum(MC_DATUM);
  PageFrame.setTextColor(TFT_SILVER);
  PageFrame.setFreeFont(&Orbitron_Bold_18);
  PageFrame.drawString(Languages[indexLanguageArray][151], 67, 55);
  PageFrame.drawString(Languages[indexLanguageArray][152] , 67, 75);
  PageFrame.pushToSprite(&Display, 173, 2, TFT_BLACK);
  PageFrame.deleteSprite();
  Icon1.deleteSprite();



  PageFrame.createSprite(135, 103);
  PageFrame.fillSprite(TFT_BACKGROUND2);
  PageFrame.setTextColor(TOLGA_WHITE);
  PageFrame.pushToSprite(&Display, 310, 2, TFT_BLACK);
  PageFrame.deleteSprite();



  PageFrame.createSprite(135, 101);
  PageFrame.fillSprite(TFT_BACKGROUND2);
  PageFrame.setTextColor(TOLGA_WHITE);
  PageFrame.pushToSprite(&Display, 36, 107, TFT_BLACK);
  PageFrame.deleteSprite();


  PageFrame.createSprite(135, 101);
  PageFrame.fillSprite(TFT_BACKGROUND2);
  PageFrame.setTextColor(TOLGA_WHITE);
  PageFrame.pushToSprite(&Display, 173, 107, TFT_BLACK);
  PageFrame.deleteSprite();



  PageFrame.createSprite(135, 101);
  PageFrame.fillSprite(TFT_BACKGROUND2);
  PageFrame.setTextColor(TOLGA_WHITE);
  PageFrame.pushToSprite(&Display, 310, 107, TFT_BLACK);
  PageFrame.deleteSprite();


  PageFrame.createSprite(34, 206);
  PageFrame.fillSprite(TFT_BACKGROUND2);
  Icon1.createSprite(24, 24);
  Icon1.setSwapBytes(true);
  Icon1.pushImage(0, 0, 24, 24, left);
  Icon1.pushToSprite(&PageFrame, 5, 89, TFT_BLACK);
  Icon1.deleteSprite();
  PageFrame.pushToSprite(&Display, 0, 2, TFT_BLACK);
  PageFrame.deleteSprite();
  PageFrame.createSprite(34, 206);
  PageFrame.fillSprite(TFT_BACKGROUND2);
  Icon1.createSprite(24, 24);
  Icon1.setSwapBytes(true);
  Icon1.pushImage(0, 0, 24, 24, right);
  Icon1.pushToSprite(&PageFrame, 5, 89, TFT_BLACK);
  Icon1.deleteSprite();
  PageFrame.pushToSprite(&Display, 447, 2, TFT_BLACK);
  PageFrame.deleteSprite();
  Display.pushSprite(0, 60);



}












void drawCellSirius() {
  float SumVoltage = 0;
  float NormSumVoltage = 0;
  float AverageVoltage = 0;
  float MaxVoltage = 0;
  float MinVoltage = 5;
  float NormAverageVoltage = 0;
  float NormMaxVoltage = 0;
  float NormMinVoltage = 200;
  float CellValues[40];
  float NormCellValues[40];


  //!!!!!!!!!!!!!!!!!!!!!!!!!!test purpose
  // ReadCellMod = 16;
  //////////////////////////////////


  if (SimMode) {
    ReadCellMod = SimCell;
  }



  if (ReadCellMod == 13) {

    for (int i = 0; i < 13; i++) {

      if (!SimMode) {
        CellValues[i] = BMS.cell_voltage[i] * 0.001;
      }
      else {
        CellValues[i] = 3.2 + random(1, 10) * 0.01;
      }

      SumVoltage = SumVoltage + CellValues[i];

      if (MaxVoltage < CellValues[i])
        MaxVoltage = CellValues[i];

      if (MinVoltage > CellValues[i])
        MinVoltage = CellValues[i];

    }
    AverageVoltage = SumVoltage / 13;




    for (int i = 0; i < 13; i++) {
      NormCellValues[i] = ((CellValues[i] - MinVoltage * 0.99) / (MaxVoltage - MinVoltage * 0.99)) * 120;
      NormSumVoltage = NormSumVoltage + NormCellValues[i];

      if (NormMaxVoltage < NormCellValues[i])
        NormMaxVoltage = NormCellValues[i];

      if (NormMinVoltage > NormCellValues[i])
        NormMinVoltage = NormCellValues[i];
    }

    NormAverageVoltage = NormSumVoltage / 13;

  }

  ///////////


  if (ReadCellMod == 4) {

    for (int i = 0; i < 4; i++) {


      if (!SimMode) {
        CellValues[i] = BMS.cell_voltage[i] * 0.001;
      }
      else {
        CellValues[i] = 3.2 + random(1, 10) * 0.01;
      }

      SumVoltage = SumVoltage + CellValues[i];

      if (MaxVoltage < CellValues[i])
        MaxVoltage = CellValues[i];

      if (MinVoltage > CellValues[i])
        MinVoltage = CellValues[i];
    }
    AverageVoltage = SumVoltage / 4;


    for (int i = 0; i < 4; i++) {
      NormCellValues[i] = ((CellValues[i] - MinVoltage * 0.99) / (MaxVoltage - MinVoltage * 0.99)) * 120;
      NormSumVoltage = NormSumVoltage + NormCellValues[i];

      if (NormMaxVoltage < NormCellValues[i])
        NormMaxVoltage = NormCellValues[i];

      if (NormMinVoltage > NormCellValues[i])
        NormMinVoltage = NormCellValues[i];
    }

    NormAverageVoltage = NormSumVoltage / 4;

  }

  ///////////////////////


  else  if (ReadCellMod == 14) {

    for (int i = 0; i < 14; i++) {

      if (!SimMode) {
        CellValues[i] = BMS.cell_voltage[i] * 0.001;
      }
      else {
        CellValues[i] = 3.2 + random(1, 10) * 0.01;
      }

      SumVoltage = SumVoltage + CellValues[i];

      if (MaxVoltage < CellValues[i])
        MaxVoltage = CellValues[i];

      if (MinVoltage > CellValues[i])
        MinVoltage = CellValues[i];

    }
    AverageVoltage = SumVoltage / 14;




    for (int i = 0; i < 14; i++) {
      NormCellValues[i] = ((CellValues[i] - MinVoltage * 0.99) / (MaxVoltage - MinVoltage * 0.99)) * 120;
      NormSumVoltage = NormSumVoltage + NormCellValues[i];

      if (NormMaxVoltage < NormCellValues[i])
        NormMaxVoltage = NormCellValues[i];

      if (NormMinVoltage > NormCellValues[i])
        NormMinVoltage = NormCellValues[i];
    }

    NormAverageVoltage = NormSumVoltage / 14;

  }




  ////////////////////


  else if (ReadCellMod == 16) {



    for (int i = 0; i < 16; i++) {

      if (!SimMode) {
        CellValues[i] = BMS.cell_voltage[i] * 0.001;
      }
      else {
        CellValues[i] = 3.2 + random(1, 10) * 0.01;
      }

      SumVoltage = SumVoltage + CellValues[i];

      // Serial.println("CellNo:" + String(ReadCellMod));
      // Serial.println("CellNo#" + String(i) + " Voltage:" + String(CellValues[i]));

      if (MaxVoltage < CellValues[i])
        MaxVoltage = CellValues[i];

      if (MinVoltage > CellValues[i])
        MinVoltage = CellValues[i];
    }

    //Serial.println("for loop is finished#1");
    AverageVoltage = SumVoltage / 16;


    for (int i = 0; i < 16; i++) {
      NormCellValues[i] = ((CellValues[i] - MinVoltage * 0.99) / (MaxVoltage - MinVoltage * 0.99)) * 50;
      NormSumVoltage = NormSumVoltage + NormCellValues[i];

      if (NormMaxVoltage < NormCellValues[i])
        NormMaxVoltage = NormCellValues[i];

      if (NormMinVoltage > NormCellValues[i])
        NormMinVoltage = NormCellValues[i];
    }

    // Serial.println("for loop is finished#2");

    NormAverageVoltage = NormSumVoltage / 16;

  }


  else if (ReadCellMod == 30) {
    for (int i = 0; i < 30; i++) {

      if (!SimMode) {
        CellValues[i] = BMS.cell_voltage[i] * 0.001;
      }
      else {
        CellValues[i] = 3.2 + random(1, 10) * 0.01;
      }



      SumVoltage = SumVoltage + CellValues[i];

      //      Serial.println("CellNo:" + String(ReadCellMod));
      //      Serial.println("CellNo#" + String(i) + " Voltage:" + String(CellValues[i]));

      if (MaxVoltage < CellValues[i])
        MaxVoltage = CellValues[i];

      if (MinVoltage > CellValues[i])
        MinVoltage = CellValues[i];
    }
    AverageVoltage = SumVoltage / 30;


    for (int i = 0; i < 30; i++) {
      NormCellValues[i] = ((CellValues[i] - MinVoltage * 0.99) / (MaxVoltage - MinVoltage * 0.99)) * 120;
      NormSumVoltage = NormSumVoltage + NormCellValues[i];

      if (NormMaxVoltage < NormCellValues[i])
        NormMaxVoltage = NormCellValues[i];

      if (NormMinVoltage > NormCellValues[i])
        NormMinVoltage = NormCellValues[i];
    }

    NormAverageVoltage = NormSumVoltage / 30;

  }


  else if (ReadCellMod == 32) {
    for (int i = 0; i < 32; i++) {


      if (!SimMode) {
        CellValues[i] = BMS.cell_voltage[i] * 0.001;
      }
      else {
        CellValues[i] = 3.2 + random(1, 10) * 0.01;
      }



      SumVoltage = SumVoltage + CellValues[i];

      //      Serial.println("CellNo:" + String(ReadCellMod));
      //      Serial.println("CellNo#" + String(i) + " Voltage:" + String(CellValues[i]));

      if (MaxVoltage < CellValues[i])
        MaxVoltage = CellValues[i];

      if (MinVoltage > CellValues[i])
        MinVoltage = CellValues[i];
    }
    AverageVoltage = SumVoltage / 32;


    for (int i = 0; i < 32; i++) {
      NormCellValues[i] = ((CellValues[i] - MinVoltage * 0.99) / (MaxVoltage - MinVoltage * 0.99)) * 120;
      NormSumVoltage = NormSumVoltage + NormCellValues[i];

      if (NormMaxVoltage < NormCellValues[i])
        NormMaxVoltage = NormCellValues[i];

      if (NormMinVoltage > NormCellValues[i])
        NormMinVoltage = NormCellValues[i];
    }

    NormAverageVoltage = NormSumVoltage / 32;

  }
















  else if (ReadCellMod == 8) {
    for (int i = 0; i < 8; i++) {

      if (!SimMode) {
        CellValues[i] = BMS.cell_voltage[i] * 0.001;
      }
      else {
        CellValues[i] = 3.2 + random(1, 10) * 0.01;
      }

      SumVoltage = SumVoltage + CellValues[i];

      if (MaxVoltage < CellValues[i])
        MaxVoltage = CellValues[i];

      if (MinVoltage > CellValues[i])
        MinVoltage = CellValues[i];
    }
    AverageVoltage = SumVoltage / 8;


    for (int i = 0; i < 8; i++) {
      NormCellValues[i] = ((CellValues[i] - MinVoltage * 0.99) / (MaxVoltage - MinVoltage * 0.99)) * 120;
      NormSumVoltage = NormSumVoltage + NormCellValues[i];

      if (NormMaxVoltage < NormCellValues[i])
        NormMaxVoltage = NormCellValues[i];

      if (NormMinVoltage > NormCellValues[i])
        NormMinVoltage = NormCellValues[i];
    }

    NormAverageVoltage = NormSumVoltage / 8;

  }



  else if (ReadCellMod == 7) {
    for (int i = 0; i < 7; i++) {


      if (!SimMode) {
        CellValues[i] = BMS.cell_voltage[i] * 0.001;
      }
      else {
        CellValues[i] = 3.2 + random(1, 10) * 0.01;
      }



      SumVoltage = SumVoltage + CellValues[i];

      if (MaxVoltage < CellValues[i])
        MaxVoltage = CellValues[i];

      if (MinVoltage > CellValues[i])
        MinVoltage = CellValues[i];
    }
    AverageVoltage = SumVoltage / 7;


    for (int i = 0; i < 7; i++) {
      NormCellValues[i] = ((CellValues[i] - MinVoltage * 0.99) / (MaxVoltage - MinVoltage * 0.99)) * 120;
      NormSumVoltage = NormSumVoltage + NormCellValues[i];

      if (NormMaxVoltage < NormCellValues[i])
        NormMaxVoltage = NormCellValues[i];

      if (NormMinVoltage > NormCellValues[i])
        NormMinVoltage = NormCellValues[i];
    }

    NormAverageVoltage = NormSumVoltage / 7;

  }



  Display.createSprite(480, 210);
  Display.fillSprite(TFT_BLACK);

  PageFrame.createSprite(360, 206);
  PageFrame.setTextDatum(TL_DATUM);
  PageFrame.setTextFont(1);
  delay(10);
  PageFrame.fillSprite(TFT_BACKGROUND2);
  //PageFrame3.setTextColor(TOLGA_WHITE);



  if (ReadCellMod != 16) {
    for (int i = 0; i < 78; i++) {
      PageFrame.drawPixel(30 + i * 4, 170 - NormMaxVoltage, TFT_RED);
      PageFrame.drawPixel(30 + i * 4, 170 - NormMinVoltage, TFT_GREEN);
      PageFrame.drawPixel(30 + i * 4, 170 - NormAverageVoltage, TFT_WHITE);
    }

    for (int i = 0; i < 5; i++) {
      PageFrame.drawPixel(330 + i * 4 , 10 , TFT_RED);
      PageFrame.drawPixel(330 + i * 4 , 20 , TFT_WHITE);
      PageFrame.drawPixel(330 + i * 4 , 30 , TFT_GREEN);
    }

    PageFrame.drawString("Max" , 300, 7);
    PageFrame.drawString("Avg." , 300, 17 );
    PageFrame.drawString("Min" , 300, 27);
  }


  if (ReadCellMod == 13) {
    PageFrame.setTextDatum(MC_DATUM);
    for (int i = 0; i < 13; i++) {
      PageFrame.fillRect(38 + i * 25, 175, 5, 5, TFT_SILVER);
      PageFrame.drawString("#" + String(i + 1) , 32 + i * 25, 188 );
    }
  }
  else if (ReadCellMod == 16) {
    PageFrame.setTextDatum(MC_DATUM);
    for (int i = 0; i < 8; i++) {
      PageFrame.fillRect(40 + i * 40, 175, 5, 5, TFT_SILVER);
      PageFrame.drawString( String(i + 1) , 43 + i * 40  , 188 );
    }
    //Serial.println("for loop is finished#3");
  }

  if (ReadCellMod == 14) {
    PageFrame.setTextDatum(MC_DATUM);
    for (int i = 0; i < 14; i++) {
      PageFrame.fillRect(34 + i * 23, 175, 5, 5, TFT_SILVER);
      PageFrame.drawString("#" + String(i + 1) , 32 + i * 24, 188 );
    }
  }


  else if (ReadCellMod == 30) {
    PageFrame.setTextDatum(MC_DATUM);
    for (int i = 0; i < 30; i++) {
      PageFrame.fillRect(38 + i * 10, 175, 5, 5, TFT_SILVER);
      //PageFrame3.drawString( String(i + 1) , 32 + i * 10, 188 );
    }
  }

  else if (ReadCellMod == 32) {
    PageFrame.setTextDatum(MC_DATUM);
    for (int i = 0; i < 32; i++) {
      PageFrame.fillRect(38 + i * 10, 175, 5, 5, TFT_SILVER);
      //PageFrame3.drawString( String(i + 1) , 32 + i * 10, 188 );
    }
  }

  else if (ReadCellMod == 8) {
    PageFrame.setTextDatum(MC_DATUM);
    for (int i = 0; i < 8; i++) {
      PageFrame.fillRect(38 + i * 40, 175, 5, 5, TFT_SILVER);
      PageFrame.drawString( String(i + 1) , 36 + i * 40  , 188 );
    }
  }

  else if (ReadCellMod == 7) {
    PageFrame.setTextDatum(MC_DATUM);
    for (int i = 0; i < 7; i++) {
      PageFrame.fillRect(38 + i * 42, 175, 5, 5, TFT_SILVER);
      PageFrame.drawString( String(i + 1) , 40 + i * 40  , 188 );
    }
  }

  else if (ReadCellMod == 4) {
    PageFrame.setTextDatum(MC_DATUM);
    for (int i = 0; i < 4; i++) {
      PageFrame.fillRect(38 + i * 80, 175, 5, 5, TFT_SILVER);
      PageFrame.drawString( String(i + 1) , 40 + i * 80  , 188 );
    }
  }


  //PageFrame3.fillRect(20, 175, 335, 2, TOLGA_WHITE);
  //PageFrame3.fillRect(20, 20, 2, 175 - 20, TOLGA_WHITE);

  if (ReadCellMod == 13) {
    for (int i = 0; i < 13; i++) {
      PageFrame.fillRect(30 + i * 25, 170 - NormCellValues[i], 20, NormCellValues[i] , TOLGA_YELLOW);
    }
  }


  else if (ReadCellMod == 16) {
    PageFrame.setTextColor(TFT_BLACK);
    for (int i = 0; i < 8; i++) {
      PageFrame.fillRect(28 + i * 40, 10, 32, 150, TOLGA_WHITE);
      PageFrame.fillRect(31 + i * 40, 85 - NormCellValues[2 * i], 26, NormCellValues[2 * i] , TOLGA_RED);
      PageFrame.drawString( String(CellValues[i * 2]) , 44 + i * 40  , 20 );


    }
    // Serial.println("for loop is finished#4");

    for (int i = 0; i < 8; i++) {

      if (!isnan(NormCellValues[2 * i + 1]) && !isnan(NormCellValues[2 * i])) {


        //Serial.println(NormCellValues[2 * i + 1]);
        PageFrame.fillRect(31 + i * 40, 95 , 26, NormCellValues[2 * i + 1] , TFT_BLUE);
        PageFrame.drawString( String(-CellValues[i * 2 + 1]) , 43 + i * 40  , 154);
        PageFrame.fillRect(28 + i * 40, 90 , 32, 3 , TFT_BACKGROUND2);
      }
    }
    //Serial.println("for loop is finished#5");








  }


  else if (ReadCellMod == 30) {
    for (int i = 0; i < 30; i++) {
      PageFrame.fillRect(30 + i * 10, 170 - NormCellValues[i], 8, NormCellValues[i] , TOLGA_YELLOW);
    }
  }

  else if (ReadCellMod == 32) {
    for (int i = 0; i < 32; i++) {
      PageFrame.fillRect(30 + i * 10, 170 - NormCellValues[i], 7, NormCellValues[i] , TOLGA_YELLOW);
    }
  }



  else if (ReadCellMod == 8) {
    for (int i = 0; i < 8; i++) {
      PageFrame.fillRect(30 + i * 40 , 170 - NormCellValues[i], 25, NormCellValues[i] , TOLGA_YELLOW);
    }
  }

  if (ReadCellMod == 14) {
    for (int i = 0; i < 14; i++) {
      PageFrame.fillRect(30 + i * 23, 170 - NormCellValues[i], 18, NormCellValues[i] , TOLGA_YELLOW);
    }
  }


  else if (ReadCellMod == 7) {
    for (int i = 0; i < 7; i++) {
      PageFrame.fillRect(30 + i * 42 , 170 - NormCellValues[i], 25, NormCellValues[i] , TOLGA_YELLOW);
    }
  }


  else if (ReadCellMod == 4) {
    for (int i = 0; i < 4; i++) {
      PageFrame.fillRect(30 + i * 80 , 170 - NormCellValues[i], 35, NormCellValues[i] , TOLGA_YELLOW);
    }
  }

  PageFrame.pushToSprite(&Display, 0, 2, TFT_BLACK);
  PageFrame.deleteSprite();

  PageFrame.createSprite(118, 206);
  PageFrame.fillSprite(TFT_BACKGROUND2);
  //PageFrame2.setTextColor(TOLGA_WHITE);t5
  //

  PageFrame.setTextDatum(TL_DATUM);
  PageFrame.setTextColor(TFT_SILVER);
  PageFrame.setFreeFont(&Orbitron_Medium_12);
  PageFrame.drawString(Languages[indexLanguageArray][47] , 25, 10);
  PageFrame.setTextColor(TFT_SILVER);
  PageFrame.setFreeFont(&Orbitron_Medium_12);
  PageFrame.drawString(Languages[indexLanguageArray][48] , 25, 70);
  // PageFrame2.setTextColor(TOLGA_WHITE);

  PageFrame.setTextColor(TFT_SILVER);
  PageFrame.setFreeFont(&Orbitron_Medium_12);
  PageFrame.drawString(Languages[indexLanguageArray][49] , 25, 130);
  // PageFrame2.setTextColor(TOLGA_WHITE);

  PageFrame.setFreeFont(&Orbitron_Bold_18);
  PageFrame.drawString(String(MaxVoltage, 2) + "V" , 35, 30);
  PageFrame.drawString(String(MinVoltage, 2) + "V" , 35, 90);
  PageFrame.drawString(String(AverageVoltage, 2) + "V" , 35, 150);

  Icon1.createSprite(24, 24);
  Icon1.setSwapBytes(true);
  Icon1.pushImage(0, 0, 24, 24, maxV);
  Icon1.pushToSprite(&PageFrame, 5, 30, TFT_BLACK);
  Icon1.deleteSprite();

  Icon1.createSprite(24, 24);
  Icon1.setSwapBytes(true);
  Icon1.pushImage(0, 0, 24, 24, minV);
  Icon1.pushToSprite(&PageFrame, 5, 90, TFT_BLACK);
  Icon1.deleteSprite();

  Icon1.createSprite(24, 24);
  Icon1.setSwapBytes(true);
  Icon1.pushImage(0, 0, 24, 24, balance);
  Icon1.pushToSprite(&PageFrame, 5, 150, TFT_BLACK);
  Icon1.deleteSprite();

  PageFrame.setTextColor(TFT_SILVER);
  PageFrame.setFreeFont(&Orbitron_Medium_12);


  PageFrame.pushToSprite(&Display, 362, 2, TFT_BLACK);
  PageFrame.deleteSprite();
  Display.pushSprite(0, 60);




}

void drawDashboard(int SOC, float MaxVolt, float MinVolt, float MaxDiff, float TerminalVolt, float TerminalCurr, float TerminalTemp) {

  Display.createSprite(480, 210);
  Display.fillSprite(TFT_BLACK);






  if (dashboard_page == 6) {
    dashboard_page = 0;
  }


  if ( dashboard_page == 0) {
    if (TerminalCurr == -3000) {
      TerminalCurr = 0;
    }
    PageFrame.createSprite(239, 160);
    PageFrame.fillSprite(TFT_BACKGROUND2);
    PageFrame.setTextColor(TFT_WHITE);
    PageFrame.setTextDatum(TL_DATUM);
    PageFrame.setTextColor(TFT_SILVER);
    PageFrame.setFreeFont(&Orbitron_Bold_18);
    PageFrame.drawString(Languages[indexLanguageArray][3] , 20, 10);
    PageFrame.setTextColor(TFT_SILVER);
    PageFrame.setFreeFont(&Orbitron_Bold_18);
    PageFrame.drawString(Languages[indexLanguageArray][4], 20, 90);
    PageFrame.setTextColor(TOLGA_WHITE);
    PageFrame.setFreeFont(&Orbitron_Medium_24);
    PageFrame.drawString(String(TerminalVolt, 2) + " V" , 60, 35);
    PageFrame.drawString(String(TerminalCurr, 2) + " A" , 60, 110);
    Icon1.createSprite(24, 24);
    Icon1.setSwapBytes(true);
    Icon1.pushImage(0, 0, 24, 24, voltage);
    Icon1.pushToSprite(&PageFrame, 15, 45, TFT_BLACK);
    Icon2.createSprite(24, 24);
    Icon2.setSwapBytes(true);
    Icon2.pushImage(0, 0, 24, 24, current);
    Icon2.pushToSprite(&PageFrame, 15, 119, TFT_BLACK);
    PageFrame.setTextColor(TFT_SILVER);
    PageFrame.pushToSprite(&Display, 0, 2, TFT_BLACK);
    PageFrame.deleteSprite();
    Icon1.deleteSprite();
    Icon2.deleteSprite();
  }

  else if ( dashboard_page == 1) {

    PageFrame.createSprite(239, 160);
    PageFrame.fillSprite(TFT_BACKGROUND2);
    PageFrame.setTextColor(TFT_WHITE);
    PageFrame.setTextDatum(TL_DATUM);
    PageFrame.setTextColor(TFT_SILVER);
    PageFrame.setFreeFont(&Orbitron_Bold_18);
    PageFrame.drawString(Languages[indexLanguageArray][5] , 20, 10);
    PageFrame.setTextColor(TFT_SILVER);
    PageFrame.setFreeFont(&Orbitron_Bold_18);
    PageFrame.drawString(Languages[indexLanguageArray][6] , 20, 90);
    PageFrame.setTextColor(TOLGA_WHITE);
    PageFrame.setFreeFont(&Orbitron_Medium_24);
    PageFrame.drawString(String(MaxVolt, 2) + " V" , 60, 35);
    PageFrame.drawString(String(MinVolt, 2) + " V" , 60, 110);
    Icon1.createSprite(24, 24);
    Icon1.setSwapBytes(true);
    Icon1.pushImage(0, 0, 24, 24, maxV);
    Icon1.pushToSprite(&PageFrame, 15, 45, TFT_BLACK);
    Icon2.createSprite(24, 24);
    Icon2.setSwapBytes(true);
    Icon2.pushImage(0, 0, 24, 24, minV);
    Icon2.pushToSprite(&PageFrame, 15, 119, TFT_BLACK);
    PageFrame.setTextColor(TFT_SILVER);
    PageFrame.setFreeFont(&Orbitron_Bold_18);

    PageFrame.pushToSprite(&Display, 0, 2, TFT_BLACK);
    PageFrame.deleteSprite();
    Icon1.deleteSprite();
    Icon2.deleteSprite();
  }


  else if ( dashboard_page == 2) {
    PageFrame.createSprite(239, 160);
    PageFrame.fillSprite(TFT_BACKGROUND2);
    PageFrame.setTextColor(TFT_WHITE);
    PageFrame.setTextDatum(TL_DATUM);
    PageFrame.setTextColor(TFT_SILVER);
    PageFrame.setFreeFont(&Orbitron_Bold_18);
    PageFrame.drawString(Languages[indexLanguageArray][7] , 20, 10);
    PageFrame.setTextColor(TFT_SILVER);
    PageFrame.setFreeFont(&Orbitron_Bold_18);


    if (CompanyMod != 7) {  //containerized
      PageFrame.drawString(Languages[indexLanguageArray][8] , 20, 90);
    }
    else {
      PageFrame.drawString("Cell Temperature" , 20, 90);
    }

    PageFrame.setTextColor(TOLGA_WHITE);
    PageFrame.setFreeFont(&Orbitron_Medium_24);
    PageFrame.drawString(String(MaxDiff, 2) + " V" , 60, 35);
    PageFrame.drawString(String(TerminalTemp, 2) + " C" , 60, 110);
    Icon1.createSprite(24, 24);
    Icon1.setSwapBytes(true);
    Icon1.pushImage(0, 0, 24, 24, balance);
    Icon1.pushToSprite(&PageFrame, 15, 45, TFT_BLACK);
    Icon2.createSprite(24, 24);
    Icon2.setSwapBytes(true);
    Icon2.pushImage(0, 0, 24, 24, temp);
    Icon2.pushToSprite(&PageFrame, 15, 119, TFT_BLACK);
    PageFrame.setTextColor(TFT_SILVER);
    PageFrame.setFreeFont(&Orbitron_Bold_18);
    PageFrame.pushToSprite(&Display, 0, 2, TFT_BLACK);
    PageFrame.deleteSprite();
    Icon1.deleteSprite();
    Icon2.deleteSprite();
  }

  else if ( dashboard_page == 3) {

    PageFrame.createSprite(239, 160);
    PageFrame.fillSprite(TFT_BACKGROUND2);
    PageFrame.setTextColor(TFT_WHITE);
    PageFrame.setTextDatum(TL_DATUM);
    PageFrame.setTextColor(TFT_SILVER);
    PageFrame.setFreeFont(&Orbitron_Bold_18);
    PageFrame.drawString(Languages[indexLanguageArray][9] , 20, 10);
    PageFrame.setTextColor(TFT_SILVER);
    PageFrame.setFreeFont(&Orbitron_Bold_18);
    PageFrame.drawString(Languages[indexLanguageArray][10] , 20, 90);
    PageFrame.setTextColor(TOLGA_WHITE);
    PageFrame.setFreeFont(&Orbitron_Medium_24);
    PageFrame.drawString(String(ChargeEnergy, 2) + " kWh" , 60, 35);
    PageFrame.drawString(String(DischargeEnergy, 2) + " kWh" , 60, 110);
    Icon1.createSprite(24, 24);
    Icon1.setSwapBytes(true);
    Icon1.pushImage(0, 0, 24, 24, charge);
    Icon1.pushToSprite(&PageFrame, 15, 45, TFT_BLACK);
    Icon2.createSprite(24, 24);
    Icon2.setSwapBytes(true);
    Icon2.pushImage(0, 0, 24, 24, discharge);
    Icon2.pushToSprite(&PageFrame, 15, 119, TFT_BLACK);
    PageFrame.setTextColor(TFT_SILVER);
    PageFrame.setFreeFont(&Orbitron_Bold_18);
    PageFrame.pushToSprite(&Display, 0, 2, TFT_BLACK);
    PageFrame.deleteSprite();
    Icon1.deleteSprite();
    Icon2.deleteSprite();
  }

  else if ( dashboard_page == 4) {

    PageFrame.createSprite(239, 160);
    PageFrame.fillSprite(TFT_BACKGROUND2);
    PageFrame.setTextColor(TFT_WHITE);
    PageFrame.setTextDatum(TL_DATUM);
    PageFrame.setTextColor(TFT_SILVER);
    PageFrame.setFreeFont(&Orbitron_Bold_18);
    PageFrame.drawString(Languages[indexLanguageArray][11] , 20, 10);
    PageFrame.setTextColor(TFT_SILVER);
    PageFrame.setFreeFont(&Orbitron_Bold_18);
    PageFrame.drawString(Languages[indexLanguageArray][12] , 20, 90);
    PageFrame.setTextColor(TOLGA_WHITE);
    PageFrame.setFreeFont(&Orbitron_Bold_18);
    PageFrame.drawString(TimeString , 60, 45);
    PageFrame.drawString(DateString , 60, 120);
    Icon1.createSprite(24, 24);
    Icon1.setSwapBytes(true);
    Icon1.pushImage(0, 0, 24, 24, date);
    Icon1.pushToSprite(&PageFrame, 15, 45, TFT_BLACK);
    Icon2.createSprite(24, 24);
    Icon2.setSwapBytes(true);
    Icon2.pushImage(0, 0, 24, 24, timeIcon);
    Icon2.pushToSprite(&PageFrame, 15, 119, TFT_BLACK);
    PageFrame.setTextColor(TFT_SILVER);
    PageFrame.setFreeFont(&Orbitron_Bold_18);
    PageFrame.pushToSprite(&Display, 0, 2, TFT_BLACK);
    PageFrame.deleteSprite();
    Icon1.deleteSprite();
    Icon2.deleteSprite();

  }


  else if ( dashboard_page == 5) {
    PageFrame.createSprite(239, 160);
    PageFrame.fillSprite(TFT_BACKGROUND2);
    PageFrame.setTextColor(TFT_WHITE);
    PageFrame.setTextDatum(TL_DATUM);
    PageFrame.setTextColor(TFT_SILVER);
    PageFrame.setFreeFont(&Orbitron_Bold_18);
    PageFrame.drawString(Languages[indexLanguageArray][13] , 20, 10);
    PageFrame.setTextColor(TFT_SILVER);
    PageFrame.setFreeFont(&Orbitron_Bold_18);
    PageFrame.drawString(Languages[indexLanguageArray][14] , 20, 90);
    PageFrame.setTextColor(TOLGA_WHITE);
    PageFrame.setFreeFont(&Orbitron_Bold_18);

    String ModeString = "";
    String AlarmString = "";

    int AlarmIndex = 0;


    for (int i = 0; i < 28; i++) {

      if (BMS.error[i]) {
        AlarmString = AlarmArray[i];
        break;
      }
      else {
        AlarmString = "No Alarm";
      }
    }

    if (WDNextRestart)
      ModeString = "Wifi-Dir.";
    else if (FBNextRestart)
      ModeString = "Online Mon.";
    else if (BTNextRestart)
      ModeString = "Bluetooth";


    PageFrame.drawString(AlarmString , 60, 45);
    PageFrame.drawString(ModeString, 60, 120);
    Icon1.createSprite(24, 24);
    Icon1.setSwapBytes(true);
    Icon1.pushImage(0, 0, 24, 24, alarms);
    Icon1.pushToSprite(&PageFrame, 15, 45, TFT_BLACK);
    Icon2.createSprite(24, 24);
    Icon2.setSwapBytes(true);
    Icon2.pushImage(0, 0, 24, 24, discharge);
    Icon2.pushToSprite(&PageFrame, 15, 119, TFT_BLACK);
    PageFrame.pushToSprite(&Display, 0, 2, TFT_BLACK);
    PageFrame.deleteSprite();
    Icon1.deleteSprite();
    Icon2.deleteSprite();
  }



  else if ( dashboard_page == 6) {

    PageFrame.createSprite(239, 160);
    PageFrame.fillSprite(TFT_BACKGROUND2);
    PageFrame.setTextColor(TFT_WHITE);
    PageFrame.setTextDatum(TL_DATUM);
    PageFrame.setTextColor(TFT_SILVER);
    PageFrame.setFreeFont(&Orbitron_Bold_18);



    if (CompanyMod == 9) {  // Omitting page 6
      dashboard_page == 7;


      float MaxTVoltage = 0;   float MinTVoltage = 60;
      int MaxTTime = 0;   int MinTTime = 0;
      float value = 0;



      PageFrame.createSprite(239, 160);
      PageFrame.fillSprite(TFT_BLACK);
      PageFrame.setTextFont(FONT2);
      PageFrame.setFreeFont(FF0);

      for (int i = 0; i < 24; i++) {




        if (VoltageArray[i] != 0 && String(BMS.year) != "2000") {
          if (MaxTVoltage < VoltageArray[i] * 0.1) {
            MaxTVoltage = VoltageArray[i] * 0.1;
            MaxTTime = i;
          }

          if (MinTVoltage > VoltageArray[i] * 0.1) {
            MinTVoltage = VoltageArray[i] * 0.1;
            MinTTime = i;
          }
        }



        PageFrame.fillRect(25 + i * 8, 96 - VoltageArray[i] * 0.1, 6, VoltageArray[i] * 0.1, TOLGA_YELLOW);
        PageFrame.setTextColor(TFT_SILVER);

        if (i == 0 || i == 6 || i == 12 || i == 18 || i == 23) {
          PageFrame.drawString(String(i), 25 + i * 8, 110);
          PageFrame.fillRect(25 + i * 8, 98, 1, 1, TFT_WHITE);
        }
      }

      PageFrame.fillRect(10, 100, 220, 1   , TOLGA_RED);
      PageFrame.fillRect(10, 30, 1, 71   , TOLGA_RED);


      PageFrame.fillRect(0, 0, 239, 20   , TFT_BACKGROUND2);

      PageFrame.fillRect(0, 140, 117, 20   , TFT_BACKGROUND2);
      PageFrame.fillRect(0, 118, 117, 20   , TFT_BACKGROUND2);

      PageFrame.fillRect(119, 140, 119, 20   , TFT_BACKGROUND2);
      PageFrame.fillRect(119, 118, 119, 20   , TFT_BACKGROUND2);


      PageFrame.setTextColor(TFT_WHITE);
      PageFrame.setTextDatum(TL_DATUM);

      PageFrame.setFreeFont(&Orbitron_Medium_12);
      PageFrame.setTextColor(TFT_SILVER);


      PageFrame.drawString(Languages[indexLanguageArray][144], 15, 3);

      //PageFrame.unloadFont();
      PageFrame.drawString(Languages[indexLanguageArray][145], 10, 121);
      PageFrame.drawString(String(MaxTVoltage, 1) + "V", 50, 121);
      PageFrame.drawString(Languages[indexLanguageArray][147], 10 , 143);
      PageFrame.drawString(String(MaxTTime), 50, 143);

      PageFrame.drawString(Languages[indexLanguageArray][146], 130, 121);
      PageFrame.drawString(String(MinTVoltage, 1) + "V", 170, 121);
      PageFrame.drawString(Languages[indexLanguageArray][147], 130, 143);
      PageFrame.drawString(String(MinTTime) , 170, 143);
      PageFrame.pushToSprite(&Display, 0, 2, TFT_BLACK);
      PageFrame.deleteSprite();


    }

    else {


      if (CompanyMod != 7) {
        PageFrame.drawString(Languages[indexLanguageArray][15] , 20, 10);
      }
      else {
        PageFrame.drawString("Terminal Temp" , 20, 10);
      }

      PageFrame.setTextColor(TFT_SILVER);
      PageFrame.setFreeFont(&Orbitron_Bold_18);

      if (CompanyMod != 7) {
        PageFrame.drawString(Languages[indexLanguageArray][16] , 20, 90);
      }
      else {
        PageFrame.drawString("Diode Temp" , 20, 90);
      }

      PageFrame.setTextColor(TOLGA_WHITE);
      PageFrame.setFreeFont(&Orbitron_Medium_24);
      PageFrame.drawString(String(CellTemp1, 2) + " C" , 60, 35);
      PageFrame.drawString(String(CellTemp2, 2) + " C" , 60, 110);
      Icon1.createSprite(24, 24);
      Icon1.setSwapBytes(true);
      Icon1.pushImage(0, 0, 24, 24, temp);
      Icon1.pushToSprite(&PageFrame, 15, 45, TFT_BLACK);
      Icon2.createSprite(24, 24);
      Icon2.setSwapBytes(true);
      Icon2.pushImage(0, 0, 24, 24, temp);
      Icon2.pushToSprite(&PageFrame, 15, 119, TFT_BLACK);
      PageFrame.setTextColor(TFT_SILVER);
      PageFrame.setFreeFont(&Orbitron_Bold_18);
      PageFrame.pushToSprite(&Display, 0, 2, TFT_BLACK);
      PageFrame.deleteSprite();
      Icon1.deleteSprite();
      Icon2.deleteSprite();
    }

  }


  else if ( dashboard_page == 7) {

    float MaxTVoltage = 0;   float MinTVoltage = 60;
    int MaxTTime = 0;   int MinTTime = 0;
    float value = 0;



    PageFrame.createSprite(239, 160);
    PageFrame.fillSprite(TFT_BLACK);
    PageFrame.setTextFont(FONT2);
    PageFrame.setFreeFont(FF0);

    for (int i = 0; i < 24; i++) {




      if (VoltageArray[i] != 0 && String(BMS.year) != "2000") {
        if (MaxTVoltage < VoltageArray[i] * 0.1) {
          MaxTVoltage = VoltageArray[i] * 0.1;
          MaxTTime = i;
        }

        if (MinTVoltage > VoltageArray[i] * 0.1) {
          MinTVoltage = VoltageArray[i] * 0.1;
          MinTTime = i;
        }
      }



      PageFrame.fillRect(25 + i * 8, 96 - VoltageArray[i] * 0.1, 6, VoltageArray[i] * 0.1, TOLGA_YELLOW);
      PageFrame.setTextColor(TFT_SILVER);

      if (i == 0 || i == 6 || i == 12 || i == 18 || i == 23) {
        PageFrame.drawString(String(i), 25 + i * 8, 110);
        PageFrame.fillRect(25 + i * 8, 98, 1, 1, TFT_WHITE);
      }
    }

    PageFrame.fillRect(10, 100, 220, 1   , TOLGA_RED);
    PageFrame.fillRect(10, 30, 1, 71   , TOLGA_RED);


    PageFrame.fillRect(0, 0, 239, 20   , TFT_BACKGROUND2);

    PageFrame.fillRect(0, 140, 117, 20   , TFT_BACKGROUND2);
    PageFrame.fillRect(0, 118, 117, 20   , TFT_BACKGROUND2);

    PageFrame.fillRect(119, 140, 119, 20   , TFT_BACKGROUND2);
    PageFrame.fillRect(119, 118, 119, 20   , TFT_BACKGROUND2);


    PageFrame.setTextColor(TFT_WHITE);
    PageFrame.setTextDatum(TL_DATUM);

    PageFrame.setFreeFont(&Orbitron_Medium_12);
    PageFrame.setTextColor(TFT_SILVER);


    PageFrame.drawString(Languages[indexLanguageArray][144], 15, 3);

    //PageFrame.unloadFont();
    PageFrame.drawString(Languages[indexLanguageArray][145], 10, 121);
    PageFrame.drawString(String(MaxTVoltage, 1) + "V", 50, 121);
    PageFrame.drawString(Languages[indexLanguageArray][147], 10 , 143);
    PageFrame.drawString(String(MaxTTime), 50, 143);

    PageFrame.drawString(Languages[indexLanguageArray][146], 130, 121);
    PageFrame.drawString(String(MinTVoltage, 1) + "V", 170, 121);
    PageFrame.drawString(Languages[indexLanguageArray][147], 130, 143);
    PageFrame.drawString(String(MinTTime) , 170, 143);
    PageFrame.pushToSprite(&Display, 0, 2, TFT_BLACK);
    PageFrame.deleteSprite();
  }

  else if ( dashboard_page == 8) {

    float MaxTTemp = 0;   float MinTTemp = 100;
    int MaxTTime = 0;   int MinTTime = 0;
    float value = 0;



    PageFrame.createSprite(239, 160);
    PageFrame.fillSprite(TFT_BLACK);
    PageFrame.setTextFont(FONT2);
    PageFrame.setFreeFont(FF0);

    for (int i = 0; i < 24; i++) {




      if (TemperatureArray[i] != 0 && String(BMS.year) != "2000") {
        if (MaxTTemp < TemperatureArray[i] * 0.1 ) {
          MaxTTemp = TemperatureArray[i] * 0.1 ;
          MaxTTime = i;
        }

        if (MinTTemp > TemperatureArray[i] * 0.1 ) {
          MinTTemp = TemperatureArray[i] * 0.1;
          MinTTime = i;
        }
      }



      PageFrame.fillRect(25 + i * 8, 96 - TemperatureArray[i] * 0.1, 6, TemperatureArray[i] * 0.1, TOLGA_YELLOW);
      PageFrame.setTextColor(TFT_SILVER);

      if (i == 0 || i == 6 || i == 12 || i == 18 || i == 23) {
        PageFrame.drawString(String(i), 25 + i * 8, 110);
        PageFrame.fillRect(25 + i * 8, 98, 1, 1, TFT_WHITE);
      }
    }

    PageFrame.fillRect(10, 100, 220, 1   , TOLGA_RED);
    PageFrame.fillRect(10, 30, 1, 71   , TOLGA_RED);


    PageFrame.fillRect(0, 0, 239, 20   , TFT_BACKGROUND2);

    PageFrame.fillRect(0, 140, 117, 20   , TFT_BACKGROUND2);
    PageFrame.fillRect(0, 118, 117, 20   , TFT_BACKGROUND2);

    PageFrame.fillRect(119, 140, 119, 20   , TFT_BACKGROUND2);
    PageFrame.fillRect(119, 118, 119, 20   , TFT_BACKGROUND2);


    PageFrame.setTextColor(TFT_WHITE);
    PageFrame.setTextDatum(TL_DATUM);

    PageFrame.setFreeFont(&Orbitron_Medium_12);
    PageFrame.setTextColor(TFT_SILVER);


    PageFrame.drawString(Languages[indexLanguageArray][150], 15, 3);

    //PageFrame.unloadFont();
    PageFrame.drawString(Languages[indexLanguageArray][145], 10, 121);
    PageFrame.drawString(String(MaxTTemp, 1) + "C", 50, 121);
    PageFrame.drawString(Languages[indexLanguageArray][147], 10 , 143);
    PageFrame.drawString(String(MaxTTime), 50, 143);

    PageFrame.drawString(Languages[indexLanguageArray][146], 130, 121);
    PageFrame.drawString(String(MinTTemp, 1) + "C", 170, 121);
    PageFrame.drawString(Languages[indexLanguageArray][147], 130, 143);
    PageFrame.drawString(String(MinTTime) , 170, 143);
    PageFrame.pushToSprite(&Display, 0, 2, TFT_BLACK);
    PageFrame.deleteSprite();



  }


  else if ( dashboard_page == 10) {

    float MaxTDischarge = 0;   float MinTDischarge = 1000000;
    int MaxTTime = 0;   int MinTTime = 0;
    float value = 0;



    PageFrame.createSprite(239, 160);
    PageFrame.fillSprite(TFT_BLACK);
    PageFrame.setTextFont(FONT2);
    PageFrame.setFreeFont(FF0);

    for (int i = 0; i < 24; i++) {




      if (HourlyDischargeArray[i] != 0 && String(BMS.year) != "2000") {
        if (MaxTDischarge < HourlyDischargeArray[i] * 10  ) {
          MaxTDischarge = HourlyDischargeArray[i] * 10  ;
          MaxTTime = i;
        }

        if (MinTDischarge > HourlyDischargeArray[i] * 10 ) {
          MinTDischarge = HourlyDischargeArray[i] * 10;
          MinTTime = i;
        }
      }



      PageFrame.fillRect(25 + i * 8, 96 - HourlyDischargeArray[i] * 0.2, 6, HourlyDischargeArray[i] * 0.2, TOLGA_YELLOW);
      PageFrame.setTextColor(TFT_SILVER);

      if (i == 0 || i == 6 || i == 12 || i == 18 || i == 23) {
        PageFrame.drawString(String(i), 25 + i * 8, 110);
        PageFrame.fillRect(25 + i * 8, 98, 1, 1, TFT_WHITE);
      }
    }

    PageFrame.fillRect(10, 100, 220, 1   , TOLGA_RED);
    PageFrame.fillRect(10, 30, 1, 71   , TOLGA_RED);


    PageFrame.fillRect(0, 0, 239, 20   , TFT_BACKGROUND2);

    PageFrame.fillRect(0, 140, 117, 20   , TFT_BACKGROUND2);
    PageFrame.fillRect(0, 118, 117, 20   , TFT_BACKGROUND2);

    PageFrame.fillRect(119, 140, 119, 20   , TFT_BACKGROUND2);
    PageFrame.fillRect(119, 118, 119, 20   , TFT_BACKGROUND2);


    PageFrame.setTextColor(TFT_WHITE);
    PageFrame.setTextDatum(TL_DATUM);

    PageFrame.setFreeFont(&Orbitron_Medium_12);
    PageFrame.setTextColor(TFT_SILVER);


    PageFrame.drawString(Languages[indexLanguageArray][149], 15, 3);

    //PageFrame.unloadFont();
    PageFrame.drawString(Languages[indexLanguageArray][145], 10, 121);
    PageFrame.drawString(String(MaxTDischarge, 1) + "W", 50, 121);
    PageFrame.drawString(Languages[indexLanguageArray][147], 10 , 143);
    PageFrame.drawString(String(MaxTTime), 50, 143);

    PageFrame.drawString(Languages[indexLanguageArray][146], 130, 121);
    PageFrame.drawString(String(MinTDischarge, 1) + "W", 170, 121);
    PageFrame.drawString(Languages[indexLanguageArray][147], 130, 143);
    PageFrame.drawString(String(MinTTime) , 170, 143);
    PageFrame.pushToSprite(&Display, 0, 2, TFT_BLACK);
    PageFrame.deleteSprite();



  }



  else if ( dashboard_page == 9) {

    float MaxTCharge = 0;   float MinTCharge = 1000000;
    int MaxTTime = 0;   int MinTTime = 0;
    float value = 0;



    PageFrame.createSprite(239, 160);
    PageFrame.fillSprite(TFT_BLACK);
    PageFrame.setTextFont(FONT2);
    PageFrame.setFreeFont(FF0);

    for (int i = 0; i < 24; i++) {




      if (HourlyChargeArray[i] != 0 && String(BMS.year) != "2000") {
        if (MaxTCharge < HourlyChargeArray[i] * 10 ) {
          MaxTCharge = HourlyChargeArray[i] * 10 ;
          MaxTTime = i;
        }

        if (MinTCharge > HourlyChargeArray[i] * 10 ) {
          MinTCharge = HourlyChargeArray[i] * 10;
          MinTTime = i;
        }
      }



      PageFrame.fillRect(25 + i * 8, 96 - HourlyChargeArray[i] * 0.2, 6, HourlyChargeArray[i] * 0.2, TOLGA_YELLOW);
      PageFrame.setTextColor(TFT_SILVER);

      if (i == 0 || i == 6 || i == 12 || i == 18 || i == 23) {
        PageFrame.drawString(String(i), 25 + i * 8, 110);
        PageFrame.fillRect(25 + i * 8, 98, 1, 1, TFT_WHITE);
      }
    }

    PageFrame.fillRect(10, 100, 220, 1   , TOLGA_RED);
    PageFrame.fillRect(10, 30, 1, 71   , TOLGA_RED);


    PageFrame.fillRect(0, 0, 239, 20   , TFT_BACKGROUND2);

    PageFrame.fillRect(0, 140, 117, 20   , TFT_BACKGROUND2);
    PageFrame.fillRect(0, 118, 117, 20   , TFT_BACKGROUND2);

    PageFrame.fillRect(119, 140, 119, 20   , TFT_BACKGROUND2);
    PageFrame.fillRect(119, 118, 119, 20   , TFT_BACKGROUND2);


    PageFrame.setTextColor(TFT_WHITE);
    PageFrame.setTextDatum(TL_DATUM);

    PageFrame.setFreeFont(&Orbitron_Medium_12);
    PageFrame.setTextColor(TFT_SILVER);


    PageFrame.drawString(Languages[indexLanguageArray][148], 15, 3);

    //PageFrame.unloadFont();
    PageFrame.drawString(Languages[indexLanguageArray][145], 10, 121);
    PageFrame.drawString(String(MaxTCharge, 1) + "W", 50, 121);
    PageFrame.drawString(Languages[indexLanguageArray][147], 10 , 143);
    PageFrame.drawString(String(MaxTTime), 50, 143);

    PageFrame.drawString(Languages[indexLanguageArray][146], 130, 121);
    PageFrame.drawString(String(MinTCharge, 1) + "W", 170, 121);
    PageFrame.drawString(Languages[indexLanguageArray][147], 130, 143);
    PageFrame.drawString(String(MinTTime) , 170, 143);
    PageFrame.pushToSprite(&Display, 0, 2, TFT_BLACK);
    PageFrame.deleteSprite();



  }







































  PageFrame.createSprite(239, 160);
  PageFrame.fillSprite(TFT_BACKGROUND2);
  PageFrame.setFreeFont(&Orbitron_Medium_24);
  //  PageFrame2.setTextColor(TOLGA_WHITE);
  PageFrame.setTextDatum(MC_DATUM);
  // ringMeter2(PageFrame2, 120, 75, 65, SOC, Languages[indexLanguageArray][17].c_str()); // Draw analogue meter
  ringMeter2(PageFrame, 120, 77, 75, SOC, Languages[indexLanguageArray][17].c_str()); // Draw analogue meter


  FooterIconLeft.createSprite(16, 16);
  if (SimulationStatus) {
    AnimationBlink = !AnimationBlink;
    if (AnimationBlink) {
      FooterIconLeft.setSwapBytes(true);
      FooterIconLeft.pushImage(0, 0, 16, 16, sim_dash);
      FooterIconLeft.setSwapBytes(false);
    }
  }
  FooterIconLeft.pushToSprite(&PageFrame, 200, 130, TFT_BLACK);
  FooterIconLeft.deleteSprite();




  PageFrame.pushToSprite(&Display, 241, 2, TFT_BLACK);
  PageFrame.deleteSprite();

  PageFrame.createSprite(239, 44);
  PageFrame.fillSprite(TFT_BACKGROUND2);
  // PageFrame3.setTextColor(TOLGA_WHITE);
  PageFrame.setTextDatum(MC_DATUM);
  PageFrame.setTextColor(TFT_SILVER);
  PageFrame.setFreeFont(&Orbitron_Bold_18);
  PageFrame.drawString(Languages[indexLanguageArray][19] , 60, 15);

  Icon2.createSprite(36, 36);
  Icon2.setSwapBytes(true);
  if (BMS.charge)
  {
    Icon2.pushImage(0, 0, 36, 36, ON);
    Icon2.pushToSprite(&PageFrame, 110, 2, TFT_BLACK);
    PageFrame.setFreeFont(&Orbitron_Bold_18);
    //  PageFrame3.setTextColor(TOLGA_WHITE);
    PageFrame.drawString(Languages[indexLanguageArray][21] , 190, 15);
  }
  else {
    Icon2.pushImage(0, 0, 36, 36, OFF);
    Icon2.pushToSprite(&PageFrame, 110, 2, TFT_BLACK);
    PageFrame.setFreeFont(&Orbitron_Bold_18);
    // PageFrame3.setTextColor(TOLGA_WHITE);
    PageFrame.drawString(Languages[indexLanguageArray][20] , 190, 15);
  }

  PageFrame.pushToSprite(&Display, 241, 164, TFT_BLACK);
  PageFrame.deleteSprite();
  Icon2.deleteSprite();

  PageFrame.createSprite(239, 44);
  PageFrame.fillSprite(TFT_BACKGROUND2);
  // PageFrame4.setTextColor(TOLGA_WHITE);
  PageFrame.setTextDatum(MC_DATUM);
  PageFrame.setTextColor(TFT_SILVER);
  PageFrame.setFreeFont(&Orbitron_Bold_18);
  PageFrame.drawString(Languages[indexLanguageArray][18] , 60, 15);
  PageFrame.setFreeFont(&Orbitron_Bold_18);
  //  PageFrame4.setTextColor(TOLGA_WHITE);

  Icon2.createSprite(36, 36);
  Icon2.setSwapBytes(true);
  if (BMS.discharge)
  {
    PageFrame.drawString(Languages[indexLanguageArray][21] , 190, 15);
    Icon2.pushImage(0, 0, 36, 36, ON);
    Icon2.pushToSprite(&PageFrame, 110, 2, TFT_BLACK);
  }
  else {
    PageFrame.drawString(Languages[indexLanguageArray][20] , 190, 15);
    Icon2.pushImage(0, 0, 36, 36, OFF);
    Icon2.pushToSprite(&PageFrame, 110, 2, TFT_BLACK);
  }

  PageFrame.pushToSprite(&Display, 0, 164, TFT_BLACK);
  PageFrame.deleteSprite();
  Icon2.deleteSprite();

  Display.pushSprite(0, 60);

}


void drawLandingPage() {

  //SETUP SCREEN------------------------------
  //  ContentFrame.createSprite(480, 80);
  //  ContentFrame.fillSprite(TFT_BLACK);
  //  ContentFrame.setTextDatum(TL_DATUM);
  //  ContentFrame.drawString(SerialNumber, 20, 50, 1);
  //  ContentFrame.drawString(FirmwareVer, 445, 50, 1);
  //  ContentFrame.drawString("SR:", 0, 80, 2);
  //  ContentFrame.pushSprite(0, 250, TFT_BLACK);


  // Serial.println("CompanyMode:" + String(CompanyMod));


  switch (CompanyMod)
  {

    case 5:  //ETISALAT

      Icon1.createSprite(245, 50);
      Icon1.setSwapBytes(true);
      Icon1.pushImage(0, 0, 225, 48, banner);
      Icon1.pushSprite(120, 5, TFT_BLACK);
      Icon1.deleteSprite();

      Icon1.createSprite(136, 50);
      Icon1.setSwapBytes(true);
      Icon1.pushImage(0, 0, 136, 50, etisalaticon);
      Icon1.pushSprite(170, 40, TFT_BLACK);
      Icon1.deleteSprite();
      break;

    case 2: // ENCAP
      Icon1.createSprite(245, 50);
      Icon1.setSwapBytes(true);
      Icon1.pushImage(0, 0, 245, 50, encaplogo);
      Icon1.pushSprite(110, 30, TFT_BLACK);
      Icon1.deleteSprite();
      break;


    case 3: // ENSIRIUS

      Icon1.createSprite(245, 50);
      Icon1.setSwapBytes(true);
      Icon1.pushImage(0, 0, 245, 50, ensiriuslogo);
      Icon1.pushSprite(110, 30, TFT_BLACK);
      Icon1.deleteSprite();
      break;

    case 4: // ENSEGA

      Icon1.createSprite(245, 50);
      Icon1.setSwapBytes(true);
      Icon1.pushImage(0, 0, 245, 50, ensegalogo);
      Icon1.pushSprite(110, 30, TFT_BLACK);
      Icon1.deleteSprite();
      break;


    case 6: // ENLIGHT

      Icon1.createSprite(245, 50);
      Icon1.setSwapBytes(true);
      Icon1.pushImage(0, 0, 245, 50, enlight);
      Icon1.pushSprite(110, 30, TFT_BLACK);
      Icon1.deleteSprite();
      break;


    case 8: // ENWALL

      Icon1.createSprite(245, 50);
      Icon1.setSwapBytes(true);
      Icon1.pushImage(0, 0, 245, 50, ENWALL);
      Icon1.pushSprite(110, 30, TFT_BLACK);
      Icon1.deleteSprite();
      break;

    default:

      Icon1.createSprite(245, 50);
      Icon1.setSwapBytes(true);
      Icon1.pushImage(0, 0, 245, 50, encaplogo);
      Icon1.pushSprite(110, 30, TFT_BLACK);
      Icon1.deleteSprite();
      break;
  }

  Icon1.createSprite(36, 36);
  Icon1.setSwapBytes(true);
  if (WDNextRestart) {
    //  Icon4.drawString("Enabled", 0, 50, 2);
    Icon1.pushImage(0, 0, 36, 36, WIFILAND2);

  }
  else {
    //  Icon4.drawString("Disabled", 0, 50, 2);
    Icon1.pushImage(0, 0, 36, 36, WIFILAND1);
  }
  Icon1.pushSprite(140, 260, TFT_BLACK);
  Icon1.deleteSprite();


  Icon1.createSprite(36, 36);
  Icon1.setSwapBytes(true);
  if (FBNextRestart) {
    //    Icon5.drawString("Enabled", 0, 50, 2);
    Icon1.pushImage(0, 0, 36, 36, CLOUDLAND2);


  }
  else {
    //  Icon5.drawString("Disabled", 0, 50, 2);
    Icon1.pushImage(0, 0, 36, 36, CLOUDLAND1);
  }
  Icon1.pushSprite(210, 260, TFT_BLACK);
  Icon1.deleteSprite();

  Icon1.createSprite(36, 36);
  Icon1.setSwapBytes(true);
  if (BTNextRestart) {
    //  Icon6.drawString("Enabled", 0, 50, 2);
    Icon1.pushImage(0, 0, 36, 36, BLUETOOTHLAND2);
  }
  else {
    //Icon6.drawString("Disabled", 0, 50, 2);
    Icon1.pushImage(0, 0, 36, 36, BLUETOOTHLAND1);
  }
  Icon1.pushSprite(280, 260, TFT_BLACK);
  Icon1.deleteSprite();






  for (int i = 0; i < frames; i++) {
    Icon1.createSprite(480, 3);
    Icon1.fillSprite(TFT_BLACK);
    Icon1.fillRect(0, 0, i * 10, 3, TOLGA_YELLOW);
    Icon1.pushSprite(0, 317);
    Icon1.deleteSprite();

    Animation.createSprite(320, 150);
    Animation.fillSprite(TFT_BLACK);
    Animation.setSwapBytes(true);

    Animation.setFreeFont(&Orbitron_Medium_12);
    Animation.setTextColor(TFT_WHITE);



    Animation.setTextDatum(MC_DATUM);
    //lala

    Animation.setTextColor(TOLGA_YELLOW);
    Animation.drawString(Languages[indexLanguageArray][141], 160, 75, 1);
    Animation.setTextColor(TFT_WHITE);
    Animation.drawString(SerialNumber, 160, 95, 1);

    Animation.setTextColor(TOLGA_YELLOW);
    Animation.drawString(Languages[indexLanguageArray][142], 160, 115, 1);
    Animation.setTextColor(TFT_WHITE);
    Animation.drawString(FirmwareVer, 160, 135, 1);



    Animation.pushImage(140, 20, animation_width  , animation_height, anim2[i]);


    Animation.pushSprite(80, 80);
    Animation.deleteSprite();
    delay(80);


  }

  ContentFrame.deleteSprite();
  Icon1.deleteSprite();
  Animation.setSwapBytes(false);

}

void drawUpload() {

  Display.createSprite(480, 210);
  Display.fillSprite(TFT_BLACK);


  Icon1.createSprite(480, 206);
  Icon1.setTextDatum(TL_DATUM);
  //  NotificationFrame.setTextColor(TOLGA_WHITE);
  Icon1.fillSprite(TFT_BACKGROUND2);
  Icon1.setFreeFont(&Orbitron_Medium_24);
  Icon1.setTextDatum(MC_DATUM);

  if (updateResponse = "Uploading") {
    Icon1.drawString("New Firmware is uploading,", 240, 50);
    Icon1.drawString("Please wait.....", 240, 90);
  }
  else if (updateResponse = "Success!") {
    Icon1.drawString("Firmware uploading,", 240, 50);
    Icon1.drawString("is successfull!", 240, 90);
  }
  else {
    Icon1.drawString("Firmware uploading,", 240, 50);
    Icon1.drawString("is failed!", 240, 90);
  }


  Icon1.setTextDatum(TL_DATUM);
  Icon1.pushToSprite(&Display, 0, 2, TFT_BLACK);
  Icon1.deleteSprite();

  Display.pushSprite(0, 60);
  Display.deleteSprite();





}


void drawBarcode() {
  Display.createSprite(480, 210);
  Display.fillSprite(TFT_BLACK);
  Icon1.createSprite(480, 206);
  Icon1.setTextDatum(MC_DATUM);
  Icon1.fillSprite(TFT_WHITE);
  Icon1.setTextColor(TFT_BLACK);
  Icon1.setFreeFont(&Orbitron_Bold_28);
  Icon1.drawString(Languages[indexLanguageArray][74], 360, 40);
  Icon1.drawString(Languages[indexLanguageArray][75], 360, 80);
  String link = "https://encap.energy/wp-content/uploads/2023/07/ENCAP-SIRIUS-Module_TDS_7.1K-48-1C-X-X-X-X-1V0_GEN1_6th-July23.pdf";
  Display_QRcode(60, 20, 3, 10, 3, link.c_str());
  Icon1.pushToSprite(&Display, 0, 2, TFT_BLACK);
  Icon1.deleteSprite();
  Display.pushSprite(0, 60);
  Display.deleteSprite();

}

void drawMonitoringBarcode() {
  Display.createSprite(480, 210);
  Display.fillSprite(TFT_BLACK);
  Icon1.createSprite(480, 206);
  Icon1.setTextDatum(MC_DATUM);
  Icon1.fillSprite(TFT_WHITE);
  Icon1.setTextColor(TFT_BLACK);
  Icon1.setFreeFont(&Orbitron_Bold_28);
  Icon1.drawString(Languages[indexLanguageArray][95], 360, 40);
  Icon1.drawString(Languages[indexLanguageArray][96], 360, 80);
  String link = "http://" + IP + "/";
  Display_QRcode(60, 20, 3, 10, 3, link.c_str());
  Icon1.pushToSprite(&Display, 0, 2, TFT_BLACK);
  Icon1.deleteSprite();
  Display.pushSprite(0, 60);
  Display.deleteSprite();

}






//SD CARD FUNCTIONS
void rm(File dir, String tempPath) {
  while (true) {
    File entry =  dir.openNextFile();
    String localPath;

    Serial.println("");
    if (entry) {
      if ( entry.isDirectory() )
      {
        localPath = tempPath + entry.name() + rootpath + '\0';
        char folderBuf[localPath.length()];
        localPath.toCharArray(folderBuf, localPath.length() );
        rm(entry, folderBuf);


        if ( SD.rmdir( folderBuf ) )
        {
          Serial.print("Deleted folder ");
          Serial.println(folderBuf);
          FolderDeleteCount++;
        }
        else
        {
          Serial.print("Unable to delete folder ");
          Serial.println(folderBuf);
          FailCount++;
        }
      }
      else
      {
        localPath = tempPath + entry.name() + '\0';
        char charBuf[localPath.length()];
        localPath.toCharArray(charBuf, localPath.length() );

        if ( SD.remove( charBuf ) )
        {
          Serial.print("Deleted ");
          Serial.println(localPath);
          DeletedCount++;
        }
        else
        {
          Serial.print("Failed to delete ");
          Serial.println(localPath);
          FailCount++;
        }
      }
    }
    else {
      // break out of recursion
      break;
    }
  }
}




//OPERATIONAL FUNCTIONS/////////////////////////////

//#########################################################################################
void Display_QRcode(int offset_x, int offset_y, int element_size, int QRsize, int ECC_Mode, const char* Message) {
  // QRcode capacity examples Size-12  65 x 65 LOW      883 535 367
  //                                           MEDIUM   691 419 287
  //                                           QUARTILE 489 296 203
  //                                           HIGH     374 227 155
  uint8_t qrcodeData[qrcode_getBufferSize(QRsize)];
  //ECC_LOW, ECC_MEDIUM, ECC_QUARTILE and ECC_HIGH. Higher levels of error correction sacrifice data capacity, but ensure damaged codes remain readable.
  if (ECC_Mode % 4 == 0) qrcode_initText(&qrcode, qrcodeData, QRsize, ECC_LOW, Message);
  if (ECC_Mode % 4 == 1) qrcode_initText(&qrcode, qrcodeData, QRsize, ECC_MEDIUM, Message);
  if (ECC_Mode % 4 == 2) qrcode_initText(&qrcode, qrcodeData, QRsize, ECC_QUARTILE, Message);
  if (ECC_Mode % 4 == 3) qrcode_initText(&qrcode, qrcodeData, QRsize, ECC_HIGH, Message);
  for (int y = 0; y < qrcode.size; y++) {
    for (int x = 0; x < qrcode.size; x++) {
      if (qrcode_getModule(&qrcode, x, y)) {
        //tft.setColor(EPD_BLACK);
        Icon1.fillRect(x * element_size + offset_x, y * element_size + offset_y, element_size, element_size, TFT_BLACK);
      }
      else
      {
        // tft.setColor(EPD_WHITE);
        Icon1.fillRect(x * element_size + offset_x, y * element_size + offset_y, element_size, element_size, TFT_WHITE);
        //  tft.fillRect(0, 0, 230, 32, TFT_RED);
      }
    }
  }
}



void firmwareUpdate(void) {

  EEPROMbusy = true;
  preferences.begin("my-app", false);
  restartCounter = 0;
  preferences.putInt("RC", restartCounter);
  preferences.end();
  EEPROMbusy = false;
  delay(100);

  updateStarted = true;
  PageNumber = 15;
  rtc_wdt_protect_off();
  rtc_wdt_disable();
  WiFiClientSecure client;
  client.setCACert(rootCACertificate);
  // httpUpdate.setLedPin(LED_BUILTIN, LOW);
  t_httpUpdate_return ret = httpUpdate.update(client, URL_fw_Bin);

  switch (ret) {
    case HTTP_UPDATE_FAILED:
      Serial.printf("HTTP_UPDATE_FAILD Error (%d): %s\n", httpUpdate.getLastError(), httpUpdate.getLastErrorString().c_str());
      updateResponse = "Failed!";
      UpdateNextRestart = false;
      break;

    case HTTP_UPDATE_NO_UPDATES:
      Serial.println("HTTP_UPDATE_NO_UPDATES");
      break;

    case HTTP_UPDATE_OK:
      Serial.println("HTTP_UPDATE_OK");
      updateResponse = "Success!";
      break;
  }
  rtc_wdt_protect_on();
  rtc_wdt_enable();
  updateStarted = false;

}
int FirmwareVersionCheck(void) {
  String payload;
  int httpCode;
  String fwurl = "";
  fwurl += URL_fw_Version;
  fwurl += "?";
  fwurl += String(rand());
  Serial.println(fwurl);
  WiFiClientSecure * client = new WiFiClientSecure;

  if (client)
  {

    client -> setCACert(rootCACertificate);

    // Add a scoping block for HTTPClient https to make sure it is destroyed before WiFiClientSecure *client is
    HTTPClient https;

    if (https.begin( * client, fwurl))
    { // HTTPS


      Serial.print("[HTTPS] GET...\n");
      // start connection and send HTTP header
      delay(100);
      httpCode = https.GET();
      delay(100);
      if (httpCode == HTTP_CODE_OK) // if version received
      {
        payload = https.getString(); // save received version
      } else {
        Serial.print("error in downloading version file:");
        Serial.println(httpCode);
      }
      https.end();


    }
    delete client;

  }

  if (httpCode == HTTP_CODE_OK) // if version received
  {
    payload.trim();
    if (payload.equals(FirmwareVer)) {
      Serial.printf("\nDevice already on latest firmware version:%s\n", FirmwareVer);


      EEPROMbusy = true;
      preferences.begin("my-app", false);
      preferences.putBool("UNR", false);
      preferences.end();
      EEPROMbusy = false;
      Serial.println("System will restart");

      for (int i = 0; i < 100; i++ )  {
        if (!deleteFilesProgress) {
          ESP.restart();
          break;
        }
        delay(10);
      }




      return 0;
    }
    else
    {
      Serial.println(payload);
      Serial.println("New firmware detected");
      return 1;
    }
  }
  return 0;
}

String ConvBinUnits(int bytes, int resolution) {
  if      (bytes < 1024)                 {
    return String(bytes) + " B";
  }
  else if (bytes < 1024 * 1024)          {
    return String((bytes / 1024.0), resolution) ;
  }
  else if (bytes < (1024 * 1024 * 1024)) {
    return String((bytes / 1024.0 / 1024.0), resolution);
  }
  else return "";
}

void HomePage() {
  Serial.println("HomePage is drawn---------------------------------------------");

  SendHTML_Header();
  navbar();
  webpage += F("<body class='hold-transition sidebar-mini'>");
  webpage += F("<div class='wrapper'>");
  Home();
  append_page_footer();

  SendHTML_Content();
  SendHTML_Stop(); // Stop is needed because no content length was sent
}
String file_size(int bytes) {
  String fsize = "";
  if (bytes < 1024)                 fsize = String(bytes) + " B";
  else if (bytes < (1024 * 1024))      fsize = String(bytes / 1024.0, 3) + " KB";
  else if (bytes < (1024 * 1024 * 1024)) fsize = String(bytes / 1024.0 / 1024.0, 3) + " MB";
  else                              fsize = String(bytes / 1024.0 / 1024.0 / 1024.0, 3) + " GB";
  return fsize;
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

void ReportCouldNotCreateFile(String target) {
  SendHTML_Header();
  webpage += F("<h3>Could Not Create Uploaded File (write-protected?)</h3>");
  webpage += F("<a href='/"); webpage += target + "'>[Back]</a><br><br>";
  append_page_footer();
  SendHTML_Content();
  SendHTML_Stop();
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

void File_Upload() {
  append_page_header();
  webpage += F("<h3>Select File to Upload</h3>");
  webpage += F("<FORM action='/fupload' method='post' enctype='multipart/form-data'>");
  webpage += F("<input class='buttons' style='width:40%' type='file' name='fupload' id = 'fupload' value=''><br>");
  webpage += F("<br><button class='buttons' style='width:10%' type='submit'>Upload File</button><br>");
  webpage += F("<a href='/'>[Back]</a><br><br>");
  append_page_footer();
  server.send(200, "text/html", webpage);
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

File UploadFile;
void handleFileUpload() { // upload a new file to the Filing system
  HTTPUpload& uploadfile = server.upload(); // See https://github.com/esp8266/Arduino/tree/master/libraries/ESP8266WebServer/srcv
  // For further information on 'status' structure, there are other reasons such as a failed transfer that could be used
  if (uploadfile.status == UPLOAD_FILE_START)
  {
    String filename = uploadfile.filename;
    if (!filename.startsWith("/")) filename = "/" + filename;
    Serial.print("Upload File Name: "); Serial.println(filename);
    SD.remove(filename);                         // Remove a previous version, otherwise data is appended the file again
    UploadFile = SD.open(filename, FILE_WRITE);  // Open the file for writing in SPIFFS (create it, if doesn't exist)
    filename = String();
  }
  else if (uploadfile.status == UPLOAD_FILE_WRITE)
  {
    if (UploadFile) UploadFile.write(uploadfile.buf, uploadfile.currentSize); // Write the received bytes to the file
  }
  else if (uploadfile.status == UPLOAD_FILE_END)
  {
    if (UploadFile)         // If the file was successfully created
    {
      UploadFile.close();   // Close the file again
      Serial.print("Upload Size: "); Serial.println(uploadfile.totalSize);
      webpage = "";
      append_page_header();
      webpage += F("<h3>File was successfully uploaded</h3>");
      webpage += F("<h2>Uploaded File Name: "); webpage += uploadfile.filename + "</h2>";
      webpage += F("<h2>File Size: "); webpage += file_size(uploadfile.totalSize) + "</h2><br>";
      append_page_footer();
      server.send(200, "text/html", webpage);
    }
    else
    {
      ReportCouldNotCreateFile("upload");
    }
  }
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

void File_Download() { // This gets called twice, the first pass selects the input, the second pass then processes the command line arguments

  if (server.args() > 0 ) { // Arguments were received
    if (server.hasArg("download"))
      SD_file_download(server.arg(0));
    Serial.print("argumnent:");
    Serial.println(server.arg(0));
    Serial.println("File Download is drawn---------------------------------------------");
  }
  else SelectInput("File Download", "Enter filename to download", "download", "download");
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

void SD_file_download(String filename) {
  if (control_sd) {
    File download = SD.open("/" + filename);
    if (download) {
      Serial.println("SD FILE DOWNLOAD is drawn---------------------------------------------");
      server.sendHeader("Content-Type", "text/text");
      server.sendHeader("Content-Disposition", "attachment; filename=" + filename);
      server.sendHeader("Connection", "close");
      server.streamFile(download, "application/octet-stream");
      download.close();
    } else ReportFileNotPresent("download");
  } else ReportSDNotPresent();
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
void SendHTML_Header() {
  server.sendHeader("Cache-vControl", "no-cache, no-store, must-revalidate");
  server.sendHeader("Pragma", "no-cache");
  server.sendHeader("Expires", "-1");
  server.setContentLength(CONTENT_LENGTH_UNKNOWN);
  server.send(200, "text/html", ""); // Empty content inhibits Content-length header so we have to close the socket ourselves.
  append_page_header();
  server.sendContent(webpage);
  webpage = "";
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
void SendHTML_Content() {
  server.sendContent(webpage);
  webpage = "";
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
void SendHTML_Stop() {
  server.sendContent("");
  server.client().stop(); // Stop is needed because no content length was sent
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
void SelectInput(String heading1, String heading2, String command, String arg_calling_name) {
  SendHTML_Header();
  navbar();
  webpage += F("<body class='hold-transition sidebar-mini'>");


  String Fname1, Fname2;


  String filename = "";
  int index = 0;
  Directory(); // Get a list of files on the FS

  webpage += " <div class='content-wrapper'>";
  webpage += "     <div class='content-header'>";
  webpage += "    </div>";
  webpage += "  <section class='content'>";
  webpage += "  <div class='container-fluid'>";

  webpage += "<table class='table table-striped table-hover'>";
  webpage += "<tr><th>Type</th><th>File Name</th><th>File Size</th><th class='sp'></th><th>Type</th><th>File Name</th><th>File Size</th></tr>";


  while (index < numfiles) {
    Serial.println("index:" + String(index));
    Serial.println("numfiles:" + String(numfiles));
    Fname1 = Filenames[index].filename;
    Fname2 = Filenames[index + 1].filename;
    webpage += "<tr>";
    webpage += "<td style = 'width:5%'>" + Filenames[index].ftype + "</td><td style = 'width:25%'>" + Fname1 + "</td><td style = 'width:10%'>" + Filenames[index].fsize + " kb</td>";
    webpage += "<td class='sp'></td>";
    if (index < numfiles - 1) {
      webpage += "<td style = 'width:5%'>" + Filenames[index + 1].ftype + "</td><td style = 'width:25%'>" + Fname2 + "</td><td style = 'width:10%'>" + Filenames[index + 1].fsize + " kb</td>";
    }
    webpage += "</tr>";
    index = index + 2;
  }
  webpage += "</table>";
  //  webpage += "  </div>";
  //  webpage += F("</div>");


  /////////////////

  //  webpage += F("<div class='content-wrapper'>");
  //  webpage += F("<div class='col-12 col-sm-6 col-md-3'>");
  webpage += F("<h3 class='rcorners_m'>");
  webpage += heading1 + "</h3><br>";
  webpage += F("<h3>");
  webpage += heading2 + "</h3>";
  webpage += F("<FORM action='/");
  webpage += command + "' method='post'>"; // Must match the calling argument e.g. '/chart' calls '/chart' after selection but with arguments!
  webpage += F("<input type='text' name='");
  webpage += arg_calling_name;
  webpage += F("' value=''><br>");
  webpage += F("<type='submit' name='");
  webpage += arg_calling_name;
  webpage += F("' value=''><br><br>");
  webpage += F("</div>");
  webpage += F("</div>");


  append_page_footer();
  SendHTML_Content();
  SendHTML_Stop();
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
void ReportSDNotPresent() {
  SendHTML_Header();
  webpage += F("<h3>No SD Card present</h3>");
  webpage += F("<a href='/'>[Back]</a><br><br>");
  append_page_footer();
  SendHTML_Content();
  SendHTML_Stop();
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
void ReportFileNotPresent(String target) {
  SendHTML_Header();
  webpage += F("<h3>File does not exist</h3>");
  webpage += F("<a href='/"); webpage += target + "'>[Back]</a><br><br>";
  append_page_footer();
  SendHTML_Content();
  SendHTML_Stop();
}
//#############################################################################################
void Directory() {
  numfiles  = 0; // Reset number of FS files counter
  File root = SD.open("/");
  if (root) {
    Serial.println("ROOT AVAILABLE");

    printDirectory(root, 0);
  }
}

//#############################################################################################
void Dir() {
  SendHTML_Header();
  navbar();
  webpage += F("<body class='hold-transition sidebar-mini'>");
  webpage += F("<div class='wrapper'>");


  String Fname1, Fname2;
  int index = 0;


  if ((OpSec < 23 && OpSec > 27)) {
    Directory();
  }// Get a list of the current files on the FS

  webpage += F("<div class='content-wrapper'>");
  webpage += "<h3>Filing System Content</h3><br>";
  if (numfiles > 0) {

    webpage += F("<div class='px-4 py-4'>");
    webpage += "<table class='table table-striped table-hover'>";
    webpage += "<tr><th>Type</th><th>File Name</th><th>File Size</th><th class='sp'></th><th>Type</th><th>File Name</th><th>File Size</th></tr>";
    while (index < numfiles) {
      Fname1 = Filenames[index].filename;
      Fname2 = Filenames[index + 1].filename;
      webpage += "<tr>";
      webpage += "<td style = 'width:5%'>" + Filenames[index].ftype + "</td><td style = 'width:25%'>" + Fname1 + "</td><td style = 'width:10%'>" + Filenames[index].fsize + "</td>";
      webpage += "<td class='sp'></td>";
      if (index < numfiles - 1) {
        webpage += "<td style = 'width:5%'>" + Filenames[index + 1].ftype + "</td><td style = 'width:25%'>" + Fname2 + "</td><td style = 'width:10%'>" + Filenames[index + 1].fsize + "</td>";
      }
      webpage += "</tr>";
      index = index + 2;
    }
    webpage += "</table>";
    webpage += F("</div>");
    webpage += "<p style='background-color:yellow;'><b>" + MessageLine + "</b></p>";
    MessageLine = "";
  }
  else
  {
    webpage += "<h2>No Files Found</h2>";
  }
  webpage += F("</div>");

  append_page_footer();
  SendHTML_Content();
  SendHTML_Stop(); // Stop is needed because no content length was sent

}

void printDirectory(File dir, int numTabs) {//1
  dir.seek(0);
  numfiles = 0;
  while (true) {

    File entry =  dir.openNextFile();

    if (! entry) {
      // no more files
      break;
    }
    for (uint8_t i = 0; i < numTabs; i++) {
      Serial.print('\t');
    }
    Serial.print(entry.name());
    if (entry.isDirectory()) {
      Serial.println("/");
      //  printDirectory(entry, numTabs + 1);//2
    } else {
      // files have sizes, directories do not
      Serial.print("\t\t");
      Serial.println(entry.size(), DEC);


      // if (String(entry.name()).indexOf("System") == -1 && String(entry.name()).indexOf("BMS Data") == -1 && String(entry.name()).indexOf("Stat") == -1) {
      Serial.println(String(entry.name()));
      Filenames[numfiles].filename = (String(entry.name()).startsWith("/") ? String(entry.name()).substring(1) : entry.name());
      Filenames[numfiles].ftype    = (entry.isDirectory() ? "Dir" : "File");
      Filenames[numfiles].fsize    = ConvBinUnits(entry.size(), 1);
      numfiles++;
      Serial.println(numfiles);
      Serial.println(numTabs);
      //  }
    }
    entry.close();
  }
}

void Home() {


  webpage += F("<div class='content-wrapper'>");
  webpage += F("<div class='content-header'></div>");
  webpage += F(     "<section class='content'>");
  webpage += F( "   <div class='container-fluid'>");

  webpage += F("     <div class='row'>");
  webpage += F("   <div class='col-12 col-sm-6 col-md-3'>");
  webpage += F(" <div class='info-box mb-3'>");
  webpage += F( "     <span class='info-box-icon bg-info elevation-1'>");
  webpage += F( "      <i class='fas fa-battery-three-quarters'></i></span>");
  webpage += F("   <div class='info-box-content'>");
  webpage += F("     <h6 class='info-box-text'> Module Voltage</h6>");
  webpage += "     <h7 class='info-box-number' id='voltage'>" + String(BMS.sum_voltage * 0.1) + "V</h7>";
  webpage += F("  </div>");
  webpage += F("</div>");
  webpage += F("  </div>");

  webpage += F(" <div class='col-12 col-sm-6 col-md-3'>");
  webpage += F("     <div class='info-box mb-3'>");
  webpage += F("        <span class='info-box-icon bg-teal elevation-1'>");
  webpage += F( "        <i class='fas fa-bolt'></i></span>");

  webpage += F("     <div class='info-box-content'>");
  webpage += F("        <h6 class='info-box-text'>Current</h6>");
  webpage += "<h7 class='info-box-number' id='current'>" + String((BMS.current - 30000) * 0.1) + "A</h7>";
  webpage += F("    </div>");
  webpage += F("     </div>");
  webpage += F("  </div>");


  webpage += F(" <div class='col-12 col-sm-6 col-md-3'>");
  webpage += F("  <div class='info-box mb-3'>");
  webpage += F("      <span class='info-box-icon bg-gray elevation-1'>");
  webpage += F("          <i class='fas fa-thermometer-half'></i></span>");
  webpage += F("      <div class='info-box-content'>");
  webpage += F("          <h6 class='info-box-text'>Module Temperature</h6>");
  webpage += "          <h7 class='info-box-number' id='temp'>" + String(BMS.max_cell_temp - 40) + "C</h7>";
  webpage += F("       </div>");
  webpage += F("      </div>");
  webpage += F("    </div>");


  webpage += F("  <div class='col-12 col-sm-6 col-md-3'>");
  webpage += F("    <div class='info-box mb-3'>");
  webpage += F("        <span class='info-box-icon bg-danger elevation-1'>");
  webpage += F("          <i class='fas fa-lightbulb'></i>");
  webpage += F("       </span>");
  webpage += F("       <div class='info-box-content'>");
  webpage += F("           <h6 class='info-box-text'>Discharge Energy</h6>");
  webpage += "           <h7 class='info-box-number' id='dischargeEnergy'>" + String(DischargeEnergy, 4) + "kWh</h7>";
  webpage += F("        </div>");
  webpage += F("     </div>");
  webpage += F("     </div>");
  webpage += F("</div>");
  //row ned



  webpage += F("   <div class='row'>");
  webpage += F("  <div class='col-12 col-sm-6 col-md-3'>");
  webpage += F("     <div class='info-box mb-3'>");
  webpage += F("       <span class='info-box-icon bg-indigo elevation-1'>");
  webpage += F("           <i class='fa fa-tachometer' aria-hidden='true'></i></span>");
  webpage += F("       <div class='info-box-content'>");
  webpage += F("           <h6 class='info-box-text'> Module SOC</h6>");
  webpage += "           <h7 class='info-box-number' id='voltage'>" + String(BMS.SOC * 0.1) + "%</h7>";
  webpage += F("       </div>");
  webpage += F("   </div>");
  webpage += F("  </div>");

  webpage += F("  <div class='col-12 col-sm-6 col-md-3'>");
  webpage += F("     <div class='info-box mb-3'>");
  webpage += F("       <span class='info-box-icon bg-maroon elevation-1'>");
  webpage += F("           <i class='fa fa-plus-square-o' aria-hidden='true'></i></span>");
  webpage += F("       <div class='info-box-content'>");
  webpage += F("           <h6 class='info-box-text'> Maximum Cell</h6>");
  webpage += "           <h7 class='info-box-number' id='voltage'>" + String(BMS.max_cell_volt * 0.001) + "V</h7>";
  webpage += F("       </div>");
  webpage += F("   </div>");
  webpage += F("  </div>");


  webpage += F("  <div class='col-12 col-sm-6 col-md-3'>");
  webpage += F("     <div class='info-box mb-3'>");
  webpage += F("       <span class='info-box-icon bg-olive elevation-1'>");
  webpage += F("           <i class='fa fa-minus-square-o' aria-hidden='true'></i></span>");
  webpage += F("       <div class='info-box-content'>");
  webpage += F("           <h6 class='info-box-text'> Minimum Cell</h6>");
  webpage += "           <h7 class='info-box-number' id='voltage'>" + String(BMS.min_cell_volt * 0.001) + "V</h7>";
  webpage += F("       </div>");
  webpage += F("   </div>");
  webpage += F("  </div>");

  webpage += F("  <div class='col-12 col-sm-6 col-md-3'>");
  webpage += F("     <div class='info-box mb-3'>");
  webpage += F("       <span class='info-box-icon bg-cyan elevation-1'>");
  webpage += F("           <i class='fa-solid fa-diagram-next'></i></span>");

  webpage += F("       <div class='info-box-content'>");
  webpage += F("           <h6 class='info-box-text'> Difference Cell</h6>");
  webpage += "           <h7 class='info-box-number' id='voltage'>" + String(BMS.max_cell_volt * 0.001 - BMS.min_cell_volt * 0.001, 3) + "V</h7>";
  webpage += F("       </div>");
  webpage += F("   </div>");
  webpage += F("  </div>");
  webpage += F("</div>");
  //row e

  webpage += F("</div>");
  webpage += F(" </section>");
  webpage += F(" </div>");
  webpage += F(" </div>");
}

void navbar() {
  webpage += F("<nav class='main-header navbar navbar-expand navbar-light'>");
  webpage += F("<ul class='navbar-nav'>");
  webpage += F("<li class='nav-item'>");
  webpage += F("<a class='nav-link' data-widget='pushmenu' href='#' role='button'><i class='fas fa-bars'></i></a>");
  webpage += F("</li>");


  webpage += F("<li class='nav-item'>");
  webpage += F("<a class='nav-link' data-widget='fullscreen' href='#' role='button'>");
  webpage += F("<i class='fas fa-expand-arrows-alt'></i>");
  webpage += F("</a>");
  webpage += F("</li>");

  //  webpage += F("  <li class='nav-item d-none d-sm-inline-block'>");
  //  webpage += F("   <a href='#' class='nav-link'>Contact</a>");
  //  webpage += F("   </li>");

  webpage += F("</ul>");

  webpage += F(" <ul class='navbar-nav ml-auto'>");
  webpage += F("   <div class='container'>");
  webpage += F("       <i class='fa-solid fa-barcode'></i>");
  webpage += "        <h7 class='nav-link' id='site_name'>" + SerialNumber + "</h7>";
  webpage += F("     </div>");
  webpage += F("     </ul>");

  webpage += F("</nav>");
  webpage += F("<aside class='main-sidebar sidebar-dark-primary elevation-4'>");
  webpage += F("<div class='brand-link'>");
  webpage += F("<img src='https://kilowattlabs.com/wp-content/themes/KiloWatt-Labs/Technical_Data_Sheets/Low/en.png' alt='' class='brand-image elevation' style='opacity: .8'>");
  webpage += F("<span class='brand-text '> ENCONNECT </span>");
  webpage += F("</div>");
  webpage += F("<div class='sidebar'>");
  webpage += F("<nav class='mt-2'>");
  webpage += F("<ul class='nav nav-pills nav-sidebar flex-column' data-widget='treeview' role='menu'");
  webpage += F("data-accordion='false' id='bars'>");


  webpage += F("<li class='nav-item'>");
  webpage += F("<a href=' / ' class='nav-link '>");
  webpage += F("<i class='nav-icon fas fa-tachometer-alt'></i>");
  webpage += F("<p>");
  webpage += F("Dashboard");
  webpage += F("</p>");
  webpage += F("</a>");
  webpage += F("</li>");


  webpage += F("<li class='nav-item'>");
  webpage += F("<a href='/download' class='nav-link'>");
  webpage += F("<i class='nav-icon fa-solid fa-download'></i>");
  webpage += F("<p>");
  webpage += F("Download");
  webpage += F("</p>");
  webpage += F("</a>");
  webpage += F("</li>");

  webpage += F("<li class='nav-item'>");
  webpage += F("<a href='/upload' class='nav-link'>");
  webpage += F("<i class='nav-icon fa-solid fa-upload'></i>");
  webpage += F("<p>");
  webpage += F("Upload");
  webpage += F("</p>");
  webpage += F("</a>");
  webpage += F("</li>");


  webpage += F("<li class='nav-item'>");
  webpage += F("<a href='#' class='nav-link '>");
  webpage += F("<i class='nav-icon fa-regular fa-clock'></i>");
  webpage += F("<p>");
  webpage += "Time: " + TimeString;
  webpage += F("</p>");
  webpage += F("</a>");
  webpage += F("</li>");


  webpage += F("<li class='nav-item'>");
  webpage += F("<a href='#' class='nav-link'>");
  webpage += F("<i class='nav-icon fa-solid fa-calendar-days'></i>");
  webpage += F("<p>");
  webpage += "Date: " + DateString;
  webpage += F("</p>");
  webpage += F("</a>");
  webpage += F("</li>");







  webpage += F(" </nav>");
  webpage += F(" </div>");
  webpage += F("</aside>");
}



void ringMeter2(TFT_eSprite & tft, int x, int y, int r, int val, const char *units)
{
  static uint16_t last_angle = 30;
  last_angle = 30;
  tft.fillCircle(x, y, r, TFT_BACKGROUND2);
  tft.drawSmoothCircle(x, y, r, TFT_BLACK, TFT_BLACK);
  uint16_t tmp = r - 3;
  tft.drawArc(x, y, tmp, tmp - tmp / 5, last_angle, 330, DARKER_GREY, DARKER_GREY);



  tft.setFreeFont(&Orbitron_Bold_18);
  tft.setTextColor(TOLGA_WHITE);
  tft.drawString(String(RemainingTime) , x, y - 35);

  tft.setTextColor(TFT_SILVER);
  tft.drawString(Languages[indexLanguageArray][143] , x , y - 15);



  tft.setFreeFont(&Orbitron_Bold_18);
  tft.setTextColor(TOLGA_WHITE);
  tft.drawString(String(BMS.rem_cap * 0.001 * 0.001 * NominalVoltage) , x, y + 13);

  tft.setTextColor(TFT_SILVER);
  tft.setFreeFont(&Orbitron_Medium_12);
  tft.drawString("kWh" , x , y + 33);

  tft.setFreeFont(&Orbitron_Bold_16);
  tft.setTextColor(TOLGA_WHITE);
  tft.drawString(String(val) , x - 7, y + 55);

  tft.setFreeFont(&Orbitron_Medium_12);
  tft.setTextColor(TFT_SILVER);
  tft.drawString("%" , x + 20, y + 55);

  //1.0.8 update
  if ((BMS.current - 30000) * 0.1 < -5) {
    RemainingTime = (BMS.rem_cap * 0.001) / abs((BMS.current - 30000) * 0.1 ) * 60;
  }

  else if ((BMS.current - 30000) * 0.1 > 5) {
    RemainingTime = ((BMS.rated_cap - BMS.rem_cap) * 0.001 ) / abs((BMS.current - 30000) * 0.1 )  * 60;
  }
  else {
    RemainingTime = 0;
  }


  r -= 3;

  // Range here is 0-100 so value is scaled to an angle 30-330
  int val_angle = map(val, 0, 100, 30, 330);

  if (last_angle != val_angle) {
    uint8_t thickness = r / 8;
    if ( r < 25 ) thickness = r / 3;
    // Update the arc, only the zone between last_angle and new val_angle is updated
    if (val_angle > last_angle) {
      tft.drawArc(x, y, r, r - thickness, last_angle, val_angle, TOLGA_YELLOW, DARKER_GREY); // TFT_SKYBLUE random(0x10000)
    }
    else {
      tft.drawArc(x, y, r, r - thickness, val_angle, last_angle, DARKER_GREY, DARKER_GREY);
    }
    last_angle = val_angle; // Store meter arc position for next redraw
  }
}






void buzzer_function() {
  //buzer
  buzzerActive = true;
}


void getLocation()
{

  Serial.println("get location started");
  HTTPClient http;
  String payload = "";
  DynamicJsonBuffer  doc(1048);
  if ((WiFi.status() == WL_CONNECTED)) { //Check the current connection status
    Serial.println("wifi started");
    http.begin("https://ipapi.co/json/"); //Specify the URL
    int httpCode = http.GET();                                        //Make the request
    if (httpCode > 0) {
      Serial.println("http started");
      payload = http.getString();

      JsonObject& root = doc.parseObject(payload);

      if (root.success()) {
        const char* country = root["country_name"];
        const char* utc_offset = root["utc_offset"];
        Latitude = root["latitude"];
        Longitude = root["longitude"];
        CountryName = String(country);


        String directionTime = String(utc_offset).substring(0, 1);
        String offsetHour = String(utc_offset).substring(2, 3);
        String offsetHourPrecision = String(utc_offset).substring(3, 5);
        String TotalOffset = directionTime + offsetHour + "." + offsetHourPrecision;
        double ConvertedOffset = TotalOffset.toDouble();


        Serial.println("--------GPS-------");
        Serial.print("Offset:");
        Serial.println(ConvertedOffset, 2);
        Serial.print("Country:");
        Serial.println(CountryName);
        Serial.print("Latitude:");
        Serial.println(Latitude, 4);
        Serial.print("Longitude:");
        Serial.println(Longitude, 4);
        Serial.println("--------GPS-------");

        //timeoffset = ConvertedOffset * 3600;


        EEPROMbusy = true;
        preferences.begin("my-app", false);
        delay(10);
        preferences.putInt("timeoffset", timeoffset);
        delay(10);
        preferences.end();
        EEPROMbusy = false;






        GPSFetched = true;
      }
      else {
        Serial.println("get location failed");
      }
    }
    http.end(); //Free the resources
  }
}

void createDirectory() {

  createDir(SD, "/Stat");
  createDir(SD, "/Datalogger");
}

void clearSD()
{
  byte sd = 0;
  digitalWrite(SD_CS_PIN, LOW);
  while (sd != 255)
  {
    sd = SPI.transfer(255);
    Serial.print("clear sd result=");
    Serial.println(sd);
  }
  digitalWrite(SD_CS_PIN, HIGH);
}




//String comm_type[] = { "NONE","PYLON","GROWATT","SOFAR","VOLTRONIC","GOODWE","SRNE","MUST","VICTRON","SMA","DYE","AISWEI","SOCALAR","SOLARK","XMT","R2","R3" };
//String comm_method[] = { "RS485","CAN" };
