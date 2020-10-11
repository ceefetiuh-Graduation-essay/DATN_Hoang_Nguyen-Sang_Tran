//----------------------------------------------------------------------THU VIEN--------------------------------------------------------------------//
//----THU VIEN KET NOI WIFI BANG HTTP----//
#include <ESP8266WiFi.h>
#include <WiFiClient.h> 
#include <ESP8266WebServer.h>
#include <ESP8266HTTPClient.h>
//---WiFiManager---//
#include <DNSServer.h> 
#include <WiFiManager.h> 
//---WiFiManager---//
//----THU VIEN KET NOI WIFI BANG HTTP----//


//----THU VIEN CHO NRF24L01----//
#include <SPI.h>                                            // khai bao file thu vien trong duong cai dat
#include <RF24.h> //khai bao file thu vien trong project
#include <nRF24L01.h>                                      // khai bao file thu vien define thanh ghi NRF trong cai dat
//#include <printf.h> //khai bao file thu vien trong project

//----THU VIEN CHO NRF24L01----//

//--thu vien EEPROM--//
#include <EEPROM.h>
//--thu vien EEPROM--//

//--thu vien xu ly chuoi--//
#include <string.h>
//--thu vien xu ly chuoi--//



//--thu vien lcd i2c--//
#include <LiquidCrystal_I2C.h>

#include <Wire.h>

LiquidCrystal_I2C lcd(0x27, 20, 4);
//--thu vien lcd i2c--//
//----------------------------------------------------------------------THU VIEN--------------------------------------------------------------------//




//----------------------------------------------------------------------BIEN--------------------------------------------------------------------//
//-------KHAI BAO BIEN DUNG TRONG CHE DO WIFI-------------//
const char *ssid = "DreamLand"; // MK WIFI
const char *password ="emlaymaymedungxaiwifichuanhaem69212169"; //Pass WIFI
//const char *ssid = "Industrial Automation"; // MK WIFI
//const char *password ="*tdh@IUH.vn#"; //Pass WIFI
//const char *host = "192.168.43.106";  // Dia chi IP may(dien thoai)
//const char *host = "192.168.1.2";//Dia chi IP may
//const char *host = "https://smartdataloggervn.000webhostapp.com";  

char payload[32], payload2[32];//bien load min, max tu server
char LoadLcdMin[12];//bien cat chuoi load chi de lai min 
char LoadLcdMax[12];//bien cat chuoi load chi de lai max 
char so_sanh_1[6];
char so_sanh_2[6];
char C1[6],H1[6],S1[4];
//-------KHAI BAO BIEN DUNG TRONG CHE DO WIFI-------------//

//-------KHAI BAO BIEN DUNG TRONG NRF24L01-------------//
RF24 radio(2, 15);  // khai bao chan(CE, CSN) la GPIO2(D4), GPIO15(D8)
int led_test = 5;
int led_rx = 16;  
int led_tx = 16;
int a=0;
int b=0;
int c=0;
int next_1=0;
int next_2=0;
int slave[10];
unsigned char  ID1a[5]= {0x11,0x11,0x11,0x11,0x01};   //5 bytes dia chi truyen/nhan-----------const dung de bien kieu bien nay thanh hang so khong thay doi duoc
unsigned char  ID1b[5]= {0x11,0x11,0x11,0x11,0x03};
unsigned char  ID2a[5]= {0x11,0x11,0x11,0x11,0x02};   //5 bytes dia chi truyen/nhan
unsigned char  ID2b[5]= {0x11,0x11,0x11,0x11,0x04};   //5 bytes dia chi truyen/nhan
unsigned char  ID3a[5]= {0x11,0x11,0x11,0x11,0x01};   //5 bytes dia chi truyen/nhan-----------const dung de bien kieu bien nay thanh hang so khong thay doi duoc
unsigned char  ID3b[5]= {0x11,0x11,0x11,0x11,0x03};
char data[32];//du lieu nhan
char data_b[32];//du lieu nhan Slave b
char kiem_data[32];//dung kiem tra motor
//char check_1a[32] = "yeu cau slave 1a ";     
//char check_2a[32] = "yeu cau slave 2a ";   
char check_ok[32];  
char ok[32] = "ok\r\n";  
char id_1[100]="id=1&";//du lieu truyen len server 1
char id_2[100]="id=2&";//du lieu truyen len server 2
static unsigned long thoi_gian_cho = 0;//dat bien dem thoi gian cho nhan duoc giu lieu
static unsigned long thoi_gian_check = 0;
static unsigned long thoi_gian_gui_lenh = 0;//dung cho Slave b
//int count=0,count2=0;
int devide=1;
//-------KHAI BAO BIEN DUNG TRONG NRF24L01-------------//
//----------------------------------------------------------------------BIEN--------------------------------------------------------------------//


//---------------------HÀM L?Y SSID, PASS WIFI---------------------//
void configModeCallback (WiFiManager *myWiFiManager)
{
  lcd.setCursor(0, 1);
  lcd.print("=> KET NOI THAT BAI");
  lcd.setCursor(0, 2);
  lcd.print("*KET NOI ESP1348035*");
  lcd.setCursor(0, 3);
  lcd.print("*DE NHAP SSID, PASS*");
  Serial.println("**----VÀO CH? Ð? NH?P SSID VÀ M?T KH?U CHO WIFI M?I----**");
  Serial.println(WiFi.softAPIP());
  Serial.println(myWiFiManager->getConfigPortalSSID());
}
//---------------------HÀM L?Y SSID, PASS WIFI---------------------//


//------------------------------------------------------------------------------------------SETUP---------------------------------------------------------------------//
void setup()
{
  
  pinMode(led_test,OUTPUT);
  pinMode(led_tx,OUTPUT);
  pinMode(led_rx,OUTPUT);
  delay(1000);
  Serial.begin(115200);

  //--------------LCD----------------//
  Wire.begin(4, 5);   //Use predefined PINS consts D2,D1
  lcd.begin(20,4);      // The begin call takes the width and height. This
                      // Should match the number provided to the constructor.
  lcd.backlight();      // Turn on the backlight.

  lcd.home();
  lcd.clear();
  //--------------LCD----------------//
  
  //----------------------------------L?Y SSID VÀ PASS WIFI T? TRANG WED LOCAL DO NODEMCU T?O RA--------------------------------------//
  //Khai báo wifiManager thu?c class WiFiManager, du?c quy d?nh trong file WiFiManager.h
  WiFiManager wifiManager;
  //có th? reset các cài d?t cu b?ng cách g?i hàm:
  //wifiManager.resetSettings();
  //--------------LCD----------------//
  lcd.setCursor(0, 0);  // Move the cursor at origin
  lcd.print("DANG KET NOI WIFI...");
  //--------------LCD----------------//
  //Cài d?t callback, khi k?t n?i v?i wifi cu th?t b?i, thi?t b? s? g?i hàm callback
  //và kh?i d?ng ch? d? AP v?i SSID du?c cài t? d?ng là "ESP+chipID"
  wifiManager.setAPCallback(configModeCallback);
  if (!wifiManager.autoConnect())
  {
//    //--------------LCD----------------//
//    lcd.clear();
//    lcd.setCursor(0, 0);  // Move the cursor at origin
//    lcd.print("DA LUU SSID, PASS");
//    lcd.setCursor(0, 1);
//    lcd.print("=> HAY RESET THIET");
//    lcd.setCursor(0, 2);
//    lcd.print("BI DE KET NOI LAI");
//    //--------------LCD----------------//
    Serial.println("K?T N?I TH?T B?I, HÃY RESET NODEMCU Ð? K?T N?I L?I");
    //N?u k?t n?i th?t b?i, th? k?t n?i l?i b?ng cách reset thi?t b?
    ESP.reset();
    delay(1000);
  }
  //N?u k?t n?i wifi thành công, in thông báo ra màn hình
  Serial.println("-----------------------ÐÃ K?T N?I THÀNH CÔNG WIFI---------------------------)");
  //--------------LCD----------------//
  lcd.clear();
  lcd.setCursor(0, 0);  // Move the cursor at origin
  lcd.print("DANG KET NOI WIFI...");
  lcd.setCursor(0, 1);
  lcd.print("=> DA KET NOI");
  //--------------LCD----------------//
  //------------------------------L?Y SSID VÀ PASS WIFI T? TRANG WED LOCAL DO NODEMCU T?O RA------------------------------------------//

  delay(3000);

  // Ket noi duoc neu thanh cong
  Serial.println("");
  Serial.print("Connected to ");
  Serial.println(wifiManager.getSSID());
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP()); 
  //--------------LCD----------------//
  lcd.clear();
  lcd.setCursor(0, 0);  // Move the cursor at origin
  lcd.print("KET NOI WIFI:");
  lcd.setCursor(0, 1);
  lcd.print(wifiManager.getSSID());
   lcd.setCursor(0, 2);  // Move the cursor at origin
  lcd.print("IP address: ");
  lcd.setCursor(0, 3);
  lcd.print(WiFi.localIP());
  //--------------LCD----------------//
  delay(4000); 
  //-----WIFI------//

  

  //-----NRF24L01------//
  //--neu muon thay doi cau hinh spi--//
  //SPI.setHwCs(true);                          //dung hay khong dung che do chon chip tren chan cs(thuong thi chi 1 slave thi de false)
  SPI.begin();                                //khoi dong spi
  SPI.setDataMode(SPI_MODE0);                 //SPI_MODE0 0x00 - CPOL: 0 CPHA: 0 |||||CPOL, CPHA: liên quan d?n vi?c thi?t l?p data dich chuy?n là c?nh lên hay c?nh xu?ng c?a clock.
  SPI.setBitOrder(LSBFIRST);                  //bit thap di truoc
  //--neu muon thay doi cau hinh spi--//
  radio.begin();                                  // khai bao  bat dau nRF24L01+
  //radio.setAutoAck(false);                       //cho phep tuy chinh luong truyen goi ack
  radio.setChannel(4);                            // chon kenh
  radio.setDataRate(RF24_1MBPS);                  // chon toc do truyen (RF24_250KBPS, RF24_1MBPS, RF24_2MBPS), RF24_1MBPS - 1????/???
                                                  //M?ch Thu Phát RF NRF24L01 + PA LNA 2.4Ghz Anten R?i.||250kbps to 2Mbps.
                                                  //M?ch Thu Phát RF NRF24L01 +                         ||250kbps to 2Mbps.
  radio.setPALevel(RF24_PA_MAX);                  // set do loi (hay cong suat phat)
                                                  //M?ch Thu Phát RF NRF24L01 + PA LNA 2.4Ghz Anten R?i.||-18dBm,-12dBm,-6dBm,0dBm qua mach PA co the len den 20dBm
                                                  //M?ch Thu Phát RF NRF24L01 +                         ||-18dBm,-12dBm,-6dBm,0dBm
  //yeu cau mo kenh (radio.openWritingPipe (ID2a)) phai duoc thuc hien ngay truoc luc truyen moi truyen duoc
  radio.openReadingPipe(0, ID1a);                 // ham mo luong 1 lang nghe du lieu den o dia chi ID1a//-------luu y: mo la mo luon
  radio.openReadingPipe(0, ID2a);                 // ham mo luong 2 lang nghe du lieu den o dia chi ID2a//
  //radio.stopListening();
  //radio.printDetails();
  //-----NRF24L01------//
}
//------------------------------------------------------------------------------------------SETUP---------------------------------------------------------------------//

//------------------------------------------------------------------------------------------LOOP---------------------------------------------------------------------//
void loop()
{
  Slave1a();
  if(next_1==1)
  {
    //-------------xu li truyen yeu cau bat tat bom-------------//
    memset(kiem_data,0x00,32);//xoa cac ki tu trong kiem_data
//    for(int i=0;i<32;i++)
//    {
//      Serial.println(kiem_data[i]);
//    }
    strncpy(kiem_data, data+25,sizeof(data)-24);//gan vao chuoi kiem_data  bat dau tu ki tu thu 20 den do dai cua chuoi data
    //Serial.println(kiem_data);
    if(strcmp(kiem_data,"ON_1b")==0 || strcmp(kiem_data,"OFF_1b")==0 || strcmp(kiem_data,"ON_2b")==0 || strcmp(kiem_data,"OFF_2b")==0)
    {
      Slave1b();
    }
    else
    {
      Serial.println("loi he thong");
      strcpy(data, "");
      strcpy(data, "C=loi&H=loi&S=loi&R=loi&M=loi4");//cap nhat he thong loi
      if(devide==1)
      {
        truyen_len_serve_slave3();
      }
      else if(devide==2)
      {
        truyen_len_serve_slave4();
      }
      //-----------LCD-----------//
      lcd.setCursor(14, 0);  // Move the cursor at origin
      lcd.print("LOI4");
      //-----------LCD-----------//
    }
    strcpy(kiem_data, "");//xoa cac ki tu trong kiem_data
    strcpy(data, "");//xoa cac ki tu trong data
    //-------------xu li truyen yeu cau bat tat bom-------------//
  }
  //---------------------chuyen qua lai giua cac slave--------------------------//
  if(devide==1)
  {
    for(int k=0;k<5;k++)
    {
      ID1a[k]=ID2a[k];
      ID1b[k]=ID2b[k];
    }
    devide=2;
  }
  else if(devide==2)
  {
    for(int k=0;k<5;k++)
    {
      ID1a[k]=ID3a[k];
      ID1b[k]=ID3b[k];
    }
    devide=1;
  }
  //---------------------chuyen qua lai giua cac slave--------------------------//



  //khoang thoi gian cap nhat lai//
  for(int i=0;i<8;i++)
  {
    Serial.println("+++++++++++++++++++++++++++++++++++++++++++++++CONTINUTE++++++++++++++++++++++++++++++++++++++++++++++");
    delay(100);
  }
  //khoang thoi gian cap nhat lai//
}
//------------------------------------------------------------------------------------------LOOP---------------------------------------------------------------------//





//------------------------------------------------------------------------------------------FUNCTION---------------------------------------------------------------------//
void Slave1a()
{

  
  //doc max-min tu app//
  lcd.clear();
  if(devide==1)
  {
    load();
    for(int i=0;i<11;i++)
    {
      LoadLcdMin[i]=payload[i];
      LoadLcdMax[i]=payload[i+11];
      if(i<5)
      {
        so_sanh_1[i]=payload[i];
      }
      //Serial.println(payload[i]);
    }
    lcd.setCursor(0, 0);  // Move the cursor at origin
    lcd.print("Load Min, Max 1");
    lcd.setCursor(0, 1);  // Move the cursor at origin
    lcd.print(LoadLcdMin);
    lcd.setCursor(0, 2);  // Move the cursor at origin
    lcd.print(LoadLcdMax);
  }
  else if(devide==2)
  {
    load2();
    for(int i=0;i<11;i++)
    {
      LoadLcdMin[i]=payload2[i];
      LoadLcdMax[i]=payload2[i+11];
      if(i<5)
      {
        so_sanh_2[i]=payload2[i];
      }
    }
    lcd.setCursor(0, 0);  // Move the cursor at origin
    lcd.print("Load Min, Max 2");
    lcd.setCursor(0, 1);  // Move the cursor at origin
    lcd.print(LoadLcdMin);
    lcd.setCursor(0, 2);  // Move the cursor at origin
    lcd.print(LoadLcdMax);
  }
  delay(1000);
  //doc max-min tu app//
  
  if(strcmp(so_sanh_1,"min1a")==0 || strcmp(so_sanh_2,"min2a")==0)
  {
    //-----------LCD-----------//
    lcd.clear();
    lcd.setCursor(0, 0);  // Move the cursor at origin
    if(devide==1)
    {
      lcd.print("GOI SLAVE 1A: ...");
    }
    else if(devide==2)
    {
      lcd.print("GOI SLAVE 2A: ...");
    }
    //-----------LCD-----------//
    
    //char check_1a[18] = "yeu cau slave 1a ";
    //strcat(check_1a, payload); //noi chuoi check_1a voi chuoi payload
    //Serial.println(check_1a);
    a=0;
    b=0;
    c=0;
    next_1=0;
    thoi_gian_cho = 0;
    thoi_gian_check=0;
    radio.stopListening();
    //radio.setAutoAck(1,true);
    //truyen yeu cau bat dau doc du lieu moi truong
    radio.openWritingPipe (ID1a);   //mo kenh truyen co dia chi ID1a(mac dinh luon luon truyen tren luong 0(khong doi duoc) vi vay truoc khi truyen phai goi ham nay de neu nhu truoc do co dung luong 0 de doc thi ham nay se chuyen thanh viet tren luong 0)
    if(devide==1)
    {
      Serial.print("yeu cau slave 1a doc du lieu\r\n");
    }
    else if(devide==2)
    {
      Serial.print("yeu cau slave 2a doc du lieu\r\n");
    }
    
    //Serial.println(check_tx);
    //vao che do doc sau khi truyen yeu cau doc du lieu
    //radio.printDetails();
    //delay(1000);
    //-------------------------vao che do doc du lieu------------------------//
    if(devide==1)
    {
      radio.write(&payload, sizeof(payload));
    }
    else if(devide==2)
    {
      radio.write(&payload2, sizeof(payload2));
    }
    //delay(2000);
    for(int k=0;k<5;k++)
    {
      //Serial.println(ID1a[k]);
    }
    radio.openReadingPipe(0, ID1a);
    radio.startListening();//can than thoi gian chay ham nay qua dong ho thoi gian cua nodemcu
    Serial.println  ("dang doi du lieu...\r\n");
    while(a==0)//doi den khi nhan duoc chuoi
    {
      delayMicroseconds(500); //delay de co the nhan du lieu duoc <=> voi khoang thoi gian Serial.println  ("dang doi du lieu...\r\n");
      //Serial.println  ("dang doi du lieu...\r\n");
      thoi_gian_cho +=1;
      //Serial.println(thoi_gian_cho);
      rx_1a();
      if(thoi_gian_cho > 30000)//delay 5 thi de thoi gian cho > 3000 la chuan
      {
        a=1;//reset lai 
        radio.stopListening();
        strcpy(data, "C=loi&H=loi&S=loi&M=loi&R=loi1");//cap nhat he thong loi
        if(devide==1)
        {
          truyen_len_serve_slave3();
        }
        else if(devide==2)
        {
          truyen_len_serve_slave4();
        }
        strcpy(data, "");//xoa cac ki tu trong data
        //Serial.println  ("thuc hien gui lai yeu cau truyen du lieu voi slave 1a\r\n");
        //-----------LCD-----------//
        lcd.setCursor(14, 0);  // Move the cursor at origin
        lcd.print("LOI1");
        //-----------LCD-----------//
      }
      yield();
    }
    //-------------------------vao che do doc du lieu------------------------//
  }
  else
  {
    lcd.setCursor(0, 1);  // Move the cursor at origin
    lcd.print("Mang yeu");
    lcd.setCursor(0, 2);  // Move the cursor at origin
    lcd.print("Khong the load Min, Max");  
    delay(1000);
  }
}
void rx_1a()
{
  if(radio.available())////doc tren thanh ghi STATUS:0x07, kiem tra tai bit thu 6 la 0(khong co chuoi) hay 1(co chuoi moi)
  {
    radio.read(&data, sizeof(data));//doc 32 byte (sizeof(data) bang 32) cua du lieu den va luu vao trong bien data
    Serial.println("co phan hoi tu slave\r\n");
    Serial.print("Received : ");
    Serial.println(data);//show chuoi den
    radio.stopListening();//ngung che do nhan du lieu
    Serial.print("gui ok de check lan cuoi\r\n");
    delay(1000);
    radio.openWritingPipe (ID1a);
    radio.write(&ok, sizeof(ok));
    delay(1000);
    Serial.print("doi ok\r\n");
    radio.openReadingPipe(0, ID1a);
    radio.startListening();//can than thoi gian chay ham nay qua dong ho thoi gian cua nodemcu
    while(b==0)
    {
      thoi_gian_check+=1;
      delayMicroseconds(500); //delay de co the nhan du lieu duoc <=> voi khoang thoi gian Serial.println  ("dang doi du lieu...\r\n");
      //Serial.println();
      //Serial.print("gui ok de check lan cuoi\r\n");
      //Serial.println(thoi_gian_check);    
      if(radio.available())
      {
        Serial.println("OK tu slave\r\n");
        radio.read(&check_ok, sizeof(check_ok));//doc 32 byte (sizeof(data) bang 32) cua du lieu den va luu vao trong bien data
        Serial.print("Received : ");
        Serial.println(check_ok);//show chuoi den
        if(strcmp(check_ok,"ok")==0)
        {
          radio.stopListening();//ngung che do nhan du lieu
          b=1;//thoat che do gui
          a=1;//thoat slave
          next_1=1;
          
          Serial.println("******************KET THUC-ok***************************");
          //-----------LCD-----------//
          lcd.setCursor(14, 0);  // Move the cursor at origin
          lcd.print("OK ");
          strcpy(C1, "");
          strcpy(H1, "");
          strcpy(S1, "");
          strncpy(C1, data+1,6);//gan vao chuoi kiem_data gia tri tu 1 den 6 cua data
          strncpy(H1, data+8,6);//gan vao chuoi kiem_data gia tri tu 8 den 13 cua data
          strncpy(S1, data+15,4);//gan vao chuoi kiem_data gia tri tu 15 den 18 cua data
          C1[5]='\0';
          H1[5]='\0';
          S1[3]='\0';
          //Serial.println(C1);
          //Serial.println(H1);
          //Serial.println(S1);
          lcd.setCursor(0, 1);  // Move the cursor at origin
          lcd.print("ND");
          lcd.setCursor(2, 1);  // Move the cursor at origin
          lcd.print(C1);
          lcd.setCursor(7, 1);  // Move the cursor at origin
          lcd.print("oC");
          lcd.setCursor(12, 1);  // Move the cursor at origin
          lcd.print("DAD");
          lcd.setCursor(15, 1);  // Move the cursor at origin
          lcd.print(S1);
          lcd.setCursor(18, 1);  // Move the cursor at origin
          lcd.print("%");
          lcd.setCursor(0, 2);  // Move the cursor at origin
          lcd.print("DAKK");
          lcd.setCursor(4, 2);  // Move the cursor at origin
          lcd.print(H1);
          lcd.setCursor(9, 2);  // Move the cursor at origin
          lcd.print("%");
          //-----------LCD-----------//
        }
        else
        {
          radio.stopListening();//ngung che do nhan du lieu
          b=1;//thoat che do gui
          a=1;//thoat slave
          strcpy(data, "C=loi&H=loi&S=loi&M=loi&R=loi3");//cap nhat he thong loi
          Serial.println(data);
          if(devide==1)
          {
            truyen_len_serve_slave3();
          }
          else if(devide==2)
          {
            truyen_len_serve_slave4();
          }
          //strcpy(data, "");//xoa cac ki tu trong data
          Serial.println("******************KET THUC 1a-false***************************");
          //-----------LCD-----------//
          lcd.setCursor(14, 0);  // Move the cursor at origin
          lcd.print("LOI3");
          //-----------LCD-----------//
        }
      }
      if((thoi_gian_check>30000))
      {
        radio.stopListening();//ngung che do nhan du lieu
        b=1;//thoat che do gui
        a=1;//thoat slave
        strcpy(data, "C=loi&H=loi&S=loi&M=loi&R=loi2");//cap nhat he thong loi
        Serial.println(data);
        if(devide==1)
        {
          truyen_len_serve_slave3();
        }
        else if(devide==2)
        {
          truyen_len_serve_slave4();
        }
        //strcpy(data, "");//xoa cac ki tu trong data
        Serial.println("******************KET THUC 1a-false***************************");
        //-----------LCD-----------//
        lcd.setCursor(14, 0);  // Move the cursor at origin
        lcd.print("LOI2");
        //-----------LCD-----------//
      }
      yield();
    }
  }
  //radio.stopListening();
}

//----------------------------------------//

void Slave1b()
{
  //-----------LCD-----------//
  lcd.setCursor(0, 3);  // Move the cursor at origin
  if(devide==1)
  {
    lcd.print("GOI SLAVE 1B: ...");
  }
  else if(devide==2)
  {
    lcd.print("GOI SLAVE 2B: ...");
  }
  //-----------LCD-----------//
  a=0;
  b=0;
  thoi_gian_cho = 0;
  thoi_gian_check=0;
  radio.stopListening();
  for(int k=0;k<5;k++)
  {
    //Serial.println(ID1b[k]);
  }
  //truyen yeu cau bat dau doc du lieu bom 1b
  radio.openWritingPipe (ID1b);   //mo kenh truyen co dia chi ID1b(mac dinh luon luon truyen tren luong 0(khong doi duoc) vi vay truoc khi truyen phai goi ham nay de neu nhu truoc do co dung luong 0 de doc thi ham nay se chuyen thanh viet tren luong 0)
  if(devide==1)
  {
    Serial.print("yeu cau slave 1b: ");
  }
  else if(devide==2)
  {
    Serial.print("yeu cau slave 2b: ");
  }
  Serial.println(kiem_data);
  //vao che do doc sau khi truyen yeu cau doc du lieu
  //radio.printDetails();
  //delay(1000);
  //-------------------------vao che do doc du lieu------------------------//
  radio.write(&kiem_data, sizeof(kiem_data));
  //delay(2000);
  radio.openReadingPipe(0, ID1b);
  radio.startListening();//can than thoi gian chay ham nay qua dong ho thoi gian cua nodemcu
  Serial.println  ("dang doi du lieu...\r\n");
  while(a==0)//doi den khi nhan duoc chuoi
  {
    delayMicroseconds(500); //delay de co the nhan du lieu duoc <=> voi khoang thoi gian Serial.println  ("dang doi du lieu...\r\n");
    //Serial.println  ("dang doi du lieu 1b...\r\n");
    thoi_gian_cho +=1;
    //Serial.println(thoi_gian_cho);
    rx_1b();
    if(thoi_gian_cho > 30000)
    {
      a=1;//reset lai 
      radio.stopListening();
      memset(kiem_data,0x00,32);//xoa cac ki tu trong kiem_data
      strncpy(kiem_data, data+0,25);//gan vao chuoi kiem_data gia tri tu 0 den 20 cua data
      kiem_data[25] = '\0';//de may hieu chi toi ki tu so 225 cua kiem_data
      memset(data,0x00,32);//xoa cac ki tu trong kiem_data  
      strcpy(data, kiem_data); // Gán chu?i data có giá tr? là chu?i kiem_data
      strcat(data, "loi5"); // n?i chu?i "loi" vào sau chu?i data
      //Serial.println("data");
      if(devide==1)
      {
        truyen_len_serve_slave1();
        truyen_len_serve_slave3();
      }
      else if(devide==2)
      {
        truyen_len_serve_slave2();
        truyen_len_serve_slave4();
      }

      //Serial.println  ("thuc hien gui lai yeu cau truyen du lieu voi slave 1b\r\n");
      //-----------LCD-----------//
      lcd.setCursor(14, 3);  // Move the cursor at origin
      lcd.print("LOI5");
      //-----------LCD-----------//
    }
    yield();
  }
  //-------------------------vao che do doc du lieu------------------------//
}
void rx_1b()
{
  if(radio.available())
  {
    if(devide==1)
    {
      Serial.println("co phan hoi tu slave 1b");
    }
    else if(devide==2)
    {
      Serial.println("co phan hoi tu slave 2b");
    }
    
    radio.read(&data_b, sizeof(data_b));//doc 32 byte (sizeof(data) bang 32) cua du lieu den va luu vao trong bien data
    Serial.print("Received : ");
    Serial.println(data_b);//show chuoi den
    if(strcmp(data_b,"ON_1b")==0 || strcmp(data_b,"ON_2b")==0)//neu da dung trang thai yeu cau thi khong gui gui
    {
      radio.stopListening();//ngung che do nhan du lieu
      Serial.print("gui ok_ON de check lan cuoi\r\n");
      delay(1000);
      radio.openWritingPipe (ID1b);
      radio.write(&ok, sizeof(ok));
      delay(1000);
      Serial.print("doi ok\r\n");
      radio.openReadingPipe(0, ID1b);
      radio.startListening();//can than thoi gian chay ham nay qua dong ho thoi gian cua nodemcu
      while(b==0)
      {
        thoi_gian_check+=1;
        delayMicroseconds(500); //delay de co the nhan du lieu duoc <=> voi khoang thoi gian Serial.println  ("dang doi du lieu...\r\n");
        if(radio.available())
        {
          Serial.println("OK tu slave\r\n");
          radio.read(&check_ok, sizeof(check_ok));//doc 32 byte (sizeof(data) bang 32) cua du lieu den va luu vao trong bien data
          Serial.print("Received : ");
          Serial.println(check_ok);//show chuoi den
          if(strcmp(check_ok,"ok")==0)
          {
            radio.stopListening();//ngung che do nhan du lieu
            b=1;//thoat che do gui
            a=1;//thoat slave
            memset(kiem_data,0x00,32);//xoa cac ki tu trong kiem_data
            //Serial.println(data);
            strncpy(kiem_data, data+0,27);//gan vao chuoi kiem_data  bat dau tu ki tu thu 20 den do dai cua chuoi data
            kiem_data[27] = '\0';//de may hieu chi toi ki tu so 225 cua kiem_data
            //Serial.println(kiem_data);
            memset(data,0x00,32);//xoa cac ki tu trong kiem_data
            strcpy(data, kiem_data); // Gán chu?i data có giá tr? là chu?i kiem_data
            if(devide==1)
            {
              truyen_len_serve_slave1();
              truyen_len_serve_slave3();
            }
            else if(devide==2)
            {
              truyen_len_serve_slave2();
              truyen_len_serve_slave4();
            }
            //strcpy(data, "");//xoa cac ki tu trong data
            //Serial.println(data);
            //-----------LCD-----------//
            lcd.setCursor(14, 3);  // Move the cursor at origin
            lcd.print("OK ");
            lcd.setCursor(17, 3);  // Move the cursor at origin
            lcd.print("ON");
            //-----------LCD-----------//
          }
          else 
          {
            radio.stopListening();//ngung che do nhan du lieu
            b=1;//thoat che do gui
            a=1;//thoat slave
            memset(kiem_data,0x00,32);//xoa cac ki tu trong kiem_data
            strncpy(kiem_data, data+0,25);//gan vao chuoi kiem_data gia tri tu 0 den 20 cua data
            kiem_data[25] = '\0';//de may hieu chi toi ki tu so 225 cua kiem_data
            memset(data,0x00,32);//xoa cac ki tu trong kiem_data     
            strcpy(data, kiem_data); // Gán chu?i data có giá tr? là chu?i kiem_data
            strcat(data, "loi9"); // n?i chu?i "loi" vào sau chu?i data
            //Serial.println("data");
            if(devide==1)
            {
              truyen_len_serve_slave1();
              truyen_len_serve_slave3();
            }
            else if(devide==2)
            {
              truyen_len_serve_slave2();
              truyen_len_serve_slave4();
            }
            //-----------LCD-----------//
            lcd.setCursor(14, 3);  // Move the cursor at origin
            lcd.print("LOI9");
            //-----------LCD-----------//
          }
        }
        if((thoi_gian_check>30000))
        {
          radio.stopListening();//ngung che do nhan du lieu
          b=1;//thoat che do gui
          a=1;//thoat slave
          memset(kiem_data,0x00,32);//xoa cac ki tu trong kiem_data
          strncpy(kiem_data, data+0,25);//gan vao chuoi kiem_data gia tri tu 0 den 20 cua data
          kiem_data[25] = '\0';//de may hieu chi toi ki tu so 225 cua kiem_data
          memset(data,0x00,32);//xoa cac ki tu trong kiem_data 
          strcpy(data, kiem_data); // Gán chu?i data có giá tr? là chu?i kiem_data
          strcat(data, "loi8"); // n?i chu?i "loi" vào sau chu?i data
          //Serial.println("data");
          truyen_len_serve_slave1();
          truyen_len_serve_slave3();
          //-----------LCD-----------//
          lcd.setCursor(14, 3);  // Move the cursor at origin
          lcd.print("LOI8");
          //-----------LCD-----------//
        }
        yield();
      }
    }    
    else if(strcmp(data_b,"OFF_1b")==0 || strcmp(data_b,"OFF_2b")==0)//neu da dung trang thai yeu cau thi khong gui gui
    {      
      radio.stopListening();//ngung che do nhan du lieu
      Serial.print("gui ok_OFF de check lan cuoi\r\n");
      delay(1000);
      radio.openWritingPipe (ID1b);
      radio.write(&ok, sizeof(ok));
      delay(1000);
      Serial.print("doi ok\r\n");
      radio.openReadingPipe(0, ID1b);
      radio.startListening();//can than thoi gian chay ham nay qua dong ho thoi gian cua nodemcu
      while(b==0)
      {
        thoi_gian_check+=1;
        delay(5);//delay de co the truyen du lieu duoc <=> voi khoang thoi gian Serial.println  ("gui lenh bat-tat bom\r\n");
        if(radio.available())
        {
          Serial.println("OK tu slave\r\n");
          radio.read(&check_ok, sizeof(check_ok));//doc 32 byte (sizeof(data) bang 32) cua du lieu den va luu vao trong bien data
          Serial.print("Received : ");
          Serial.println(check_ok);//show chuoi den
          if(strcmp(check_ok,"ok")==0)
          {
            radio.stopListening();//ngung che do nhan du lieu
            b=1;//thoat che do gui
            a=1;//thoat slave
            memset(kiem_data,0x00,32);//xoa cac ki tu trong kiem_data
            strncpy(kiem_data, data+0,28);//gan vao chuoi kiem_data  bat dau tu ki tu thu 20 den do dai cua chuoi data
            //Serial.println(sizeof(data));
            kiem_data[28] = '\0';//de may hieu chi toi ki tu so 225 cua kiem_data
            memset(data,0x00,32);//xoa cac ki tu trong kiem_data
            strcpy(data, kiem_data); // Gán chu?i data có giá tr? là chu?i kiem_data
            if(devide==1)
            {
              truyen_len_serve_slave1();
              truyen_len_serve_slave3();
            }
            else if(devide==2)
            {
              truyen_len_serve_slave2();
              truyen_len_serve_slave4();
            }
            //strcpy(data, "");//xoa cac ki tu trong data
            //Serial.println(data);
            //-----------LCD-----------//
            lcd.setCursor(14, 3);  // Move the cursor at origin
            lcd.print("OK ");
            lcd.setCursor(17, 3);  // Move the cursor at origin
            lcd.print("OFF");
            //-----------LCD-----------//
          }
          else 
          {
            radio.stopListening();//ngung che do nhan du lieu
            b=1;//thoat che do gui
            a=1;//thoat slave
            memset(kiem_data,0x00,32);//xoa cac ki tu trong kiem_data
            strncpy(kiem_data, data+0,25);//gan vao chuoi kiem_data gia tri tu 0 den 20 cua data
            kiem_data[25] = '\0';//de may hieu chi toi ki tu so 225 cua kiem_data
            memset(data,0x00,32);//xoa cac ki tu trong kiem_data      
            strcpy(data, kiem_data); // Gán chu?i data có giá tr? là chu?i kiem_data
            strcat(data, "loi9"); // n?i chu?i "loi" vào sau chu?i data
            if(devide==1)
            {
              truyen_len_serve_slave1();
              truyen_len_serve_slave3();
            }
            else if(devide==2)
            {
              truyen_len_serve_slave2();
              truyen_len_serve_slave4();
            }
            //-----------LCD-----------//
            lcd.setCursor(14, 3);  // Move the cursor at origin
            lcd.print("LOI9");
            //-----------LCD-----------//
          }
        }
        if((thoi_gian_check>3000))
        {
          radio.stopListening();//ngung che do nhan du lieu
          b=1;//thoat che do gui
          a=1;//thoat slave
          memset(kiem_data,0x00,32);//xoa cac ki tu trong kiem_data
          strncpy(kiem_data, data+0,25);//gan vao chuoi kiem_data gia tri tu 0 den 20 cua data
          kiem_data[25] = '\0';//de may hieu chi toi ki tu so 225 cua kiem_data
          memset(data,0x00,32);//xoa cac ki tu trong kiem_data
          strcpy(data, kiem_data); // Gán chu?i data có giá tr? là chu?i kiem_data
          strcat(data, "loi8"); // n?i chu?i "loi" vào sau chu?i data
          if(devide==1)
          {
            truyen_len_serve_slave1();
            truyen_len_serve_slave3();
          }
          else if(devide==2)
          {
            truyen_len_serve_slave2();
            truyen_len_serve_slave4();
          }
          //-----------LCD-----------//
          lcd.setCursor(14, 3);  // Move the cursor at origin
          lcd.print("LOI8");
          //-----------LCD-----------//
        }
        yield();
      }
    }
    else if(strcmp(data_b,"Khong bat duoc bom_1b")==0 || strcmp(data_b,"Khong tat duoc bom_1b")==0 || strcmp(data_b,"Khong bat duoc bom_2b")==0 || strcmp(data_b,"Khong tat duoc bom_2b")==0)
    {
      radio.stopListening();//ngung che do nhan du lieu   
      b=1;//thoat che do gui
      a=1;//thoat slave
      memset(kiem_data,0x00,32);//xoa cac ki tu trong kiem_data
      strncpy(kiem_data, data+0,25);//gan vao chuoi kiem_data gia tri tu 0 den 20 cua data
      kiem_data[25] = '\0';//de may hieu chi toi ki tu so 225 cua kiem_data
      memset(data,0x00,32);//xoa cac ki tu trong kiem_data
      strcpy(data, kiem_data); // Gán chu?i data có giá tr? là chu?i kiem_data
      strcat(data, data_b); // n?i chu?i "loi" vào sau chu?i data
      if(devide==1)
      {
        truyen_len_serve_slave1();
        truyen_len_serve_slave3();
      }
      else if(devide==2)
      {
        truyen_len_serve_slave2();
        truyen_len_serve_slave4();
      }
      //-----------LCD-----------//
      lcd.setCursor(14, 3);  // Move the cursor at origin
      lcd.print("LOI6");
      //-----------LCD-----------//
    }
    else if(strcmp(data_b,"DK TAY")==0)
    {
      radio.stopListening();//ngung che do nhan du lieu   
      b=1;//thoat che do gui
      a=1;//thoat slave
      memset(kiem_data,0x00,32);//xoa cac ki tu trong kiem_data
//      for(int i=0;i<32;i++)
//      {
//        Serial.println(kiem_data[i]);
//      }
      strncpy(kiem_data, data+0,25);//gan vao chuoi kiem_data gia tri tu 0 den 20 cua data
      kiem_data[25] = '\0';//de may hieu chi toi ki tu so 225 cua kiem_data
      memset(data,0x00,32);//xoa cac ki tu trong kiem_data
      strcpy(data, kiem_data); // Gán chu?i data có giá tr? là chu?i kiem_data
      strcat(data, data_b); // n?i chu?i "loi" vào sau chu?i data
      if(devide==1)
      {
        truyen_len_serve_slave1();
        truyen_len_serve_slave3();
      }
      else if(devide==2)
      {
        truyen_len_serve_slave2();
        truyen_len_serve_slave4();
      }
      //-----------LCD-----------//
      lcd.setCursor(14, 3);  // Move the cursor at origin
      lcd.print("DK TAY");
      //-----------LCD-----------//
    }
    else
    {
      radio.stopListening();//ngung che do nhan du lieu
      b=1;//thoat che do gui
      a=1;//thoat slave
      memset(kiem_data,0x00,32);//xoa cac ki tu trong kiem_data
      strncpy(kiem_data, data+0,25);//gan vao chuoi kiem_data gia tri tu 0 den 20 cua data
      kiem_data[25] = '\0';//de may hieu chi toi ki tu so 225 cua kiem_data
      memset(data,0x00,32);//xoa cac ki tu trong kiem_data
      strcpy(data, kiem_data); // Gán chu?i data có giá tr? là chu?i kiem_data
      strcat(data, "loi7"); // n?i chu?i "loi" vào sau chu?i data
      if(devide==1)
      {
        truyen_len_serve_slave1();
        truyen_len_serve_slave3();
      }
      else if(devide==2)
      {
        truyen_len_serve_slave2();
        truyen_len_serve_slave4();
      }
      //-----------LCD-----------//
      lcd.setCursor(14, 3);  // Move the cursor at origin
      lcd.print("LOI7");
      //-----------LCD-----------//
    }
  }
}

//----------------------------------------------------------------------------------------------------------------------1---------------------------------------------------------------//


void load(){
  HTTPClient http;    
  //---gui du lieu len server php--------//
  String resquest = "load=1";// Chuoi up len 
  http.begin("http://smartdataloggerspace.000webhostapp.com/host/max-min.php");
  http.addHeader("Content-Type", "application/x-www-form-urlencoded");    //Specify content-type header
  int httpCode = http.POST(resquest);   //Send the request
  //---gui du lieu len server php--------//

  //---nhan phan hoi tu server php--------//
  //Serial.println(httpCode);   //Print HTTP return code
  Serial.print("load: ");
  String payload0 = http.getString();    //chuoi payload0 se co 4 ki tu dau bi loi nen phai doc tu vi tri so 5
  //Serial.println(payload0.length());//22
  for (byte len = 5;len<=payload0.length()+1; len++)
  {
    payload0.toCharArray(payload, len);
    //Serial.println(payload);
  }
  //payload[payload0.length()+2] = '\0';
  Serial.println(payload);    //Print request response payload
  //---nhan phan hoi tu server php--------//

  
  http.end();  //Close connection
  
  delay(1000);  //Post Data at every 2 seconds
}

void load2(){
  HTTPClient http;    
  //---gui du lieu len server php--------//
  String resquest2 = "load=1";// Chuoi up len 
  http.begin("http://smartdataloggerspace.000webhostapp.com/host/max-min2.php");
  http.addHeader("Content-Type", "application/x-www-form-urlencoded");    //Specify content-type header
  int httpCode = http.POST(resquest2);   //Send the request
  //---gui du lieu len server php--------//

  //---nhan phan hoi tu server php--------//
  //Serial.println(httpCode);   //Print HTTP return code
  Serial.print("load: ");
  String payload0 = http.getString();    //Get the response payload
  for (byte len = 5;len<=payload0.length()+1; len++){
    payload0.toCharArray(payload2, len);
  }
  Serial.println(payload2);    //Print request response payload
  //---nhan phan hoi tu server php--------//

  
  http.end();  //Close connection
  
  delay(1000);  //Post Data at every 2 seconds
}


void truyen_len_serve_slave1()
{
  HTTPClient http;    

  //String ADCData, station, postData;
  //int adcvalue=50; //bien du lieu
  //int bcd=20;  //bien du lieu
  //ADCData = String(adcvalue);   //String to interger conversion
  //station = String(bcd);
  //Serial.println(ADCData);
  //Serial.println(station);
  // bien nhiet_do,do_am giong bien trong database
  //strcat(id_1, data); // n?i chu?i data vào sau chu?i id_1
  //---gui du lieu len server php--------//
  
  Serial.print("Send data: ");
  //postData = "nhiet_do=" + ADCData + "&do_am=" + station ;// Chuoi up len 
  Serial.println(data);
  //http.begin("http://192.168.43.106:80/doan/esp.php");              //duong dan file php... vi dang ket noi wifi nen phai lay ip may thay vi localhost
  //http.begin("http://192.168.1.2:80/doan/update.php");              //duong dan file php... vi dang ket noi wifi nen phai lay ip may thay vi localhost
  http.begin("http://smartdataloggerspace.000webhostapp.com/host/update.php");    
  http.addHeader("Content-Type", "application/x-www-form-urlencoded");    //Specify content-type header
  int httpCode = http.POST(data);   //Send the request
  //---gui du lieu len server php--------//

  //---nhan phan hoi tu server php--------//
  String payload = http.getString();    //Get the response payload
  Serial.println(httpCode);   //Print HTTP return code
  Serial.println(payload);    //Print request response payload
  //---nhan phan hoi tu server php--------//

  
  http.end();  //Close connection
  //strcpy(id_1, "id=1&");
  delay(1000);  //Post Data at every 2 seconds
}

void truyen_len_serve_slave2()
{
  HTTPClient http;    
  
  //String ADCData, station, postData;
  //int adcvalue=50; //bien du lieu
  //int bcd=20;  //bien du lieu
  //ADCData = String(adcvalue);   //String to interger conversion
  //station = String(bcd);
  //Serial.println(ADCData);
  //Serial.println(station);
  // bien nhiet_do,do_am giong bien trong database
  //strcat(id_2, data); // n?i chu?i data vào sau chu?i id_2
  //---gui du lieu len server php--------//
  Serial.print("Send data: ");
  //postData = "nhiet_do=" + ADCData + "&do_am=" + station ;// Chuoi up len 
  Serial.println(data);
  //http.begin("http://192.168.43.106:80/doan/esp.php");              //duong dan file php... vi dang ket noi wifi nen phai lay ip may thay vi localhost
  //http.begin("http://192.168.1.2:80/doan/update2.php");              //duong dan file php... vi dang ket noi wifi nen phai lay ip may thay vi localhost
  http.begin("http://smartdataloggerspace.000webhostapp.com/host/update2.php");    
  http.addHeader("Content-Type", "application/x-www-form-urlencoded");    //Specify content-type header
  int httpCode = http.POST(data);   //Send the request
  //---gui du lieu len server php--------//

  //---nhan phan hoi tu server php--------//
  String payload = http.getString();    //Get the response payload
  Serial.println(httpCode);   //Print HTTP return code
  Serial.println(payload);    //Print request response payload
  //---nhan phan hoi tu server php--------//

  
  http.end();  //Close connection
  //strcpy(id_2, "");
  //strcpy(id_2, "id=2&"); // khoi phuc du lieu cho id_2
  delay(1000);  //Post Data at every 2 seconds
}

void truyen_len_serve_slave3()
{
  HTTPClient http;    
  
  //String ADCData, station, postData;
  //int adcvalue=50; //bien du lieu
  //int bcd=20;  //bien du lieu
  //ADCData = String(adcvalue);   //String to interger conversion
  //station = String(bcd);
  //Serial.println(ADCData);
  //Serial.println(station);
  // bien nhiet_do,do_am giong bien trong database
  //strcat(id_2, data); // n?i chu?i data vào sau chu?i id_2
  //---gui du lieu len server php--------//
  Serial.print("Send data: ");
  //postData = "nhiet_do=" + ADCData + "&do_am=" + station ;// Chuoi up len 
  Serial.println(data);
  //http.begin("http://192.168.43.106:80/doan/esp.php");              //duong dan file php... vi dang ket noi wifi nen phai lay ip may thay vi localhost
  //http.begin("http://192.168.1.2:80/doan/update3.php");              //duong dan file php... vi dang ket noi wifi nen phai lay ip may thay vi localhost
  http.begin("http://smartdataloggerspace.000webhostapp.com/host/update3.php");    
  http.addHeader("Content-Type", "application/x-www-form-urlencoded");    //Specify content-type header
  int httpCode = http.POST(data);   //Send the request
  //---gui du lieu len server php--------//

  //---nhan phan hoi tu server php--------//
  String payload = http.getString();    //Get the response payload
  Serial.println(httpCode);   //Print HTTP return code
  Serial.println(payload);    //Print request response payload
  //---nhan phan hoi tu server php--------//

  
  http.end();  //Close connection
  //strcpy(id_2, "");
  //strcpy(id_2, "id=2&"); // khoi phuc du lieu cho id_2
  delay(1000);  //Post Data at every 2 seconds
}

void truyen_len_serve_slave4()
{
  HTTPClient http;    
  
  //String ADCData, station, postData;
  //int adcvalue=50; //bien du lieu
  //int bcd=20;  //bien du lieu
  //ADCData = String(adcvalue);   //String to interger conversion
  //station = String(bcd);
  //Serial.println(ADCData);
  //Serial.println(station);
  // bien nhiet_do,do_am giong bien trong database
  //strcat(id_2, data); // n?i chu?i data vào sau chu?i id_2
  //---gui du lieu len server php--------//
  Serial.print("Send data: ");
  //postData = "nhiet_do=" + ADCData + "&do_am=" + station ;// Chuoi up len 
  Serial.println(data);
  //http.begin("http://192.168.43.106:80/doan/esp.php");              //duong dan file php... vi dang ket noi wifi nen phai lay ip may thay vi localhost
  //http.begin("http://192.168.1.2:80/doan/update4.php");              //duong dan file php... vi dang ket noi wifi nen phai lay ip may thay vi localhost
  http.begin("http://smartdataloggerspace.000webhostapp.com/host/update4.php");    
  http.addHeader("Content-Type", "application/x-www-form-urlencoded");    //Specify content-type header
  int httpCode = http.POST(data);   //Send the request
  //---gui du lieu len server php--------//

  //---nhan phan hoi tu server php--------//
  String payload = http.getString();    //Get the response payload
  Serial.println(httpCode);   //Print HTTP return code
  Serial.println(payload);    //Print request response payload
  //---nhan phan hoi tu server php--------//

  
  http.end();  //Close connection
  //strcpy(id_2, "");
  //strcpy(id_2, "id=2&"); // khoi phuc du lieu cho id_2
  delay(1000);  //Post Data at every 2 seconds
}
//------------------------------------------------------------------------------------------FUNCTION---------------------------------------------------------------------//
