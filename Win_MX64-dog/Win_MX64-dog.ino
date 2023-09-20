#include <Arduino.h>
#include <math.h>
#include <SPI.h>

#define FILTER_N 12
int coe[FILTER_N] = {1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12};    // 加權係數表
int sum_coe = 1 + 2 + 3 + 4 + 5 + 6 + 7 + 8 + 9 + 10 + 11 + 12; // 加權係數和
int filter_buf[FILTER_N + 1];

// Define parameters
#define KP_X (0.6f) //0.94
#define KI_X (0.2f) //0.004
#define KD_X (0.005f) //0.019
#define KP_Y (0.03f) //0.94
#define KI_Y (0.035f) //0.004
#define KD_Y (0.00f) //0.019

#define FSR_1 (19)
#define FSR_2 (18)
#define FSR_3 (17)
#define FSR_4 (16)


//MX64 parameters
unsigned char incomingByte = 0x00;   // for incoming serial data
uint16_t pos;
uint16_t map_theta1[4], map_theta2[4], map_theta3[4];
unsigned char pos_low, pos_high , pos_pos;
unsigned char  metor_id[12] = {0x01 , 0x02 , 0x03, 0x04, 0x05 , 0x06 , 0x07, 0x08, 0x09, 0x0A, 0x0B, 0X0C} ;
bool start_ = false;
char start_command;
int pattern  ;

//DOG ROBOT PARAMETERS
const int ite;
int i, counter;
double theta1[4], theta2[4], theta3[4], x, y, z;//variables
double newz;
double theta1_0, theta2_0, theta3_0, x_0, y_0, z_0;
double l1, l2, a1, a2, a3, a4, a5, angle1, angle2, angle3, r, y_axis, z_axis, half_length, half_width, body_propotion;//parameters
double L, w_dir;
float C_theta_stand_ud, c_changetheta_stand_ud, c;
bool change = true ;

//IMU PARAMETERS
float rool, pitch, yaw, pre_pitch, pre_rool,decrease_noise;
float filter_pitch, filter_rool;
time_t Start_time;
byte* r_byte;
byte* p_byte;
byte* y_byte;
int dt;
byte* readin;
float set_point_x , integral_x, prev_error_x, error_x, derivative_x, output_x;
int pitchb ,pitchf;
float set_point_y , integral_y, prev_error_y, error_y, derivative_y, output_y;
int rool_b ,rool_f,rool_b1,rool_f1;
unsigned long imu_time;
bool open_imu = false;
// float imu_number = 0.0;

//FSR PARAMETERS
int FSR_V_1 ,FSR_V_2, FSR_V_3, FSR_V_4;
int FSR_V_1_offset, FSR_V_2_offset, FSR_V_3_offset, FSR_V_4_offset;

//update_crc function from robotis documentation
uint16_t update_crc(uint16_t crc_accum, unsigned char *data_blk_ptr, unsigned short data_blk_size) {
  uint16_t i, j;
  uint16_t crc_table[256] = {
    0x0000, 0x8005, 0x800F, 0x000A, 0x801B, 0x001E, 0x0014, 0x8011,
    0x8033, 0x0036, 0x003C, 0x8039, 0x0028, 0x802D, 0x8027, 0x0022,
    0x8063, 0x0066, 0x006C, 0x8069, 0x0078, 0x807D, 0x8077, 0x0072,
    0x0050, 0x8055, 0x805F, 0x005A, 0x804B, 0x004E, 0x0044, 0x8041,
    0x80C3, 0x00C6, 0x00CC, 0x80C9, 0x00D8, 0x80DD, 0x80D7, 0x00D2,
    0x00F0, 0x80F5, 0x80FF, 0x00FA, 0x80EB, 0x00EE, 0x00E4, 0x80E1,
    0x00A0, 0x80A5, 0x80AF, 0x00AA, 0x80BB, 0x00BE, 0x00B4, 0x80B1,
    0x8093, 0x0096, 0x009C, 0x8099, 0x0088, 0x808D, 0x8087, 0x0082,
    0x8183, 0x0186, 0x018C, 0x8189, 0x0198, 0x819D, 0x8197, 0x0192,
    0x01B0, 0x81B5, 0x81BF, 0x01BA, 0x81AB, 0x01AE, 0x01A4, 0x81A1,
    0x01E0, 0x81E5, 0x81EF, 0x01EA, 0x81FB, 0x01FE, 0x01F4, 0x81F1,
    0x81D3, 0x01D6, 0x01DC, 0x81D9, 0x01C8, 0x81CD, 0x81C7, 0x01C2,
    0x0140, 0x8145, 0x814F, 0x014A, 0x815B, 0x015E, 0x0154, 0x8151,
    0x8173, 0x0176, 0x017C, 0x8179, 0x0168, 0x816D, 0x8167, 0x0162,
    0x8123, 0x0126, 0x012C, 0x8129, 0x0138, 0x813D, 0x8137, 0x0132,
    0x0110, 0x8115, 0x811F, 0x011A, 0x810B, 0x010E, 0x0104, 0x8101,
    0x8303, 0x0306, 0x030C, 0x8309, 0x0318, 0x831D, 0x8317, 0x0312,
    0x0330, 0x8335, 0x833F, 0x033A, 0x832B, 0x032E, 0x0324, 0x8321,
    0x0360, 0x8365, 0x836F, 0x036A, 0x837B, 0x037E, 0x0374, 0x8371,
    0x8353, 0x0356, 0x035C, 0x8359, 0x0348, 0x834D, 0x8347, 0x0342,
    0x03C0, 0x83C5, 0x83CF, 0x03CA, 0x83DB, 0x03DE, 0x03D4, 0x83D1,
    0x83F3, 0x03F6, 0x03FC, 0x83F9, 0x03E8, 0x83ED, 0x83E7, 0x03E2,
    0x83A3, 0x03A6, 0x03AC, 0x83A9, 0x03B8, 0x83BD, 0x83B7, 0x03B2,
    0x0390, 0x8395, 0x839F, 0x039A, 0x838B, 0x038E, 0x0384, 0x8381,
    0x0280, 0x8285, 0x828F, 0x028A, 0x829B, 0x029E, 0x0294, 0x8291,
    0x82B3, 0x02B6, 0x02BC, 0x82B9, 0x02A8, 0x82AD, 0x82A7, 0x02A2,
    0x82E3, 0x02E6, 0x02EC, 0x82E9, 0x02F8, 0x82FD, 0x82F7, 0x02F2,
    0x02D0, 0x82D5, 0x82DF, 0x02DA, 0x82CB, 0x02CE, 0x02C4, 0x82C1,
    0x8243, 0x0246, 0x024C, 0x8249, 0x0258, 0x825D, 0x8257, 0x0252,
    0x0270, 0x8275, 0x827F, 0x027A, 0x826B, 0x026E, 0x0264, 0x8261,
    0x0220, 0x8225, 0x822F, 0x022A, 0x823B, 0x023E, 0x0234, 0x8231,
    0x8213, 0x0216, 0x021C, 0x8219, 0x0208, 0x820D, 0x8207, 0x0202
  };

  for (j = 0; j < data_blk_size; j++)
  {
    i = ((uint16_t)(crc_accum >> 8) ^ data_blk_ptr[j]) & 0xFF;
    crc_accum = (crc_accum << 8) ^ crc_table[i];
  }
  return crc_accum;
}

void setup() {
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(2, OUTPUT);
  digitalWrite(2, HIGH);

  Serial.begin(115200);
  Serial1.begin(115200);
  Serial5.begin(115200);
  
  //Trajectrory
  counter = 0;
  pattern = 0;

  //DOG ROBOT Parameters
  l1 = 0.08; //0.1;
  l2 = 0.122208; //0.102496; //0.16;
  r = 0;
  a1 = 0.03;//0.026;//0.035;
  a2 = 0.03;//0.026;//0.035;
  a3 = 0.0655;//0.045;
  a4 = 0.0655;//0.045;
  a5 = 0.03;//0.035;
  angle1 = 32.45;//31.94;
  angle2 = 64.46; //50;//41.25;
  angle3 =  180 - (acos((a5 * a5 + l2 * l2 - 0.151715 * 0.151715) / (2 * a5 * l2))) / M_PI * 180;
  y_axis = 0.0485;
  z_axis = -0.0068;
  half_length = 0.2748 / 2 ; //0.13/2; //0.244 / 2;
  half_width = 0.204269 / 2 ;//0.26/2; //0.194 / 2;
  body_propotion = atan(half_length / half_width) * 180 / M_PI;
  w_dir = -60; // right: +, left: -

  //zero point
  x_0 = 0.0;
  y_0 = 0.0;
  z_0 = -0.1;
  first_rotation(y_axis, z_axis, y_0, z_0, newz, theta1_0);
  quad_IK_2D(x_0, newz, l1, l2, a1, a2, a3, a4, r, angle1, angle2, angle3, theta2_0, theta3_0);

  pos = 90;
  LED_on();
  LED_off();

  //IMU setup
  pre_pitch = 0.0;
  pre_rool = 0.0;
  decrease_noise = 0.95;
  readin = new byte[13];
  r_byte = new byte[2];
  p_byte = new byte[2];
  y_byte = new byte[2];
  dt = 100;

  Start_time = millis();
  imuRead();
  set_point_x = pitch;//-1.07;
  set_point_y = rool;//-0.56;
  imu_time = millis();

  //FSR setup
  // FSR_dectect();
  // FSR_V_1_offset = -FSR_V_1;
  // FSR_V_2_offset = -FSR_V_2;
  // FSR_V_3_offset = -FSR_V_3;
  // FSR_V_4_offset = -FSR_V_4;
}

void map_pos() {
  pos = map(pos, 0, 360, 0, 4096);
  pos_low = (pos & 0x00ff);
  pos_high = (pos >> 8 ) & 0x00ff;
  // Serial.print("pos   ");Serial.println(pos);
  // Serial.print("poslow   ");Serial.println(pos_low,HEX);
  // Serial.print("poshigh   ");Serial.println(pos_high,HEX);
}

void loop() {  
  imuRead();
  // FSR_dectect();
  if (Serial.available() > 0) {
    start_command = Serial.read();
    if ((char)(start_command) == 's') {
      Serial.println("start");
      syn_torque_on();
      start_ = true;
    }
    if ((char)(start_command) == 'e') {
      Serial.println("false");
      // syn_torque_off();
      start_ = false;
    }
    if ((char)(start_command) == 'z') {
      Serial.println("tozero");
      pattern = 0 ;
    }
    if ((char)(start_command) == 't') {
      Serial.println("trop");
      pattern = 1 ;
    }
    if ((char)(start_command) == 'w') {
      // Serial.println("walk");
      Serial.println("test_syn_write_trop");
      pattern = 2 ;
    }
    if ((char)(start_command) == 'j') {
      Serial.println("small_jump");
      pattern = 3 ;
    }
    if ((char)(start_command) == 'v') {
      Serial.println("left_right");
      pattern = 4 ;
    }
    if ((char)(start_command) == 'u') {
      Serial.println("up_down");
      pattern = 5 ;
    }
    if ((char)(start_command) == 'r') {
      Serial.println("roration_360");
      pattern = 6 ;
    }
    if ((char)(start_command) == 'd') {
      Serial.println("walk_right");
      pattern = 7 ;
    }
    if ((char)(start_command) == 'a') {
      Serial.println("walk_left");
      pattern = 8 ;
    }
    if ((char)(start_command) == 'm') {
      Serial.println("test_syn_write");
      pattern = 9 ;
    }
    if ((char)(start_command) == 'n') {
      Serial.println("test_syn_write_trop");
      pattern = 10 ;
    }
    if ((char)(start_command) == 'b') {
    Serial.println("test_syn_write_trop_rorate_left");
    pattern = 11 ;
    }
    if ((char)(start_command) == 'i') {
    Serial.println("IMU_TEST");
    pattern = 12;
    }
  }
  if (start_) {
    // counter ++;
    if (pattern == 0) {
      Goal_position_toZero(90);
    }
    if (pattern == 1) {
      servopos_t(counter, ite);
    }
    if (pattern == 2) {
      syn_servopos_w(counter, ite);
      if((counter%ite == (ite / 2)) || (counter%ite == ite ) || (counter%ite == (ite / 4 )) || (counter%ite == (0.75 * ite ))){
        open_imu = true;
      }
      if(change == false){
        synwrite_Goal_position_walkortrop_leftright();
        // test_current_time = millis();
        // while(test_current_time - test_previous_time < 1000){};
        change = true;
        counter ++;
      }
    }
    if (pattern == 3) {
      //c = P4 ~ P8 distance
      for (c = 0.11 ; c <= 0.19; c += 0.01) {
        servopos_smalljump();
      }
      for (c = 0.19; c >= 0.11; c -= 0.01) {
        servopos_smalljump();
      }
    }
    if (pattern == 4) {
      servopos_r(counter, ite);
    }
    if (pattern == 5) {
      servopos_stand_up_down(counter, ite);
    }
    if (pattern == 6) {
      servopos_360(counter, ite);
    }
    if (pattern == 7) {
      servopos_rorate_rt(counter, ite);
    }
    if (pattern == 8) {
      servopos_rotate_lt(counter, ite);
    }
    if(pattern == 9){
      synwrite_Goal_position_walkortrop(90);
    }
    if(pattern == 10){
      // unsigned long test_time = 10.5;
      // imu_number = imu_number + 1.0;
      syn_servopos_t(counter, ite);
      if((counter%ite == (ite / 2)) || (counter%ite == ite ) || (counter%ite == (ite / 4 )) || (counter%ite == (0.75 * ite ))){
        open_imu = true;
      }
      if(change == false){
        synwrite_Goal_position_walkortrop_leftright();
        // test_current_time = millis();
        // while(test_current_time - test_previous_time < 1000){};
        change = true;
        counter ++;
      }
    }
    if(pattern == 11){
      // unsigned long test_time = 10.5;
      SYN_servopos_rotate_lt(counter, ite);
      if(change == false){
        synwrite_Goal_position_walkortrop();
        // test_current_time = millis();
        // while(test_current_time - test_previous_time < 1000){};
        change = true;
        counter ++;
      }
    }
    if(pattern == 12){
      imuRead();
      if (millis() - imu_time > 39)
      {
        synwrite_Goal_position_IMU(90);
        imu_time = millis();
      }
    }
  }
}

void torgue_on() {
  int torgue_id = 0;
  for (int start_i = 0 ; start_i < 13 ; start_i++) //12
  {
    // Serial.print("torgue_id:    ");Serial.println(torgue_id);
    uint16_t crc_accum = 0;
    uint16_t CRC;
    unsigned char CRC_L = 0;
    unsigned char CRC_H = 0;
    uint16_t data_blk_size = 18;//size of TxPacket
    unsigned char TxPacket[14] = { 0xFF, 0xFF, 0xFD, 0x00, metor_id[torgue_id], 0x06, 0x00, 0x03, 0x40, 0x00, 0x01, CRC_L, CRC_H };
    CRC = update_crc(0, TxPacket, 11);
    CRC_L = (CRC & 0x00FF);
    CRC_H = (CRC >> 8) & 0x00FF;
    TxPacket[11] = CRC_L;
    TxPacket[12] = CRC_H;
    Serial1.write(TxPacket, 13);
    torgue_id = torgue_id + 1;
    delay(6);
  }
}

//syn motor on
void syn_torque_on(){
  uint16_t crc_accum = 0;
  uint16_t CRC;
  unsigned char CRC_L = 0;
  unsigned char CRC_H = 0;
  unsigned char TxPacket2[14 + 2 * 12];
  TxPacket2[0] = TxPacket2[1] = 0xFF;
  TxPacket2[2] = 0xFD;
  TxPacket2[3] = 0x00;
  TxPacket2[4] = 0xFE;
  TxPacket2[5] = 0x1F;  //4 + 2*12 + 3
  TxPacket2[6] = 0x00;
  TxPacket2[7] = 0x83;  //0x83
  TxPacket2[8] = 0x40;  //65 
  TxPacket2[9] = 0x00;  //0x00 p2
  TxPacket2[10] = 0x01; //1
  TxPacket2[11] = 0x00; //0x00 p4
  TxPacket2[12] = 0x01; 
  TxPacket2[14] = 0x02; 
  TxPacket2[16] = 0x03; 
  TxPacket2[18] = 0x04; 
  TxPacket2[20] = 0x05; 
  TxPacket2[22] = 0x06; 
  TxPacket2[24] = 0x07; 
  TxPacket2[26] = 0x08; 
  TxPacket2[28] = 0x09; 
  TxPacket2[30] = 0x0A; 
  TxPacket2[32] = 0x0B; 
  TxPacket2[34] = 0x0C; 
  TxPacket2[13]=TxPacket2[15]=TxPacket2[17]=TxPacket2[19]=TxPacket2[21]=TxPacket2[23]=TxPacket2[25]=TxPacket2[27]=TxPacket2[29]=TxPacket2[31]=TxPacket2[33]=TxPacket2[35]=0x01;
  CRC = update_crc(0, TxPacket2, 36);
  CRC_L = (CRC & 0x00FF);
  CRC_H = (CRC >> 8) & 0x00FF;
  TxPacket2[36] = CRC_L;
  TxPacket2[37] = CRC_H;
  Serial1.write(TxPacket2, 38);
}

void torgue_off() {
  int torgue_id = 0;
  for (int start_i = 1 ; start_i < 13 ; start_i++)
  {
    uint16_t crc_accum = 0;
    uint16_t CRC;
    unsigned char CRC_L = 0;
    unsigned char CRC_H = 0;
    uint16_t data_blk_size = 18;//size of TxPacket
    unsigned char TxPacket[14] = { 0xFF, 0xFF, 0xFD, 0x00, metor_id[torgue_id], 0x06, 0x00, 0x03, 0x40, 0x00, 0x00, CRC_L, CRC_H };
    CRC = update_crc(0, TxPacket, 11);
    CRC_L = (CRC & 0x00FF);
    CRC_H = (CRC >> 8) & 0x00FF;
    TxPacket[11] = CRC_L;
    TxPacket[12] = CRC_H;
    Serial1.write(TxPacket, 13);
    torgue_id = (torgue_id + 1);
    // Serial.print("torgue_on");
    delay(5);
  }
}

//syn motor off
void syn_torque_off(){
  uint16_t crc_accum = 0;
  uint16_t CRC;
  unsigned char CRC_L = 0;
  unsigned char CRC_H = 0;
  unsigned char TxPacket2[14 + 2 * 12];
  TxPacket2[0] = TxPacket2[1] = 0xFF;
  TxPacket2[2] = 0xFD;
  TxPacket2[3] = 0x00;
  TxPacket2[4] = 0xFE;
  TxPacket2[5] = 0x1F;  //4 + 2*12 + 3
  TxPacket2[6] = 0x00;
  TxPacket2[7] = 0x83;  //0x83
  TxPacket2[8] = 0x40;  //65 
  TxPacket2[9] = 0x00;  //0x00 p2
  TxPacket2[10] = 0x01; //1
  TxPacket2[11] = 0x00; //0x00 p4
  TxPacket2[12] = 0x01; 
  TxPacket2[14] = 0x02; 
  TxPacket2[16] = 0x03; 
  TxPacket2[18] = 0x04; 
  TxPacket2[20] = 0x05; 
  TxPacket2[22] = 0x06; 
  TxPacket2[24] = 0x07; 
  TxPacket2[26] = 0x08; 
  TxPacket2[28] = 0x09; 
  TxPacket2[30] = 0x0A; 
  TxPacket2[32] = 0x0B; 
  TxPacket2[34] = 0x0C; 
  TxPacket2[13]=TxPacket2[15]=TxPacket2[17]=TxPacket2[19]=TxPacket2[21]=TxPacket2[23]=TxPacket2[25]=TxPacket2[27]=TxPacket2[29]=TxPacket2[31]=TxPacket2[33]=TxPacket2[35]=0x00;
  CRC = update_crc(0, TxPacket2, 36);
  CRC_L = (CRC & 0x00FF);
  CRC_H = (CRC >> 8) & 0x00FF;
  TxPacket2[36] = CRC_L;
  TxPacket2[37] = CRC_H;
  Serial1.write(TxPacket2, 38);
}

void LED_on() {
  int torgue_id = 0;
  for (int start_i = 1 ; start_i < 13 ; start_i++) {
    uint16_t crc_accum = 0;
    uint16_t CRC;
    unsigned char CRC_L = 0;
    unsigned char CRC_H = 0;
    uint16_t data_blk_size = 18;//size of TxPacket
    //SEND LED ON COMMAND
    unsigned char TxPacket[13] = { 0xFF, 0xFF, 0xFD, 0x00, metor_id[torgue_id], 0x06, 0x00, 0x03, 0x41, 0x00, 0x01, CRC_L, CRC_H };
    CRC = update_crc(0, TxPacket, 11);
    CRC_L = (CRC & 0x00FF);
    CRC_H = (CRC >> 8) & 0x00FF;
    TxPacket[11] = CRC_L;
    TxPacket[12] = CRC_H;
    // Serial.print("CRC_L:  ");Serial.println(TxPacket[11]);
    // Serial.print("CRC_H:  ");Serial.println(TxPacket[12]);
    Serial1.write(TxPacket, 13);
    torgue_id = (torgue_id + 1);
    delay(100);
  }
}

void LED_off() {
  int torgue_id = 0;
  for (int start_i = 1 ; start_i < 13 ; start_i++)
  {
    uint16_t crc_accum = 0;
    uint16_t CRC;
    unsigned char CRC_L = 0;
    unsigned char CRC_H = 0;
    uint16_t data_blk_size = 18;//size of TxPacket
    //SEND LED ON COMMAND
    unsigned char TxPacket[13] = { 0xFF, 0xFF, 0xFD, 0x00, metor_id[torgue_id], 0x06, 0x00, 0x03, 0x41, 0x00, 0x00, CRC_L, CRC_H };
    CRC = update_crc(0, TxPacket, 11);
    CRC_L = (CRC & 0x00FF);
    CRC_H = (CRC >> 8) & 0x00FF;
    TxPacket[11] = CRC_L;
    TxPacket[12] = CRC_H;
    // Serial.print("CRC_L:  ");Serial.println(TxPacket[11]);
    // Serial.print("CRC_H:  ");Serial.println(TxPacket[12]);
    Serial1.write(TxPacket, 13);
    torgue_id = (torgue_id + 1);
    delay(100);
  }
}

void first_rotation(double ya, double za, double y, double z, double &newz, double &theta1) {
  newz = za - sqrt(za * za + y * y - 2 * y * ya + z * z - 2 * z * za);
  double theta_value = acos(((y - ya) * (-ya) + (z - za) * (newz - za)) / (sqrt((y - ya) * (y - ya) + (z - za) * (z - za)) * sqrt(ya * ya + (newz - za) * (newz - za)))) / M_PI * 180;
  if (isnan(theta_value)) {
    theta_value = 0.0;
  }
  if (y == 0.00) {
    theta1 = 0;
  } else if (y > 0) {
    theta1 = theta_value;
  } else {
    theta1 = -theta_value;
  }
}

void quad_IK_2D(double x, double y, double l1, double l2, double a1, double a2, double a3, double a4, double r, double angle1, double angle2, double angle3, double &theta1, double &theta2) {
  double x_act = x;
  double y_act = y + r;
  double d = sqrt(x_act * x_act + y_act * y_act);

  if (x_act < 0) {
    theta1 = (atan(y_act / x_act) - acos((l1 * l1 + d * d - l2 * l2) / (2 * l1 * d))) / M_PI * 180;
  } else if (x_act > 0) {
    theta1 = (atan(y_act / x_act) - acos((l1 * l1 + d * d - l2 * l2) / (2 * l1 * d))) / M_PI * 180 + 180;
  } else {
    theta1 = 90 - (acos((l1 * l1 + d * d - l2 * l2) / (2 * l1 * d))) / M_PI * 180;
  }

  double temp_theta2;
  if (x_act - l1 * cos((180 + theta1) / 180 * M_PI) < 0) {
    temp_theta2 = (atan((y_act - l1 * sin((180 + theta1) / 180 * M_PI)) / (x_act - l1 * cos((180 + theta1) / 180 * M_PI)))) / M_PI * 180;
  } else if (x_act - l1 * cos((180 + theta1) / 180 * M_PI) > 0) {
    temp_theta2 = (atan((y_act - l1 * sin((180 + theta1) / 180 * M_PI)) / (x_act - l1 * cos((180 + theta1) / 180 * M_PI)))) / M_PI * 180 + 180;
  } else {
    temp_theta2 = 90;
  }

  temp_theta2 = temp_theta2 + angle3 - angle2 + angle1;
  if (temp_theta2 < 180) {
    a5 = sqrt(a2 * a2 + a3 * a3 + -2 * a2 * a3 * cos(temp_theta2 / 180 * M_PI));
    temp_theta2 = acos((a5 * a5 + a3 * a3 - a2 * a2) / (2 * a5 * a3)) * 180 / M_PI + acos((a5 * a5 + a1 * a1 - a4 * a4) / (2 * a5 * a1)) * 180 / M_PI;
    theta2 = 180 - angle1 - temp_theta2;
  } else {
    a5 = sqrt(a2 * a2 + a3 * a3 + -2 * a2 * a3 * cos(temp_theta2 / 180 * M_PI));
    temp_theta2 = -acos((a5 * a5 + a3 * a3 - a2 * a2) / (2 * a5 * a3)) * 180 / M_PI + acos((a5 * a5 + a1 * a1 - a4 * a4) / (2 * a5 * a1)) * 180 / M_PI;
    theta2 = 180 - angle1 - temp_theta2;
  }
}

//trop steps
void servopos_t(int counter, int ite) {
  int dir;
  int cycle_delay;
  L = 0.03;//0.03; //0.035; //0.055
  for (i = 0; i < 4; i++) {
    if (i == 3 || i == 0) {
      cycle_delay = 0;
    } else {
      cycle_delay = ite / 2;
    }
    if ((ite + counter % ite - cycle_delay) % ite < ite * 1 / 2) {
      x = -L + (2.0 * ((ite + counter % ite - cycle_delay) % ite) * L / (1.0 / 2.0 * ite));
      y = 0;
      z = -38.88* x * x  - 0.065; //-27.78 * x * x  - 0.075; //-40.816 * x * x  - 0.05; //-200.0 / 9.0 * x * x - 0.08; // -9.99*x *x  -0.07
    } else {
      x = L - (2.0 * (((ite + counter % ite - cycle_delay) % ite) - 1.0 / 2.0 * ite) * L / (1.0 / 2.0 * ite));
      y = 0;
      z = -0.1;
    }

    first_rotation(y_axis, z_axis, y, z, newz, theta1[i]);
    quad_IK_2D(x, newz, l1, l2, a1, a2, a3, a4, r, angle1, angle2, angle3, theta2[i], theta3[i]);
    // if( i == 0){
    //   map_theta1[i] = -1 * theta1[i] + 65;
    //   map_theta2[i] = -1 * theta2[i] + 205;
    //   map_theta3[i] = -1 * theta3[i] + 90;
    // }
    // if( i == 1){
    //   map_theta1[i] = 1 * theta1[i] + 115;
    //   map_theta2[i] = 1 * theta2[i] - 25;
    //   map_theta3[i] = -1* theta3[i] + 90;
    // }
    // if( i == 2){
    //   map_theta1[i] = -1 * theta1[i] + 60;
    //   map_theta2[i] = -1 * theta2[i] + 210;
    //   map_theta3[i] = -1 * theta3[i] + 90;
    // }
    // if( i == 3){
    //   map_theta1[i] = 1 * theta1[i] + 120;
    //   map_theta2[i] = 1 * theta2[i] - 30;
    //   map_theta3[i] = -1 * theta3[i] + 90;
    // }
  }

  for (i = 0; i < 12; i++) {
    if (i < 8) {
      if (i == 0 || i == 2 || i == 4 || i == 6) {
        dir = -1; //1;
      } else {
        dir = 1 ; //-1;
      }
    }
    //  Serial.print("theta2[0]:  ");Serial.print(-theta2[0] +90); //+75
    //  Serial.print("  theta2[1]:  ");Serial.print(theta2[1] +90);//+105
    //  Serial.print("  theta2[2]:  ");Serial.print(-theta2[2] +60);//+75
    //  Serial.print("  theta2[3]:  ");Serial.println(theta2[3] +90);//+105
    // Serial.print("theta3[0]:  ");Serial.print(theta3[0]+30);
    // Serial.print("  theta3[1]:  ");Serial.print(-theta3[1]+150);
    // Serial.print("  theta3[2]:  ");Serial.println(-theta3[2]+210);
    // Serial.print("  theta3[3]:  ");Serial.println(-theta3[3]+150);
    if (i < 4) {
      //將大腿角度向下調整20度
      // Goal_position_walkortrop( i, 1*theta2[i%4] +110); // +110為兩邊大腿呈現90度
      if (i == 0 || i == 2)
      {
        if (i == 0)
        {
          Goal_position_walkortrop( i, dir * theta2[i % 4] + 65); //以90為基準向下減  85, 60, 50
        }
        if (i == 2)
        {
          Goal_position_walkortrop( i, dir * theta2[i % 4] + 65); //以90為基準向下減 85, 60, 55
        }
      }
      if (i == 1 || i == 3)
      {
        if (i == 1) {
          Goal_position_walkortrop( i, dir * theta2[i % 4] + 115); //以90為基準向上加  95, 120, 130
        }
        if (i == 3) {
          Goal_position_walkortrop( i, dir * theta2[i % 4] + 120); //以90為基準向上加  95, 120, 130 125
        }
      }
    } else if (i > 7) {
      Goal_position_walkortrop( i, -1 * theta1[i % 4] + 90); 
    } else {
      if (i == 4 || i == 6) {
        if (i == 4) {
          Goal_position_walkortrop( i, dir * theta3[i % 4] + 205 ); //以180為基準向上加
        }
        if (i == 6) {
          Goal_position_walkortrop( i, dir * theta3[i % 4] + 205); //以180為基準向上加
        }
      }
      else if (i == 5 || i == 7) {
        if (i == 5)
        {
          Goal_position_walkortrop( i, dir * theta3[i % 4] - 25 ); //以0為基準 向下減
        }
        if (i == 7)
        {
          Goal_position_walkortrop( i, dir * theta3[i % 4] - 30 ); //以0為基準 向下減
        }
      }
    }
  }
}

//0x43 syn_trop steps
void syn_servopos_t(int counter, int ite) {
  // int dir;
  ite = 150;
  int cycle_delay;
  L = 0.03 ;//0.025; //0.02;//0.025; //0.03; //0.035; //0.055
  if(change ==true){
    for (i = 0; i < 4; i++) {
      if (i == 3 || i == 0) {
        cycle_delay = 0;
      } else {
        cycle_delay = ite / 2;
      }
      if ((ite + counter % ite - cycle_delay) % ite < ite * 1 / 2) {
        x = -L + (2.0 * ((ite + counter % ite - cycle_delay) % ite) * L / (1.0 / 2.0 * ite));
        y = 0;
        z = -50 * x * x  - 0.055;//-64* x * x  - 0.06;  //-100* x * x  - 0.06; // -64* x * x  - 0.06;  //-27.78 * x * x  - 0.075; //-40.816 * x * x  - 0.05; //-200.0 / 9.0 * x * x - 0.08; // -9.99*x *x  -0.07
        // z = -88* x * x  - 0.06;
      } else {
        x = L - (2.0 * (((ite + counter % ite - cycle_delay) % ite) - 1.0 / 2.0 * ite) * L / (1.0 / 2.0 * ite));
        //x = L - (2.0 * (((ite + counter % ite - cycle_delay) % ite) - 1.0 / 2.0 * ite) * L / (1.0 / 2.0 * ite));
        y = 0;
        z = -0.1; //-0.12;
      }
      first_rotation(y_axis, z_axis, y, z, newz, theta1[i]);
      quad_IK_2D(x, newz, l1, l2, a1, a2, a3, a4, r, angle1, angle2, angle3, theta2[i], theta3[i]);
      
      if( i == 0){
        map_theta2[i] = -1 * theta2[i] + 50;//大腿 以90為基準向下減   //35
        map_theta3[i] = -1 * theta3[i] + 190;//小腿 以180為基準向上加  //10
        map_theta1[i] = 1 * theta1[i] + 90;
      }
      if( i == 1){
        map_theta2[i] = 1 * theta2[i] + 130;//大腿 以90為基準向上加 //35
        map_theta3[i] = 1 * theta3[i] - 10;//小腿 以0為基準 向下減  //10
        map_theta1[i] = 1* theta1[i] + 90;
      }
      if( i == 2 ){
        map_theta2[i] = -1 * theta2[i] + 58;//大腿 以90為基準向下減   //35
        map_theta3[i] = -1 * theta3[i] + 190;//小腿 以180為基準向上加   //10
        map_theta1[i] = -1 * theta1[i] + 90;
      }
      if( i == 3){
        map_theta2[i] = 1 * theta2[i] + 122;//大腿 以90為基準向上加   //35
        map_theta3[i] = 1 * theta3[i] - 10;//小腿 以0為基準 向下減   //10
        map_theta1[i] = -1 * theta1[i] + 90;
      }
    }
    change = false;
  }
}

//0x43 syn_trop_left
void syn_servopos_trop_left(int counter, int ite) {
  // int dir;
  int cycle_delay;
  L = 0.02;//0.025; //0.03; //0.035; //0.055
  if(change ==true){
    for (i = 0; i < 4; i++) {
      if (i == 3 || i == 0) {
        cycle_delay = 0;
      } else {
        cycle_delay = ite / 2;
      }
      if ((ite + counter % ite - cycle_delay) % ite < ite * 1 / 2) {
        x = 0;
        y = -L + (2.0 * ((ite + counter % ite - cycle_delay) % ite) * L / (1.0 / 2.0 * ite));
        z = -62.5* y * y  - 0.075; // -64* x * x  - 0.06;  //-27.78 * x * x  - 0.075; //-40.816 * x * x  - 0.05; //-200.0 / 9.0 * x * x - 0.08; // -9.99*x *x  -0.07
      } else {
        x = 0;
        y = L - (2.0 * (((ite + counter % ite - cycle_delay) % ite) - 1.0 / 2.0 * ite) * L / (1.0 / 2.0 * ite));
        z = -0.1;
      }
      first_rotation(y_axis, z_axis, y, z, newz, theta1[i]);
      quad_IK_2D(x, newz, l1, l2, a1, a2, a3, a4, r, angle1, angle2, angle3, theta2[i], theta3[i]);
      
      if( i == 0){
        map_theta2[i] = -1 * theta2[i] + 55;//大腿 以90為基準向下減   //35
        map_theta3[i] = -1 * theta3[i] + 190;//小腿 以180為基準向上加  //10
        map_theta1[i] = 1 * theta1[i] + 90;
      }
      if( i == 1){
        map_theta2[i] = 1 * theta2[i] + 125;//大腿 以90為基準向上加 //35
        map_theta3[i] = 1 * theta3[i] - 10;//小腿 以0為基準 向下減  //10
        map_theta1[i] = 1* theta1[i] + 90;
      }
      if( i == 2 ){
        map_theta2[i] = -1 * theta2[i] + 55;//大腿 以90為基準向下減   //35
        map_theta3[i] = -1 * theta3[i] + 190;//小腿 以180為基準向上加   //10
        map_theta1[i] = -1 * theta1[i] + 90;
      }
      if( i == 3){
        map_theta2[i] = 1 * theta2[i] + 125;//大腿 以90為基準向上加   //35
        map_theta3[i] = 1 * theta3[i] - 10;//小腿 以0為基準 向下減   //10
        map_theta1[i] = -1 * theta1[i] + 90;
      }
      // Serial.print("theta2[0]:  ");Serial.print(map_theta2[0]); //+75
      // Serial.print("  theta2[1]:  ");Serial.print(theta2[1] );//+105
      // Serial.print("  theta2[2]:  ");Serial.print(theta2[2] );//+75
      // Serial.print("  theta2[3]:  ");Serial.print(-theta2[3] );//+105
      // Serial.print("        theta3[0]:  ");Serial.println(map_theta3[0]);
      // Serial.print("  theta3[1]:  ");Serial.println(theta3[1]);
      // Serial.print("  theta3[2]:  ");Serial.print(theta3[2]);
      // Serial.print("  theta3[3]:  ");Serial.println(-theta3[3]+180);
    }
    change = false;
  }
}

//0x43 syn_write
void synwrite_Goal_position_walkortrop() {
  uint16_t crc_accum = 0;
  uint16_t CRC;
  unsigned char CRC_L = 0;
  unsigned char CRC_H = 0;
  unsigned char TxPacket2[14 + 5*12];

  TxPacket2[0] = TxPacket2[1] = 0xFF;
  TxPacket2[2] = 0xFD;
  TxPacket2[3] = 0x00;
  TxPacket2[4] = 0xFE;
  TxPacket2[5] = 0x43;  //4 + 5*12 + 3
  TxPacket2[6] = 0x00;  
  TxPacket2[7] = 0x83;  //0x83
  TxPacket2[8] = 0x74;  //0x74 p1
  TxPacket2[9] = 0x00;  //0x00 p2
  TxPacket2[10] = 0x04; //0x04 p3
  TxPacket2[11] = 0x00; //0x00 p4
  TxPacket2[15] = TxPacket2[16] = TxPacket2[20] = TxPacket2[21] = TxPacket2[25] = TxPacket2[26] = TxPacket2[30] = TxPacket2[31] = TxPacket2[35] = TxPacket2[36] =TxPacket2[40] =TxPacket2[41] = TxPacket2[45] = TxPacket2[46] =  TxPacket2[50] = TxPacket2[51] =TxPacket2[55] =  TxPacket2[56] = TxPacket2[60] = TxPacket2[61] = TxPacket2[65] = TxPacket2[66] = TxPacket2[70] = TxPacket2[71]  = 0x00; //0x00 p4
  //---MOtor ID:1----//
  TxPacket2[12] = 0x01; //ID:0x01 
  TxPacket2[13] = ((map_theta2[0] - rool_b1 + pitchb) * 4096 /360 ) & (0x00FF); //pos_low
  TxPacket2[14] = (((map_theta2[0] - rool_b1 + pitchb)  * 4096 / 360 ) >> 8) & (0x00FF); //pos_high
  //---MOtor ID:2----//
  TxPacket2[17] = 0x02; //ID:0x02 
  TxPacket2[18] = ((map_theta2[1] - rool_f1 - pitchb) * 4096 /360 ) & 0x00FF; //pos_low
  TxPacket2[19] = (((map_theta2[1] - rool_f1 - pitchb)* 4096 /360 ) >> 8) & 0x00FF; //pos_high
  //---MOtor ID:3----//
  TxPacket2[22] = 0x03; //ID:0x03
  TxPacket2[23] = ((map_theta2[2] - rool_b1 - pitchf) * 4096 /360 ) & 0x00FF; //pos_low
  TxPacket2[24] = (((map_theta2[2] - rool_b1 - pitchf) * 4096 /360 ) >> 8) & 0x00FF; //pos_high
  //---MOtor ID:4----//
  TxPacket2[27] = 0x04; //ID:0x04 
  TxPacket2[28] = ((map_theta2[3] - rool_f1 + pitchf) * 4096 /360  ) & 0x00FF; //pos_low
  TxPacket2[29] = (((map_theta2[3] - rool_f1 + pitchf) * 4096 /360 ) >> 8) & 0x00FF; //pos_high
  //---MOtor ID:5----//
  TxPacket2[32] = 0x05; //ID:0x05
  TxPacket2[33] = (map_theta3[0] * 4096 /360 ) & 0x00FF; //pos_low
  TxPacket2[34] = ((map_theta3[0]* 4096 /360 ) >> 8) & 0x00FF; //pos_high
  //---MOtor ID:6----//
  TxPacket2[37] = 0x06; //ID:0x06
  TxPacket2[38] = (map_theta3[1]* 4096 /360) & 0x00FF; //pos_low
  TxPacket2[39] = ((map_theta3[1]* 4096 /360) >> 8) & 0x00FF; //pos_high
  //---MOtor ID:7----//
  TxPacket2[42] = 0x07; //ID:0x07 
  TxPacket2[43] = (map_theta3[2]* 4096 /360) & 0x00FF; //pos_low
  TxPacket2[44] = ((map_theta3[2]* 4096 /360) >> 8 ) & 0x00FF; //pos_high
  //---MOtor ID:8----//
  TxPacket2[47] = 0x08; //ID:0x08 
  TxPacket2[48] = (map_theta3[3]* 4096 /360) & 0x00FF; //pos_low
  TxPacket2[49] = ((map_theta3[3]* 4096 /360) >> 8) & 0x00FF; //pos_high
  //---MOtor ID:9----//
  TxPacket2[52] = 0x09; //ID:0x09 
  TxPacket2[53] = ((map_theta1[0] - rool_f)* 4096 /360) & 0x00FF; //pos_low
  TxPacket2[54] = (((map_theta1[0] - rool_f)* 4096 /360) >> 8) & 0x00FF; //pos_high
  //---MOtor ID:10----//
  TxPacket2[57] = 0x0A; //ID:0x010
  TxPacket2[58] = ((map_theta1[1] - rool_b)* 4096 /360) & 0x00FF; //pos_low
  TxPacket2[59] = (((map_theta1[1]-rool_b)* 4096 /360) >> 8) & 0x00FF; //pos_high
  //---MOtor ID:11----//
  TxPacket2[62] = 0x0B; //ID:0x011 
  TxPacket2[63] = ((map_theta1[2]+rool_f)* 4096 /360) & 0x00FF; //pos_low
  TxPacket2[64] = (((map_theta1[2]+rool_f)* 4096 /360) >> 8) & 0x00FF; //pos_high
  //---MOtor ID:12----//
  TxPacket2[67] = 0x0C; //ID:0x012 
  TxPacket2[68] = ((map_theta1[3]+rool_b)* 4096 /360) & 0x00FF; //pos_low
  TxPacket2[69] = (((map_theta1[3] + rool_b)* 4096 /360) >> 8) & 0x00FF; //pos_high

  CRC = update_crc(0, TxPacket2, 72);
  CRC_L = (CRC & 0x00FF);
  CRC_H = (CRC >> 8) & 0x00FF;
  TxPacket2[72] = CRC_L;
  TxPacket2[73] = CRC_H;
  Serial1.write(TxPacket2, 74);
}

//0x43 syn_walk steps
void syn_servopos_w(int counter, int ite) {
  // int dir;
  ite = 300; //test 800
  int cycle_delay;
  L = 0.025; //0.03; //0.02;//0.025; //0.03; //0.035; //0.055
  if(change ==true){
    for (i = 0; i < 4; i++) {
      if (i == 0 ) {
        cycle_delay = 0;
      } else if (i == 1) {
        cycle_delay = ((2 * ite) / 4);
      } else if (i == 2) {
        cycle_delay = ((3 * ite) / 4);
      } else if (i == 3) {
        cycle_delay = ((1 * ite) / 4);
      }
      if (((ite + (counter % ite) - cycle_delay) % ite) < ite * 0.25) {
        x = -L + (2.0 * ((ite + (counter % ite) - cycle_delay) % ite) * L / ( ite * 0.25 )); //i * ite / 4
        y = 0;
        z = -64* x * x  - 0.06;  //-38.88* x * x  - 0.065; //-15.625 * x * x  - 0.075; //-200.0 / 9.0 * x * x - 0.08; // -9.99*x *x  -0.07
      } else {
        x = L - (2.0 * (((ite + (counter % ite) - cycle_delay) % ite) -  ite * 0.25 ) * L / ( ite * 0.75)); //i * ite / 4
        y = 0;
        z = -0.1;
      }
      first_rotation(y_axis, z_axis, y, z, newz, theta1[i]);
      quad_IK_2D(x, newz, l1, l2, a1, a2, a3, a4, r, angle1, angle2, angle3, theta2[i], theta3[i]);
      
      if( i == 0){
        map_theta2[i] = -1 * theta2[i] + 55;//大腿 以90為基準向下減   //35
        map_theta3[i] = -1 * theta3[i] + 190;//小腿 以180為基準向上加  //10
        map_theta1[i] = 1 * theta1[i] + 90;
      }
      if( i == 1){
        map_theta2[i] = 1 * theta2[i] + 125;//大腿 以90為基準向上加 //35
        map_theta3[i] = 1 * theta3[i] - 10;//小腿 以0為基準 向下減  //10
        map_theta1[i] = 1* theta1[i] + 90;
      }
      if( i == 2 ){
        map_theta2[i] = -1 * theta2[i] + 55;//大腿 以90為基準向下減   //35
        map_theta3[i] = -1 * theta3[i] + 190;//小腿 以180為基準向上加   //10
        map_theta1[i] = -1 * theta1[i] + 90;
      }
      if( i == 3){
        map_theta2[i] = 1 * theta2[i] + 125;//大腿 以90為基準向上加   //35
        map_theta3[i] = 1 * theta3[i] - 10;//小腿 以0為基準 向下減   //10
        map_theta1[i] = -1 * theta1[i] + 90;
      }
    }
    change = false;
  }
}

//0x03 write walk step
void servopos_w(int counter, int ite) {
  int dir;
  int cycle_delay;
  L = 0.03;// 0.04; //0.055
  for (i = 0; i < 4; i++) {
    if (i == 0 ) {
      cycle_delay = 0;
    } else if (i == 1) {
      cycle_delay = ((2 * ite) / 4);
    } else if (i == 2) {
      cycle_delay = ((3 * ite) / 4);
    } else if (i == 3) {
      cycle_delay = ((1 * ite) / 4);
    }
    if (((ite + (counter % ite) - cycle_delay) % ite) < ite * 0.25) {
      x = -L + (2.0 * ((ite + (counter % ite) - cycle_delay) % ite) * L / ( ite * 0.25 )); //i * ite / 4
      y = 0;
      z = -38.88* x * x  - 0.065; //-15.625 * x * x  - 0.075; //-200.0 / 9.0 * x * x - 0.08; // -9.99*x *x  -0.07
    } else {
      x = L - (2.0 * (((ite + (counter % ite) - cycle_delay) % ite) -  ite * 0.25 ) * L / ( ite * 0.75)); //i * ite / 4
      y = 0;
      z = -0.1;
    }
    first_rotation(y_axis, z_axis, y, z, newz, theta1[i]);
    quad_IK_2D(x, newz, l1, l2, a1, a2, a3, a4, r, angle1, angle2, angle3, theta2[i], theta3[i]);
  }

  for (i = 0; i < 12; i++) {
    if (i < 8)
    {
      if (i == 0 || i == 2 || i == 4 || i == 6) {
        dir = -1;
      } else {
        dir = 1;
      }
    }
    //  Serial.print("theta2[0]:  ");Serial.print(theta2[0] ); //+75
    //  Serial.print("  theta2[1]:  ");Serial.print(theta2[1] );//+105
    //  Serial.print("  theta2[2]:  ");Serial.print(theta2[2] );//+75
    //  Serial.print("  theta2[3]:  ");Serial.print(-theta2[3] );//+105

    // Serial.print("theta3[0]:  ");Serial.println(theta3[0]);
    // Serial.print("  theta3[1]:  ");Serial.println(theta3[1]);
    // Serial.print("  theta3[2]:  ");Serial.print(theta3[2]);
    // Serial.print("  theta3[3]:  ");Serial.println(-theta3[3]+180);
    if (i < 4) {
      //將大腿角度向下調整20度
      // Goal_position_walkortrop( i, 1*theta2[i%4] +110); // +110為兩邊大腿呈現90度
      if (i == 0 || i == 2)
      {
        if (i == 0)
        {
          Goal_position_walkortrop( i, dir * theta2[i % 4] + 65); //以90為基準向下減  85, 60, 50
        }
        if (i == 2)
        {
          Goal_position_walkortrop( i, dir * theta2[i % 4] + 60); //以90為基準向下減 85, 60, 55
        }
      }
      if (i == 1 || i == 3)
      {
        if (i == 1) {
          Goal_position_walkortrop( i, dir * theta2[i % 4] + 115); //以90為基準向上加  95, 120, 130
        }
        if (i == 3) {
          Goal_position_walkortrop( i, dir * theta2[i % 4] + 120); //以90為基準向上加  95, 120, 130 125
        }
      }
    } else if (i > 7) {
      Goal_position_walkortrop( i, -1 * theta1[i % 4] + 90);
    } else {
      if (i == 4 || i == 6) {
        if (i == 4) {
          Goal_position_walkortrop( i, dir * theta3[i % 4] + 205 ); //以180為基準向上加
        }
        if (i == 6) {
          Goal_position_walkortrop( i, dir * theta3[i % 4] + 210 ); //以180為基準向上加
        }
      }
      else if (i == 5 || i == 7) {
        if (i == 5)
        {
          Goal_position_walkortrop( i, dir * theta3[i % 4] - 25 ); //以0為基準 向下減
        }
        if (i == 7)
        {
          Goal_position_walkortrop( i, dir * theta3[i % 4] - 30 ); //以0為基準 向下減
        }
      }
    }
  }
}

//0x03 write small_jump
void servopos_smalljump() {
  C_theta_stand_ud = acos((pow(l1, 2) + pow(l2, 2) - pow(c, 2)) / (2 * l1 * l2));
  c_changetheta_stand_ud = ( (C_theta_stand_ud / PI * 180) - 44.7); //44.7為目前夾腳
  int ii = 3;
  for (int start_i = 3 ; start_i >= 0 ; start_i--)
  {
    uint16_t crc_accum = 0;
    uint16_t CRC;
    unsigned char CRC_L = 0;
    unsigned char CRC_H = 0;
    uint16_t data_blk_size = 18; //size of TxPacket
    pos = c_changetheta_stand_ud;
    if (start_i == 0 ) {
      pos = 90 - round(c_changetheta_stand_ud) + 25 ; //
    }
    else if (start_i == 2) {
      pos = 90 - round(c_changetheta_stand_ud);
    }
    else if (start_i == 1) {
      pos = 90 + round(c_changetheta_stand_ud) - 25;
    }
    else if (start_i == 3) {
      pos = 90 + round(c_changetheta_stand_ud);
    }
    // Serial.print(ii); Serial.print("   "); Serial.println(pos);
    map_pos();
    unsigned char TxPacket2[18] = { 0xFF, 0xFF, 0xFD, 0x00, metor_id[ii], 0x09, 0x00, 0x03, 0x74, 0x00,  pos_low, pos_high, 0x00, 0x00, CRC_L, CRC_H };
    CRC = update_crc(0, TxPacket2, 14);
    CRC_L = (CRC & 0x00FF);
    CRC_H = (CRC >> 8) & 0x00FF;
    TxPacket2[14] = CRC_L;
    TxPacket2[15] = CRC_H;
    Serial1.write(TxPacket2, 16);
    ii = ii - 1;
    delay(5);
  }
}

//left_right
void servopos_r(int counter, int ite) {
  int dir;
  double dist1 = sqrt((half_width - half_width * cos(M_PI / 24.0 * sin(M_PI * 2.0 / ite * counter))) * (half_width - half_width * cos(M_PI / 24.0 * sin(M_PI * 2.0 / ite * counter))) + (0.1 + half_width * sin(M_PI / 24.0 * sin(M_PI * 2.0 / ite * counter))) * (0.1 + half_width * sin(M_PI / 24.0 * sin(M_PI * 2.0 / ite * counter))));
  double dist2 = sqrt((half_width - half_width * cos(M_PI / 24.0 * sin(M_PI * 2.0 / ite * counter))) * (half_width - half_width * cos(M_PI / 24.0 * sin(M_PI * 2.0 / ite * counter))) + (0.1 - half_width * sin(M_PI / 24.0 * sin(M_PI * 2.0 / ite * counter))) * (0.1 - half_width * sin(M_PI / 24.0 * sin(M_PI * 2.0 / ite * counter))));
  for (i = 0; i < 4; i++) {
    //if (i == 0 || i == 2)
    if (i == 3 || i == 2) {
      y = -sin(M_PI / 35.0 * sin(M_PI * 2.0 / ite * counter)) * dist1;
      x = 0;
      z = -cos(M_PI / 35.0 * sin(M_PI * 2.0 / ite * counter)) * dist1;
    } else {
      y = sin(M_PI / 35.0 * sin(M_PI * 2.0 / ite * counter)) * dist2;
      x = 0;
      z = -cos(M_PI / 35.0 * sin(M_PI * 2.0 / ite * counter)) * dist2;
    }
    first_rotation(y_axis, z_axis, y, z, newz, theta1[i]);
    quad_IK_2D(x, newz, l1, l2, a1, a2, a3, a4, r, angle1, angle2, angle3, theta2[i], theta3[i]);
  }
  for (i = 0; i < 12; i++) {
    // Serial.print("theta1[0]:  ");Serial.print(theta1[0] );
    // Serial.print("  theta1[1]:  ");Serial.print(theta1[1] );
    // Serial.print("  theta1[2]:  ");Serial.print(theta1[2] );
    // Serial.print("  theta1[3]:  ");Serial.println(theta1[3]);

    // Serial.print("theta2[0]:  "); Serial.print(theta2[0] + 70);
    // Serial.print("  theta2[1]:  "); Serial.print(theta2[2] + 70);
    // Serial.print("  theta2[2]:  "); Serial.print(theta2[1] + 70);
    // Serial.print("  theta2[3]:  "); Serial.println(theta2[3] + 70);

    // if (i%2 == 0){
    //   dir = -1;
    // }else{
    //   dir = 1;
    // }

    if (i < 4) {
      //將大腿角度向下調整20度
      // Goal_position_walkortrop( i, 1*theta2[i%4] +110); // +110為兩邊大腿呈現90度
      if (i == 0 )
      {
        Goal_position_walkortrop( i, 1 * theta2[i % 4] + 70); //dir
      }
      if (i == 2) {
        Goal_position_walkortrop( i - 1, 1 * theta2[i % 4] + 70); //dir
      }
      if (i == 1 ) {
        Goal_position_walkortrop( i + 1, 1 * theta2[i % 4] + 70); //dir
      }
      if (i == 3 )
      {
        Goal_position_walkortrop( i, 1 * theta2[i % 4] + 70); //dir
      }
    } else if (i > 7) {
      if (i == 8) {
        Goal_position_walkortrop( i, 1 * theta1[i % 4] + 90);
      }
      if (i == 9) {
        Goal_position_walkortrop( i, 1 * theta1[i % 4] + 90);
      }
      if (i == 10) {
        Goal_position_walkortrop( i, 1 * theta1[i % 4] + 90);
      }
      if (i == 11) {
        Goal_position_walkortrop( i, 1 * theta1[i % 4] + 90);
      }
    } else {
      // Goal_position_walkortrop( i, 1 * theta3[i % 4]); //dir
    }
  }
}

//up_down
void servopos_stand_up_down(int counter, int ite) {
  int dir;
  double dist1 = sqrt((half_length - half_length * cos(M_PI / 10.0 * sin(M_PI * 2.0 / ite * counter))) * (half_length - half_length * cos(M_PI / 10.0 * sin(M_PI * 2.0 / ite * counter))) + (0.1 + half_length * sin(M_PI / 10.0 * sin(M_PI * 2.0 / ite * counter))) * (0.1 + half_length * sin(M_PI / 10.0 * sin(M_PI * 2.0 / ite * counter))));
  double dist2 = sqrt((half_length - half_length * cos(M_PI / 10.0 * sin(M_PI * 2.0 / ite * counter))) * (half_length - half_length * cos(M_PI / 10.0 * sin(M_PI * 2.0 / ite * counter))) + (0.1 - half_length * sin(M_PI / 10.0 * sin(M_PI * 2.0 / ite * counter))) * (0.1 - half_length * sin(M_PI / 10.0 * sin(M_PI * 2.0 / ite * counter))));
  for (i = 0; i < 4; i++) {
    if (i == 0 || i == 1) {
      x = -sin(M_PI / 10.0 * sin(M_PI * 2.0 / ite * counter)) * dist1;
      y = 0;
      z = -cos(M_PI / 10.0 * sin(M_PI * 2.0 / ite * counter)) * dist1;
    } else {
      x = -sin(M_PI / 10.0 * sin(M_PI * 2.0 / ite * counter)) * dist2;
      y = 0;
      z = -cos(M_PI / 10.0 * sin(M_PI * 2.0 / ite * counter)) * dist2;
    }


    first_rotation(y_axis, z_axis, y, z, newz, theta1[i]);
    quad_IK_2D(x, newz, l1, l2, a1, a2, a3, a4, r, angle1, angle2, angle3, theta2[i], theta3[i]);
  }

  for (i = 0; i < 12; i++) {
    if (i < 8)
    {
      if (i == 0 || i == 2 || i == 4 || i == 6) {
        dir = 1;
      } else {
        dir = -1;
      }
    }
    // Serial.print("theta2[0]:  ");Serial.print(theta2[0]+45 ); //+75
    // Serial.print("  theta2[1]:  ");Serial.print(-theta2[1]+125);//+105
    // Serial.print("  theta2[2]:  ");Serial.print(theta2[2]+45);//+75
    // Serial.print("  theta2[3]:  ");Serial.println(-theta2[3]+135);//+105

    // Serial.print("theta3[0]:  ");Serial.print(theta3[0]);
    // Serial.print("  theta3[1]:  ");Serial.print(-theta3[1]+180);
    // Serial.print("  theta3[2]:  ");Serial.print(theta3[2]);
    // Serial.print("  theta3[3]:  ");Serial.println(-theta3[3]+180);
    if (i < 4) {
      //將大腿角度向下調整20度
      // Goal_position_walkortrop( i, 1*theta2[i%4] +110); // +110為兩邊大腿呈現90度
      if (i == 0 || i == 2)
      {
        if ( i == 0) {
          Goal_position_walkortrop( i, dir * theta2[i % 4] + 45); //85
        }
        if (i == 2) {
          Goal_position_walkortrop( i, dir * theta2[i % 4] + 45); //85
        }
      }
      if (i == 1 || i == 3)
      {
        if ( i == 1) {
          Goal_position_walkortrop( i, dir * theta2[i % 4] + 125); //85
        }
        if (i == 3) {
          Goal_position_walkortrop( i, dir * theta2[i % 4] + 135); //85
        }
      }
    } else if (i > 7) {
      if (i == 8) {
        Goal_position_walkortrop( i, -1 * theta1[i % 4] + 90);
      }
      if (i == 9) {
        Goal_position_walkortrop( i, -1 * theta1[i % 4] + 90);
      }
      if (i == 10) {
        Goal_position_walkortrop( i, -1 * theta1[i % 4] + 90);
      }
      if (i == 11) {
        Goal_position_walkortrop( i, -1 * theta1[i % 4] + 90);
      }
    } else {
      if (i == 4 || i == 6) {
        Goal_position_walkortrop( i, dir * theta3[i % 4] + 0); //以0為基準向上加
      }
      else if (i == 5 || i == 7) {
        Goal_position_walkortrop( i, dir * theta3[i % 4] + 180 ); //以180為基準向下減
      }
    }
  }
}


void Goal_position_toZero(double rorate) {
uint16_t crc_accum = 0;
  uint16_t CRC;
  unsigned char CRC_L = 0;
  unsigned char CRC_H = 0;
  unsigned char TxPacket2[14 + 5*12];
  pos = rorate;
  TxPacket2[0] = TxPacket2[1] = 0xFF;
  TxPacket2[2] = 0xFD;
  TxPacket2[3] = 0x00;
  TxPacket2[4] = 0xFE;
  TxPacket2[5] = 0x43;  //4 + 5*12 + 3
  TxPacket2[6] = 0x00;  
  TxPacket2[7] = 0x83;  //0x83
  TxPacket2[8] = 0x74;  //0x74 p1
  TxPacket2[9] = 0x00;  //0x00 p2
  TxPacket2[10] = 0x04; //0x04 p3
  TxPacket2[11] = 0x00; //0x00 p4
  TxPacket2[15] = TxPacket2[16] = TxPacket2[20] = TxPacket2[21] = TxPacket2[25] = TxPacket2[26] = TxPacket2[30] = TxPacket2[31] = TxPacket2[35] = TxPacket2[36] =TxPacket2[40] =TxPacket2[41] = TxPacket2[45] = TxPacket2[46] =  TxPacket2[50] = TxPacket2[51] =TxPacket2[55] =  TxPacket2[56] = TxPacket2[60] = TxPacket2[61] = TxPacket2[65] = TxPacket2[66] = TxPacket2[70] = TxPacket2[71]  = 0x00; //0x00 p4
  //---MOtor ID:1----//
  TxPacket2[12] = 0x01; //ID:0x01 
  TxPacket2[13] = ((pos - 30) * 4096 /360 ) & (0x00FF); //pos_low
  TxPacket2[14] = (((pos - 30)  * 4096 / 360 ) >> 8) & (0x00FF); //pos_high
  //---MOtor ID:2----//
  TxPacket2[17] = 0x02; //ID:0x02 
  TxPacket2[18] = ((pos + 30) * 4096 /360 ) & 0x00FF; //pos_low
  TxPacket2[19] = (((pos + 30) * 4096 /360 ) >> 8) & 0x00FF; //pos_high
  //---MOtor ID:3----//
  TxPacket2[22] = 0x03; //ID:0x03
  TxPacket2[23] = ((pos-30) * 4096 /360 ) & 0x00FF; //pos_low
  TxPacket2[24] = (((pos-30) * 4096 /360 ) >> 8) & 0x00FF; //pos_high
  //---MOtor ID:4----//
  TxPacket2[27] = 0x04; //ID:0x04 
  TxPacket2[28] = ((pos+30) * 4096 /360  ) & 0x00FF; //pos_low
  TxPacket2[29] = (((pos+30) * 4096 /360 ) >> 8) & 0x00FF; //pos_high
  //---MOtor ID:5----//
  TxPacket2[32] = 0x05; //ID:0x05
  TxPacket2[33] = ((pos+20) * 4096 /360 ) & 0x00FF; //pos_low
  TxPacket2[34] = (((pos+20)* 4096 /360 ) >> 8) & 0x00FF; //pos_high
  //---MOtor ID:6----//
  TxPacket2[37] = 0x06; //ID:0x06
  TxPacket2[38] = ((pos-20)* 4096 /360) & 0x00FF; //pos_low
  TxPacket2[39] = (((pos-20)* 4096 /360) >> 8) & 0x00FF; //pos_high
  //---MOtor ID:7----//
  TxPacket2[42] = 0x07; //ID:0x07 
  TxPacket2[43] = ((pos+20)* 4096 /360) & 0x00FF; //pos_low
  TxPacket2[44] = (((pos+20)* 4096 /360) >> 8 ) & 0x00FF; //pos_high
  //---MOtor ID:8----//
  TxPacket2[47] = 0x08; //ID:0x08 
  TxPacket2[48] = ((pos-20)* 4096 /360) & 0x00FF; //pos_low
  TxPacket2[49] = (((pos-20)* 4096 /360) >> 8) & 0x00FF; //pos_high
  //---MOtor ID:9----//
  TxPacket2[52] = 0x09; //ID:0x09 
  TxPacket2[53] = ((pos+0)* 4096 /360) & 0x00FF; //pos_low
  TxPacket2[54] = (((pos+0)* 4096 /360) >> 8) & 0x00FF; //pos_high
  //---MOtor ID:10----//
  TxPacket2[57] = 0x0A; //ID:0x010
  TxPacket2[58] = (pos* 4096 /360) & 0x00FF; //pos_low
  TxPacket2[59] = ((pos* 4096 /360) >> 8) & 0x00FF; //pos_high
  //---MOtor ID:11----//
  TxPacket2[62] = 0x0B; //ID:0x011 
  TxPacket2[63] = (pos* 4096 /360) & 0x00FF; //pos_low
  TxPacket2[64] = ((pos* 4096 /360) >> 8) & 0x00FF; //pos_high
  //---MOtor ID:12----//
  TxPacket2[67] = 0x0C; //ID:0x012 
  TxPacket2[68] = ((pos+0)* 4096 /360) & 0x00FF; //pos_low
  TxPacket2[69] = (((pos+0)* 4096 /360) >> 8) & 0x00FF; //pos_high

  CRC = update_crc(0, TxPacket2, 72);
  CRC_L = (CRC & 0x00FF);
  CRC_H = (CRC >> 8) & 0x00FF;
  TxPacket2[72] = CRC_L;
  TxPacket2[73] = CRC_H;
  Serial1.write(TxPacket2, 74);
}

//call this function to change every step
void Goal_position_walkortrop(int id , double rorate) {
  uint16_t crc_accum = 0;
  uint16_t CRC;
  unsigned char CRC_L = 0;
  unsigned char CRC_H = 0;
  uint16_t data_blk_size = 18;//size of TxPacket
  pos = rorate;
  map_pos();
  unsigned char TxPacket2[16] = { 0xFF, 0xFF, 0xFD, 0x00, metor_id[id], 0x09, 0x00, 0x03, 0x74, 0x00,  pos_low, pos_high, 0x00, 0x00, CRC_L, CRC_H };
  CRC = update_crc(0, TxPacket2, 14);
  CRC_L = (CRC & 0x00FF);
  CRC_H = (CRC >> 8) & 0x00FF;
  TxPacket2[14] = CRC_L;
  TxPacket2[15] = CRC_H;
  Serial1.write(TxPacket2, 16);
  delay(2);
}

void synwrite_Goal_position_walkortrop(double rorate) {
  uint16_t crc_accum = 0;
  uint16_t CRC;
  unsigned char CRC_L = 0;
  unsigned char CRC_H = 0;
  // pos = map(rorate, 0, 360, 0, 4096);
  // Serial.print("Tpos");Serial.println(pos);
  // map_pos();
  pos = rorate;
  unsigned char TxPacket2[14 + 5*12];

  TxPacket2[0] = TxPacket2[1] = 0xFF;
  TxPacket2[2] = 0xFD;
  TxPacket2[3] = 0x00;
  TxPacket2[4] = 0xFE;
  TxPacket2[5] = 0x43;  //4 + 5*12 + 3
  TxPacket2[6] = 0x00;  
  TxPacket2[7] = 0x83;  //0x83
  TxPacket2[8] = 0x74;  //0x74 p1
  TxPacket2[9] = 0x00;  //0x00 p2
  TxPacket2[10] = 0x04; //0x04 p3
  TxPacket2[11] = 0x00; //0x00 p4
  TxPacket2[15] = TxPacket2[16] = TxPacket2[20] = TxPacket2[21] = TxPacket2[25] = TxPacket2[26] = TxPacket2[30] = TxPacket2[31] = TxPacket2[35] = TxPacket2[36] =TxPacket2[40] =TxPacket2[41] = TxPacket2[45] = TxPacket2[46] =  TxPacket2[50] = TxPacket2[51] =TxPacket2[55] =  TxPacket2[56] = TxPacket2[60] = TxPacket2[61] = TxPacket2[65] = TxPacket2[66] = TxPacket2[70] = TxPacket2[71]  = 0x00; //0x00 p4
  //---MOtor ID:1----//
  TxPacket2[12] = 0x01; //ID:0x01 
  TxPacket2[13] = (pos * 4096 /360 ) & (0x00FF); //pos_low
  TxPacket2[14] = ((pos * 4096 / 360 ) >> 8) & (0x00FF); //pos_high
  // Serial.print("TxPacket2[13]");Serial.println(TxPacket2[13]);
  // Serial.print("TxPacket2[14]");Serial.println(TxPacket2[14]);
  //---MOtor ID:2----//
  TxPacket2[17] = 0x02; //ID:0x02 
  TxPacket2[18] = (pos ) & 0x00FF; //pos_low
  TxPacket2[19] = ((pos ) >> 8) & 0x00FF; //pos_high
  //---MOtor ID:3----//
  TxPacket2[22] = 0x03; //ID:0x03
  TxPacket2[23] = (pos ) & 0x00FF; //pos_low
  TxPacket2[24] = ((pos ) >> 8) & 0x00FF; //pos_high
  //---MOtor ID:4----//
  TxPacket2[27] = 0x04; //ID:0x04 
  TxPacket2[28] = (pos ) & 0x00FF; //pos_low
  TxPacket2[29] = ((pos ) >> 8) & 0x00FF; //pos_high
  //---MOtor ID:5----//
  TxPacket2[32] = 0x05; //ID:0x05
  TxPacket2[33] = (pos ) & 0x00FF; //pos_low
  TxPacket2[34] = ((pos ) >> 8) & 0x00FF; //pos_high
  //---MOtor ID:6----//
  TxPacket2[37] = 0x06; //ID:0x06
  TxPacket2[38] = (pos) & 0x00FF; //pos_low
  TxPacket2[39] = ((pos) >> 8) & 0x00FF; //pos_high
  //---MOtor ID:7----//
  TxPacket2[42] = 0x07; //ID:0x07 
  TxPacket2[43] = (pos ) & 0x00FF; //pos_low
  TxPacket2[44] = ((pos) >> 8) & 0x00FF; //pos_high
  //---MOtor ID:8----//
  TxPacket2[47] = 0x08; //ID:0x08 
  TxPacket2[48] = (pos) & 0x00FF; //pos_low
  TxPacket2[49] = ((pos) >> 8) & 0x00FF; //pos_high
  //---MOtor ID:9----//
  TxPacket2[52] = 0x09; //ID:0x09 
  TxPacket2[53] = (pos ) & 0x00FF; //pos_low
  TxPacket2[54] = ((pos ) >> 8) & 0x00FF; //pos_high
  //---MOtor ID:10----//
  TxPacket2[57] = 0x0A; //ID:0x010
  TxPacket2[58] = (pos ) & 0x00FF; //pos_low
  TxPacket2[59] = ((pos ) >> 8) & 0x00FF; //pos_high
  //---MOtor ID:11----//
  TxPacket2[62] = 0x0B; //ID:0x011 
  TxPacket2[63] = (pos ) & 0x00FF; //pos_low
  TxPacket2[64] = ((pos ) >> 8) & 0x00FF; //pos_high
  //---MOtor ID:12----//
  TxPacket2[67] = 0x0C; //ID:0x012 
  TxPacket2[68] = (pos ) & 0x00FF; //pos_low
  TxPacket2[69] = ((pos ) >> 8) & 0x00FF; //pos_high

  CRC = update_crc(0, TxPacket2, 72);
  CRC_L = (CRC & 0x00FF);
  CRC_H = (CRC >> 8) & 0x00FF;
  TxPacket2[72] = CRC_L;
  TxPacket2[73] = CRC_H;
  Serial1.write(TxPacket2, 74);
  delay(10);
}

void Goal_position_rorate(int id , double rorate) {
  uint16_t crc_accum = 0;
  uint16_t CRC;
  unsigned char CRC_L = 0;
  unsigned char CRC_H = 0;
  uint16_t data_blk_size = 18;//size of TxPacket
  pos = rorate;
  map_pos();
  unsigned char TxPacket2[18] = { 0xFF, 0xFF, 0xFD, 0x00, metor_id[id], 0x09, 0x00, 0x03, 0x74, 0x00,  pos_low, pos_high, 0x00, 0x00, CRC_L, CRC_H };
  CRC = update_crc(0, TxPacket2, 14);
  CRC_L = (CRC & 0x00FF);
  CRC_H = (CRC >> 8) & 0x00FF;
  TxPacket2[14] = CRC_L;
  TxPacket2[15] = CRC_H;
  Serial1.write(TxPacket2, 16);
  delay(8);
}

//call this fuction to do 360 rotation
void get_position_after_rotation(double phi, double theta, double xo, double yo, double zo, double &x, double &y, double &z) {
  double R[3][3];
  double u = cos(phi * M_PI / 180);
  double v = sin(phi * M_PI / 180);
  double init_r[3];
  double after_r[3];
  int i, j;
  double temp_sum = 0;

  init_r[0] = xo;
  init_r[1] = yo;
  init_r[2] = zo;
  R[0][0] = u * u + v * v * cos(theta * M_PI / 180);
  R[0][1] = u * v * (1 - cos(theta * M_PI / 180));
  R[0][2] = v * sin(theta * M_PI / 180);
  R[1][0] = R[0][1];
  R[1][1] = v * v + u * u * cos(theta * M_PI / 180);
  R[1][2] = -u * sin(theta * M_PI / 180);
  R[2][0] = -R[0][2];
  R[2][1] = -R[1][2];
  R[2][2] = cos(theta * M_PI / 180);

  for (i = 0; i < 3; i++) {
    temp_sum = 0;
    for (j = 0; j < 3; j++) {
      temp_sum = temp_sum + R[i][j] * init_r[j];
    }
    after_r[i] = temp_sum;
  }

  x = after_r[0] - init_r[0];
  y = after_r[1] - init_r[1];
  z = after_r[2];
}

//rotation 360
void servopos_360(int counter, int ite) {
  int dir;
  double phi = 360.0 * counter / ite;
  double theta = 10.0;
  for (i = 0; i < 4; i++) {
    if (i == 0) {
      get_position_after_rotation(phi, theta, half_length, half_width, -0.1, x, y, z);
      y = -y;
    } else if (i == 1) {
      get_position_after_rotation(phi, theta, half_length, -half_width, -0.1, x, y, z);
    } else if (i == 2) {
      get_position_after_rotation(phi, theta, -half_length, half_width, -0.1, x, y, z);
      y = -y;
    } else {
      get_position_after_rotation(phi, theta, -half_length, -half_width, -0.1, x, y, z);
    }


    first_rotation(y_axis, z_axis, y, z, newz, theta1[i]);
    quad_IK_2D(x, newz, l1, l2, a1, a2, a3, a4, r, angle1, angle2, angle3, theta2[i], theta3[i]);
  }

  for (i = 0; i < 12; i++) {

    // Serial.print("theta1[0]:  ");Serial.print(theta1[0]+90);
    // Serial.print("  theta1[1]:  ");Serial.print(theta1[1]+90);
    // Serial.print("  theta1[2]:  ");Serial.print(theta1[2]+90);
    // Serial.print("  theta1[3]:  ");Serial.println(theta1[3]+90);

    //  Serial.print("theta2[0]:  ");Serial.print(theta2[0]+50); //+75
    //  Serial.print("  theta2[1]:  ");Serial.print(-theta2[1]+130);//+105
    //  Serial.print("  theta2[2]:  ");Serial.print(theta2[2]+50);//+75
    //  Serial.print("  theta2[3]:  ");Serial.println(-theta2[3]+130);//+105

    // Serial.print("theta3[0]:  ");Serial.print(theta3[0]+40);
    // Serial.print("  theta3[1]:  ");Serial.print(-theta3[1]+140);
    // Serial.print("  theta3[2]:  ");Serial.print(theta3[2]+20);
    // Serial.print("  theta3[3]:  ");Serial.println(-theta3[3]+160);
    if (i < 8)
    {
      if (i == 0 || i == 2 || i == 4 || i == 6) {
        dir = 1;
      } else {
        dir = -1;
      }
    }
    if (i < 4) {
      if (i == 0 || i == 2)
      {
        Goal_position_rorate( i, dir * theta2[i % 4] + 50); //80
      }
      if (i == 1 || i == 3)
      {
        Goal_position_rorate( i, dir * theta2[i % 4] + 130); //+100
      }
    } else if (i > 7) {
      if (i == 8) {
        Goal_position_rorate( i, 1 * theta1[i % 4] + 90);
      }
      if (i == 9) {
        Goal_position_rorate( i, -1 * theta1[i % 4] + 90);
      }
      if (i == 10) {
        Goal_position_rorate( i, -1 * theta1[i % 4] + 90);
      }
      if (i == 11) {
        Goal_position_rorate( i, 1 * theta1[i % 4] + 90);
      }
    } else {
      if (i == 4 || i == 6) {
        Goal_position_rorate( i, dir * theta3[i % 4] + 40); //+20
      }
      else if (i == 5 || i == 7) {
        Goal_position_rorate( i, dir * theta3[i % 4] + 140 ); //+180
      }
    }
  }
}

void servopos_rotate_lt(int counter, int ite){
  ite = 150;
  int dir;
  double orient;
  int coor;
  L = 0.03; //0.035; //0.04; //0.055
  for(i=0;i < 4;i++){
    if (i==0){
      orient = -180+body_propotion;
      coor = -1;
    }else if (i==1){
      orient = body_propotion;
      coor = 1;
    }else if (i==2){
      orient = 180-body_propotion;
      coor = -1;
    }else{
      orient = -body_propotion;
      coor = 1;
    }
    if ((ite + counter%ite - i*ite/4)%ite < ite*1/2){
      x = (-L + (2.0*((ite + counter%ite - i*ite/4)%ite)*L/(1.0/2.0*ite)))*cos(orient/180.0*M_PI);
      y = coor*(-L + (2.0*((ite + counter%ite - i*ite/4)%ite)*L/(1.0/2.0*ite)))*sin(orient/180.0*M_PI);
      z = -28.57 * x * x  - 0.065;//-15.625 * x * x  - 0.075;//-200.0/9.0*x*x-0.08;
    }else{
      x = (L - (2.0*(((ite + counter%ite - i*ite/4)%ite)-1.0/2.0*ite)*L/(1.0/2.0*ite)))*cos(orient/180.0*M_PI);
      y = coor*(L - (2.0*(((ite + counter%ite - i*ite/4)%ite)-1.0/2.0*ite)*L/(1.0/2.0*ite)))*sin(orient/180.0*M_PI);
      z = -0.1;
    }
    first_rotation(y_axis, z_axis, y, z, newz, theta1[i]);
    quad_IK_2D(x, newz, l1, l2, a1, a2, a3, a4, r, angle1, angle2, angle3, theta2[i], theta3[i]);
  }

  
  for (i=0;i <12;i++){
    // Serial.print("theta1[0]:  ");Serial.print(theta1[0] +90);
    // Serial.print("  theta1[1]:  ");Serial.print(theta1[1] + 90);
    // Serial.print("  theta1[2]:  ");Serial.print(-theta1[2] +90);
    // Serial.print("  theta1[3]:  ");Serial.println(-theta1[3] +90);

    // Serial.print("theta2[0]:  ");Serial.print(-theta2[0] +70); //+75
    // Serial.print("  theta2[1]:  ");Serial.print(theta2[1] +110);//+105
    // Serial.print("  theta2[2]:  ");Serial.print(-theta2[2] +70);//+75
    // Serial.print("  theta2[3]:  ");Serial.println(theta2[3] +110);//+105

    // Serial.print("theta3[0]:  ");Serial.print(-theta3[0] +200);
    // Serial.print("  theta3[1]:  ");Serial.print(theta3[1] -20);
    // Serial.print("  theta3[2]:  ");Serial.print(-theta3[2] +200);
    // Serial.print("  theta3[3]:  ");Serial.println(theta3[3] -20);
    if (i < 8){
      if (i == 0 || i == 2 || i == 4 || i == 6) {
        dir = -1;
      } else {
        dir = 1;
      }
    }
    
    if (i < 4){
      if (i == 0 || i == 2)
      {
        Goal_position_walkortrop( i, dir * theta2[i % 4] + 75); //以90為基準向下減
      }
      if (i == 1 || i == 3)
      {
        Goal_position_walkortrop( i, dir * theta2[i % 4] + 110); //以90為基準向上加
      }
    }else if (i > 7){
      if (i==8){
        Goal_position_walkortrop( i,1 * theta1[i % 4] + 90);
      }
      if (i==9){
        Goal_position_walkortrop( i,1 * theta1[i % 4] + 90);
      }
      if (i==10){
        Goal_position_walkortrop( i,-1 * theta1[i % 4] + 90);
      }
      if (i==11){
        Goal_position_walkortrop( i,-1 * theta1[i % 4] + 90);
      }
    }else{
      if (i == 4 || i == 6) {
        Goal_position_walkortrop( i, dir * theta3[i % 4] + 195 ); //以180為基準向上加
      }
      else if (i == 5 || i == 7) {
        Goal_position_walkortrop( i, dir * theta3[i % 4] - 20 ); //以0為基準 向下減
      }
    }
  }
}

void servopos_rorate_rt(int counter, int ite){
  ite = 150;
  int dir;
  double orient;
  int coor;
  L = 0.03; //0.035; //0.04; //0.055
  for(i=0;i < 4;i++){
    if (i==0){
      orient = body_propotion;
      coor = -1;
    }else if (i==1){
      orient = -180+body_propotion;
      coor = 1;
    }else if (i==2){
      orient = -body_propotion;
      coor = -1;
    }else{
      orient = 180-body_propotion;
      coor = 1;
    }
    if ((ite + counter%ite - i*ite/4)%ite < ite*0.25){
      x = (-L + (2.0*((ite + counter%ite - i*ite/4)%ite)*L/(0.25*ite)))*cos(orient/180.0*M_PI);
      y = coor*(-L + (2.0*((ite + counter%ite - i*ite/4)%ite)*L/(0.25*ite)))*sin(orient/180.0*M_PI);
      z = -44.44 * x * x  - 0.06;//-28.57 * x * x  - 0.065;//-15.625 * x * x  - 0.075;//-200.0/9.0*x*x-0.08;
    }else{
      x = (L - (2.0*(((ite + counter%ite - i*ite/4)%ite)-0.25*ite)*L/(0.75*ite)))*cos(orient/180.0*M_PI);
      y = coor*(L - (2.0*(((ite + counter%ite - i*ite/4)%ite)-0.25*ite)*L/(0.75*ite)))*sin(orient/180.0*M_PI);
      z = -0.1;
    }

    first_rotation(y_axis, z_axis, y, z, newz, theta1[i]);
    quad_IK_2D(x, newz, l1, l2, a1, a2, a3, a4, r, angle1, angle2, angle3, theta2[i], theta3[i]);
  }

  for (i=0;i <12;i++){
    // Serial.print("theta1[0]:  ");Serial.print(theta1[0] +90);
    // Serial.print("  theta1[1]:  ");Serial.print(theta1[1] + 90);
    // Serial.print("  theta1[2]:  ");Serial.print(-theta1[2] +90);
    // Serial.print("  theta1[3]:  ");Serial.println(-theta1[3] +90);

    // Serial.print("theta2[0]:  ");Serial.print(-theta2[0] +65); //+75
    // Serial.print("  theta2[1]:  ");Serial.print(theta2[1] +115);//+105
    // Serial.print("  theta2[2]:  ");Serial.print(-theta2[2] +65);//+75
    // Serial.print("  theta2[3]:  ");Serial.println(theta2[3] +11);//+105

    // Serial.print("theta3[0]:  ");Serial.print(-theta3[0] +205);
    // Serial.print("  theta3[1]:  ");Serial.print(theta3[1] -25);
    // Serial.print("  theta3[2]:  ");Serial.print(-theta3[2] +205);
    // Serial.print("  theta3[3]:  ");Serial.println(theta3[3] -25);
    if (i < 8)
    {
      if (i == 0 || i == 2 || i == 4 || i == 6) {
        dir = -1;
      } else {
        dir = 1;
      }
    }
    if (i < 4){
      if (i == 0){
        Goal_position_walkortrop( i, dir * theta2[i % 4] + 65); //以90為基準向下減  85, 60, 50
      }
      if (i == 2){
        Goal_position_walkortrop( i, dir * theta2[i % 4] + 60); //以90為基準向下減 85, 60, 55
      }
      if (i == 1) {
        Goal_position_walkortrop( i, dir * theta2[i % 4] + 115); //以90為基準向上加  95, 120, 130
      }
      if (i == 3) {
        Goal_position_walkortrop( i, dir * theta2[i % 4] + 120); //以90為基準向上加  95, 120, 130 125
      }
    }else if (i > 7){
      if (i==8){
        Goal_position_walkortrop( i,1 * theta1[i % 4] + 90);
      }
      if (i==9){
        Goal_position_walkortrop( i,1 * theta1[i % 4] + 90);
      }
      if (i==10){
        Goal_position_walkortrop( i,-1 * theta1[i % 4] + 90);
      }
      if (i==11){
        Goal_position_walkortrop( i,-1 * theta1[i % 4] + 90);
      }
    }else{
      if (i == 4) {
        Goal_position_walkortrop( i, dir * theta3[i % 4] + 205 ); //以180為基準向上加
      }
      if (i == 6) {
        Goal_position_walkortrop( i, dir * theta3[i % 4] + 210 ); //以180為基準向上加
      }
      if (i == 5)
      {
        Goal_position_walkortrop( i, dir * theta3[i % 4] - 25 ); //以0為基準 向下減
      }
      if (i == 7)
      {
        Goal_position_walkortrop( i, dir * theta3[i % 4] - 30 ); //以0為基準 向下減
      }
    }
  }
}

void synwrite_Goal_position_IMU(double rorate) {
  uint16_t crc_accum = 0;
  uint16_t CRC;
  unsigned char CRC_L = 0;
  unsigned char CRC_H = 0;
  // pos = map(rorate, 0, 360, 0, 4096);
  // Serial.print("Tpos");Serial.println(pos);
  // map_pos();
  pos = rorate;
  unsigned char TxPacket2[14 + 5 * 12];


  // Serial.print("pitch");Serial.print(pitch);
  // Serial.print("  output_x");Serial.print(output_x);
  // Serial.print("     pitchb");Serial.print(pitchb );
  // Serial.print("     pitchf");Serial.println(pitchf );

  TxPacket2[0] = TxPacket2[1] = 0xFF;
  TxPacket2[2] = 0xFD;
  TxPacket2[3] = 0x00;
  TxPacket2[4] = 0xFE;
  TxPacket2[5] = 0x43;  //4 + 5*12 + 3
  TxPacket2[6] = 0x00;  
  TxPacket2[7] = 0x83;  //0x83
  TxPacket2[8] = 0x74;  //0x74 p1
  TxPacket2[9] = 0x00;  //0x00 p2
  TxPacket2[10] = 0x04; //0x04 p3
  TxPacket2[11] = 0x00; //0x00 p4
  TxPacket2[15] = TxPacket2[16] = TxPacket2[20] = TxPacket2[21] = TxPacket2[25] = TxPacket2[26] = TxPacket2[30] = TxPacket2[31] = TxPacket2[35] = TxPacket2[36] =TxPacket2[40] =TxPacket2[41] = TxPacket2[45] = TxPacket2[46] =  TxPacket2[50] = TxPacket2[51] =TxPacket2[55] =  TxPacket2[56] = TxPacket2[60] = TxPacket2[61] = TxPacket2[65] = TxPacket2[66] = TxPacket2[70] = TxPacket2[71]  = 0x00; //0x00 p4
  //---MOtor ID:1----//
  TxPacket2[12] = 0x01; //ID:0x01 
  TxPacket2[13] = ((pos - 25 + pitchb - rool_b1) * 4096 /360 ) & (0x00FF); //pos_low
  TxPacket2[14] = ((((pos - 25 + pitchb - rool_b1))  * 4096 / 360 ) >> 8) & (0x00FF); //pos_high
  //---MOtor ID:2----//
  TxPacket2[17] = 0x02; //ID:0x02 
  TxPacket2[18] = ((pos + 25 - pitchb - rool_f1) * 4096 /360 ) & 0x00FF; //pos_low
  TxPacket2[19] = (((pos + 25 - pitchb - rool_f1) * 4096 /360 ) >> 8) & 0x00FF; //pos_high
  //---MOtor ID:3----//
  TxPacket2[22] = 0x03; //ID:0x03
  TxPacket2[23] = ((pos-25 - pitchf  - rool_b1) * 4096 /360 ) & 0x00FF; //pos_low
  TxPacket2[24] = (((pos-25 - pitchf - rool_b1) * 4096 /360 ) >> 8) & 0x00FF; //pos_high
  //---MOtor ID:4----//
  TxPacket2[27] = 0x04; //ID:0x04 
  TxPacket2[28] = ((pos + 25 + pitchf - rool_f1) * 4096 /360  ) & 0x00FF; //pos_low
  TxPacket2[29] = (((pos + 25 + pitchf - rool_f1) * 4096 /360 ) >> 8) & 0x00FF; //pos_high
  //---MOtor ID:5----//
  TxPacket2[32] = 0x05; //ID:0x05
  TxPacket2[33] = ((pos+30) * 4096 /360 ) & 0x00FF; //pos_low
  TxPacket2[34] = (((pos+30)* 4096 /360 ) >> 8) & 0x00FF; //pos_high
  //---MOtor ID:6----//
  TxPacket2[37] = 0x06; //ID:0x06
  TxPacket2[38] = ((pos-30)* 4096 /360) & 0x00FF; //pos_low
  TxPacket2[39] = (((pos-30)* 4096 /360) >> 8) & 0x00FF; //pos_high
  //---MOtor ID:7----//
  TxPacket2[42] = 0x07; //ID:0x07 
  TxPacket2[43] = ((pos+30)* 4096 /360) & 0x00FF; //pos_low
  TxPacket2[44] = (((pos+30)* 4096 /360) >> 8 ) & 0x00FF; //pos_high
  //---MOtor ID:8----//
  TxPacket2[47] = 0x08; //ID:0x08 
  TxPacket2[48] = ((pos-30)* 4096 /360) & 0x00FF; //pos_low
  TxPacket2[49] = (((pos-30)* 4096 /360) >> 8) & 0x00FF; //pos_high
  //---MOtor ID:9----//
  TxPacket2[52] = 0x09; //ID:0x09 
  TxPacket2[53] = ((pos - rool_f)* 4096 /360) & 0x00FF; //pos_low
  TxPacket2[54] = (((pos - rool_f)* 4096 /360) >> 8) & 0x00FF; //pos_high
  //---MOtor ID:10----//
  TxPacket2[57] = 0x0A; //ID:0x010
  TxPacket2[58] = ((pos - rool_b)* 4096 /360) & 0x00FF; //pos_low
  TxPacket2[59] = (((pos - rool_b)* 4096 /360) >> 8) & 0x00FF; //pos_high
  //---MOtor ID:11----//
  TxPacket2[62] = 0x0B; //ID:0x011 
  TxPacket2[63] = ((pos + rool_f)* 4096 /360) & 0x00FF; //pos_low
  TxPacket2[64] = (((pos + rool_f)* 4096 /360) >> 8) & 0x00FF; //pos_high
  //---MOtor ID:12----//
  TxPacket2[67] = 0x0C; //ID:0x012 
  TxPacket2[68] = ((pos + rool_b)* 4096 /360) & 0x00FF; //pos_low
  TxPacket2[69] = (((pos + rool_b)* 4096 /360) >> 8) & 0x00FF; //pos_high

  CRC = update_crc(0, TxPacket2, 72);
  CRC_L = (CRC & 0x00FF);
  CRC_H = (CRC >> 8) & 0x00FF;
  TxPacket2[72] = CRC_L;
  TxPacket2[73] = CRC_H;
  Serial1.write(TxPacket2, 74);
}

//syn_write left and right 
void synwrite_Goal_position_walkortrop_leftright() {
  uint16_t crc_accum = 0;
  uint16_t CRC;
  unsigned char CRC_L = 0;
  unsigned char CRC_H = 0;
  unsigned char TxPacket2[14 + 5 * 12];

  TxPacket2[0] = TxPacket2[1] = 0xFF;
  TxPacket2[2] = 0xFD;
  TxPacket2[3] = 0x00;
  TxPacket2[4] = 0xFE;
  TxPacket2[5] = 0x43;  //4 + 5*12 + 3
  TxPacket2[6] = 0x00;
  TxPacket2[7] = 0x83;  //0x83
  TxPacket2[8] = 0x74;  //0x74 p1
  TxPacket2[9] = 0x00;  //0x00 p2
  TxPacket2[10] = 0x04; //0x04 p3
  TxPacket2[11] = 0x00; //0x00 p4
  TxPacket2[15] = TxPacket2[16] = TxPacket2[20] = TxPacket2[21] = TxPacket2[25] = TxPacket2[26] = TxPacket2[30] = TxPacket2[31] = TxPacket2[35] = TxPacket2[36] = TxPacket2[40] = TxPacket2[41] = TxPacket2[45] = TxPacket2[46] =  TxPacket2[50] = TxPacket2[51] = TxPacket2[55] =  TxPacket2[56] = TxPacket2[60] = TxPacket2[61] = TxPacket2[65] = TxPacket2[66] = TxPacket2[70] = TxPacket2[71]  = 0x00; //0x00 p4
  //---MOtor ID:1----//
  TxPacket2[12] = 0x01; //ID:0x01
  TxPacket2[13] = (int)(map_theta2[0] * 4096.0 / 360.0 ) & (0x00FF); //pos_low
  TxPacket2[14] = ((int)(map_theta2[0]  * 4096.0 / 360.0 ) >> 8) & (0x00FF); //pos_high
  //---MOtor ID:2----//
  TxPacket2[17] = 0x02; //ID:0x02
  TxPacket2[18] = (int)(map_theta2[1] * 4096.0 / 360.0 ) & 0x00FF; //pos_low
  TxPacket2[19] = ((int)(map_theta2[1] * 4096.0 / 360.0 ) >> 8) & 0x00FF; //pos_high
  //---MOtor ID:3----//
  TxPacket2[22] = 0x03; //ID:0x03
  TxPacket2[23] = (int)(map_theta2[2] * 4096.0 / 360.0 ) & 0x00FF; //pos_low
  TxPacket2[24] = ((int)(map_theta2[2] * 4096.0 / 360.0 ) >> 8) & 0x00FF; //pos_high
  //---MOtor ID:4----//
  TxPacket2[27] = 0x04; //ID:0x04
  TxPacket2[28] = (int)(map_theta2[3] * 4096.0 / 360.0  ) & 0x00FF; //pos_low
  TxPacket2[29] = ((int)(map_theta2[3] * 4096.0 / 360.0 ) >> 8) & 0x00FF; //pos_high
  //---MOtor ID:5----//
  TxPacket2[32] = 0x05; //ID:0x05
  TxPacket2[33] = (int)(map_theta3[0] * 4096.0 / 360.0 ) & 0x00FF; //pos_low
  TxPacket2[34] = ((int)(map_theta3[0] * 4096.0 / 360.0 ) >> 8) & 0x00FF; //pos_high
  //---MOtor ID:6----//
  TxPacket2[37] = 0x06; //ID:0x06
  TxPacket2[38] = (int)(map_theta3[1] * 4096.0 / 360.0) & 0x00FF; //pos_low
  TxPacket2[39] = ((int)(map_theta3[1] * 4096.0 / 360.0) >> 8) & 0x00FF; //pos_high
  //---MOtor ID:7----//
  TxPacket2[42] = 0x07; //ID:0x07
  TxPacket2[43] = (int)(map_theta3[2] * 4096.0 / 360.0) & 0x00FF; //pos_low
  TxPacket2[44] = ((int)(map_theta3[2] * 4096.0 / 360.0) >> 8 ) & 0x00FF; //pos_high
  //---MOtor ID:8----//
  TxPacket2[47] = 0x08; //ID:0x08
  TxPacket2[48] = (int)(map_theta3[3] * 4096.0 / 360.0) & 0x00FF; //pos_low
  TxPacket2[49] = ((int)(map_theta3[3] * 4096.0 / 360.0) >> 8) & 0x00FF; //pos_high
  //---MOtor ID:9----//
  TxPacket2[52] = 0x09; //ID:0x09
  TxPacket2[53] = (int)(map_theta1[0] * 4096.0 / 360.0) & 0x00FF; //pos_low
  TxPacket2[54] = ((int)(map_theta1[0] * 4096.0 / 360.0) >> 8) & 0x00FF; //pos_high
  //---MOtor ID:10----//
  TxPacket2[57] = 0x0A; //ID:0x010
  TxPacket2[58] = (int)(map_theta1[1] * 4096.0 / 360.0) & 0x00FF; //pos_low
  TxPacket2[59] = ((int)(map_theta1[1] * 4096.0 / 360.0) >> 8) & 0x00FF; //pos_high
  //---MOtor ID:11----//
  TxPacket2[62] = 0x0B; //ID:0x011
  TxPacket2[63] = (int)(map_theta1[2] * 4096.0 / 360.0) & 0x00FF; //pos_low
  TxPacket2[64] = ((int)(map_theta1[2] * 4096.0 / 360.0) >> 8) & 0x00FF; //pos_high
  //---MOtor ID:12----//
  TxPacket2[67] = 0x0C; //ID:0x012
  TxPacket2[68] = (int)(map_theta1[3] * 4096.0 / 360.0) & 0x00FF; //pos_low
  TxPacket2[69] = ((int)(map_theta1[3] * 4096.0 / 360.0) >> 8) & 0x00FF; //pos_high

  CRC = update_crc(0, TxPacket2, 72);
  CRC_L = (CRC & 0x00FF);
  CRC_H = (CRC >> 8) & 0x00FF;
  TxPacket2[72] = CRC_L;
  TxPacket2[73] = CRC_H;
  Serial1.write(TxPacket2, 74);
}


void imuRead() {
  if (Serial5.available() > 0) {
    Serial5.readBytes(readin, 1);
    if (readin[0] == 0x5A) {
      Serial5.readBytes(readin, 1);
      if (readin[0] == 0xA5) {
        Serial5.readBytes(readin, 5);
        uint16_t payload_len = readin[0] + (readin[1] << 8);
        if(payload_len == 7  && readin[4] == 0xD0 ){
        Serial5.readBytes(readin, 6);

        p_byte[0] = readin[0];
        p_byte[1] = readin[1];
        r_byte[0] = readin[2];
        r_byte[1] = readin[3];
        y_byte[0] = readin[4];
        y_byte[1] = readin[5];

        pitch = (float)((p_byte[1] << 8) + p_byte[0]);
        rool = (float)((r_byte[1] << 8) + r_byte[0]);
        yaw = (float)((y_byte[1] << 8) + y_byte[0]);

        if (pitch > 32767) {
          pitch = (float)((p_byte[1] << 8) + p_byte[0]) - 0xffff;
        }
        if (rool > 32767) {
          rool = (float)((r_byte[1] << 8) + r_byte[0]) - 0xffff;
        }
        if (yaw > 32767) {
          yaw = (float)((y_byte[1] << 8) + y_byte[0]) - 0xffff;
        }

        pitch = pitch / 100;
        rool = rool / 100;
        yaw = yaw / 10;

        
      
        //pitch運行平衡控制
        unsigned long Now_time = millis();
        float dt = (Now_time - Start_time ) * 0.001;

        error_x = pitch - set_point_x ; // Calculate the error between the current orientation and the setpoint
        integral_x += error_x * dt; // Update the integral term with the accumulated error, scaled by the delta time
        if (integral_x > 10) integral_x = 10; // Limit the integral term to prevent windup
        else if (integral_x < -10) integral_x = -10;

        derivative_x = (error_x - prev_error_x) / dt; // Calculate the derivative term based on the difference between the current and previous errors, divided by the delta time
        output_x = KP_X * error_x + KI_X * integral_x + KD_X * derivative_x;

        // Update the previous error and time for the next iteration
        
        prev_error_x = error_x;
        Start_time = Now_time;
        if((pitch < 1.0 && pitch > -1.0)&&abs(abs(pre_pitch) - abs(pitch)) < 0.03 ){error_x = 0;} 
        if (abs(abs(set_point_x) - abs(pitch)) < 0.1) {integral_x = 0;}

        //rool運行平衡控制
        error_y = rool - set_point_y ; // Calculate the error between the current orientation and the setpoint
        integral_y += error_y * dt; // Update the integral term with the accumulated error, scaled by the delta time
        if (integral_y > 2.5) integral_y = 2.5; // Limit the integral term to prevent windup
        else if (integral_y < -2.5) integral_y = -2.5;

        derivative_y = (error_y - prev_error_y) / dt; // Calculate the derivative term based on the difference between the current and previous errors, divided by the delta time
        output_y = KP_Y * error_y + KI_Y * integral_y + KD_Y * derivative_y;

        // Update the previous error and time for the next iteration
        
        prev_error_y = error_y;
        Start_time = Now_time;

        if((rool < 1.0 && rool > -1.0)&&abs(abs(pre_rool) - abs(rool)) < 0.03 ){error_y = 0;}
        if (abs(abs(set_point_y) - abs(rool)) < 0.1) {integral_y = 0;}

        //Noise cancellation for IMU sensor signal
        // filter_pitch = decrease_noise * pre_pitch + (1 - decrease_noise) * pitch;
        // filter_rool = decrease_noise * pre_rool + (1 - decrease_noise) * rool;
        // pre_rool = filter_rool;
        // pre_pitch = filter_pitch;

        pitch = decrease_noise * pre_pitch + (1 - decrease_noise) * pitch;
        rool = decrease_noise * pre_rool + (1 - decrease_noise) * rool;
        pre_rool = rool;
        pre_pitch = pitch;
        
        //計算
        if(open_imu){
          if(pitch < 0.0) { 
          pitchb =  round(3.0 *output_x ) ;
          pitchf =  round(3.0 *output_x )  ;
          }else if(pitch > 0.0) {
            pitchf = round(3.0 *output_x  )  ;
            pitchb = round(3.0 *output_x  )  ;
          }else{
            pitchb = 0.0;
            pitchf = 0.0;
          } 
          if(rool < 0.0) { 
            rool_f =  round(8*output_y ) ;
            rool_b =  round(8 *output_y )  ;
            rool_b1 =  round(40 *output_y )  ;
            rool_f1 = round(40 *output_y )  ;
          }else if(rool > 0.0) {
            rool_f = round(8 *output_y  )  ;
            rool_b = round(8 *output_y  )  ;
            rool_f1 = round(40 *output_y  )  ;
            rool_b1 = round(40 *output_y  )  ;
          }else{
            rool_f = 0.0;
            rool_b = 0.0;
          }
          open_imu = false;
        }else{
          pitchb = 0.0;
          pitchf = 0.0;
          rool_f = 0.0;
          rool_b = 0.0;
          rool_f1 = 0.0;
          rool_b1 = 0.0;
        }

        // Serial.print(String(imu_number));Serial.print(',');
        // Serial.print(String(set_point_x));Serial.print(',');
        // Serial.print(String(set_point_y));Serial.print(',');
        // Serial.print( String(pitch));Serial.print(',');
        // Serial.println(String(rool));

        // Serial.print("set_point_x = " + String(set_point_x));Serial.print(", ");
        // Serial.print(" set_point_y = " + String(set_point_y));Serial.print(" ,");
        // Serial.print("pitch = " + String(pitch));Serial.print(" ,");
        // Serial.print("  output_x = " + String(output_x));
        // Serial.println("roll = " + String(rool));
        // Serial.print("  \tyaw = " + String(yaw));
        // Serial.print("  error_x = " + String(error_x));
        // Serial.print("  integral_x = " + String(integral_x));
        // Serial.print("  prev_error_x = " + String(prev_error_x));
        // Serial.println("  derivative_x = " + String(derivative_x));
        // Serial.print("  output_y = " + String(output_y));
        // Serial.print("  error_y = " + String(error_y));
        // Serial.print("  integral_y = " + String(integral_y));
        // Serial.print("  prev_error_y = " + String(prev_error_y));
        // Serial.println("  derivative_y = " + String(derivative_y));
        }
      }
    }
  }
}


void SYN_servopos_rorate_rt(int counter, int ite) {
  // int dir;
  double orient;
  int coor;
  int cycle_delay;
  L = 0.02; //0.025; //0.035; //0.04; //0.055
  if (change == true) {
    for (i = 0; i < 4; i++) {
      if (i == 3 || i == 0) {
        cycle_delay = 0;
      } else {
        cycle_delay = ite / 2;
      }
      if (i == 0) {
        orient = body_propotion;
        coor = -1;
      } else if (i == 1) {
        orient = -180 + body_propotion;
        coor = 1;
      } else if (i == 2) {
        orient = -body_propotion;
        coor = -1;
      } else {
        orient = 180 - body_propotion;
        coor = 1;
      }
      if ((ite + counter % ite - cycle_delay) % ite < ite * 1 / 2) {
        x = (-L + (2.0 * ((ite + counter % ite - cycle_delay) % ite) * L / (1.0 / 2.0 * ite))) * cos(orient / 180.0 * M_PI); //if ((ite + counter%ite - i*ite/4)%ite < ite*0.25){  //x = (-L + (2.0*((ite + counter%ite - i*ite/4)%ite)*L/(0.25*ite)))*cos(orient/180.0*M_PI);
        y = coor * (-L + (2.0 * ((ite + counter % ite - cycle_delay) % ite) * L / (1.0 / 2.0 * ite))) * sin(orient / 180.0 * M_PI); // y = coor*(-L + (2.0*((ite + counter%ite - i*ite/4)%ite)*L/(0.25*ite)))*sin(orient/180.0*M_PI);
        z = (-75 * x * x)  - 0.07; //(-56 * x * x)  - 0.065; //(-44.44 * x * x)  - 0.06; //-28.57 * x * x  - 0.065;//-15.625 * x * x  - 0.075;//-200.0/9.0*x*x-0.08;
      } else {
        x = (L - (2.0 * (((ite + counter % ite - cycle_delay) % ite) - 1.0 / 2.0 * ite) * L / (1.0 / 2.0 * ite))) * cos(orient / 180.0 * M_PI); //x = (L - (2.0*(((ite + counter%ite - i*ite/4)%ite)-0.25*ite)*L/(0.75*ite)))*cos(orient/180.0*M_PI);
        y = coor * (L - (2.0 * (((ite + counter % ite - cycle_delay) % ite) - 1.0 / 2.0 * ite) * L / (1.0 / 2.0 * ite))) * sin(orient / 180.0 * M_PI); //y = coor*(L - (2.0*(((ite + counter%ite - i*ite/4)%ite)-0.25*ite)*L/(0.75*ite)))*sin(orient/180.0*M_PI);
        z = -0.1;
      }
      first_rotation(y_axis, z_axis, y, z, newz, theta1[i]);
      quad_IK_2D(x, newz, l1, l2, a1, a2, a3, a4, r, angle1, angle2, angle3, theta2[i], theta3[i]);
      if ( i == 0) {
        map_theta2[i] = -1 * theta2[i] + 55;//大腿 以90為基準向下減   //35
        map_theta3[i] = -1 * theta3[i] + 190;//小腿 以180為基準向上加  //10
        map_theta1[i] = 1 * theta1[i] + 90;
      }
      if ( i == 1) {
        map_theta2[i] = 1 * theta2[i] + 125;//大腿 以90為基準向上加 //35
        map_theta3[i] = 1 * theta3[i] - 10;//小腿 以0為基準 向下減  //10
        map_theta1[i] = 1 * theta1[i] + 90;
      }
      if ( i == 2 ) {
        map_theta2[i] = -1 * theta2[i] + 55;//大腿 以90為基準向下減   //35
        map_theta3[i] = -1 * theta3[i] + 190;//小腿 以180為基準向上加   //10
        map_theta1[i] = -1 * theta1[i] + 90;
      }
      if ( i == 3) {
        map_theta2[i] = 1 * theta2[i] + 125;//大腿 以90為基準向上加   //35
        map_theta3[i] = 1 * theta3[i] - 10;//小腿 以0為基準 向下減   //10
        map_theta1[i] = -1 * theta1[i] + 90;
      }
    }
    change = false;
  }
}

void SYN_servopos_rotate_lt(int counter, int ite) {
  // int dir;
  double orient;
  int coor;
  int cycle_delay;
  L = 0.02; //0.025;  //0.035; //0.04; //0.055
  if(change){
    for (i = 0; i < 4; i++) {
      if (i == 0 || i == 3) {
        cycle_delay = 0;
      } else {
        cycle_delay = ite / 2;
      }
      if (i == 0) {
        orient = -180 + body_propotion;
        coor = -1;
      } else if (i == 1) {
        orient = body_propotion;
        coor = 1;
      } else if (i == 2) {
        orient = 180 - body_propotion;
        coor = -1;
      } else {
        orient = -body_propotion;
        coor = 1;
      }
      if ((ite + counter % ite - cycle_delay) % ite < ite * 1 / 2) {
        x = (-L + (2.0 * ((ite + counter % ite - cycle_delay) % ite) * L / (1.0 / 2.0 * ite))) * cos(orient / 180.0 * M_PI); //if ((ite + counter%ite - i*ite/4)%ite < ite*0.25){  //x = (-L + (2.0*((ite + counter%ite - i*ite/4)%ite)*L/(0.25*ite)))*cos(orient/180.0*M_PI);
        y = coor * (-L + (2.0 * ((ite + counter % ite - cycle_delay) % ite) * L / (1.0 / 2.0 * ite))) * sin(orient / 180.0 * M_PI); // y = coor*(-L + (2.0*((ite + counter%ite - i*ite/4)%ite)*L/(0.25*ite)))*sin(orient/180.0*M_PI);
        z = (-75 * x * x)  - 0.07; //(-56 * x * x)  - 0.065; //(-44.44 * x * x)  - 0.06; //-28.57 * x * x  - 0.065;//-15.625 * x * x  - 0.075;//-200.0/9.0*x*x-0.08;
      } else {
        x = (L - (2.0 * (((ite + counter % ite - cycle_delay) % ite) - 1.0 / 2.0 * ite) * L / (1.0 / 2.0 * ite))) * cos(orient / 180.0 * M_PI); //x = (L - (2.0*(((ite + counter%ite - i*ite/4)%ite)-0.25*ite)*L/(0.75*ite)))*cos(orient/180.0*M_PI);
        y = coor * (L - (2.0 * (((ite + counter % ite - cycle_delay) % ite) - 1.0 / 2.0 * ite) * L / (1.0 / 2.0 * ite))) * sin(orient / 180.0 * M_PI); //y = coor*(L - (2.0*(((ite + counter%ite - i*ite/4)%ite)-0.25*ite)*L/(0.75*ite)))*sin(orient/180.0*M_PI);
        z = -0.1;
      }
      // if ((ite + counter % ite - i * ite / 4) % ite < ite * 1 / 2) {
      //   x = (-L + (2.0 * ((ite + counter % ite - i * ite / 4) % ite) * L / (1.0 / 2.0 * ite))) * cos(orient / 180.0 * M_PI);
      //   y = coor * (-L + (2.0 * ((ite + counter % ite - i * ite / 4) % ite) * L / (1.0 / 2.0 * ite))) * sin(orient / 180.0 * M_PI);
      //   z = -44.44 * x * x  - 0.06;//-28.57 * x * x  - 0.065;//-15.625 * x * x  - 0.075;//-200.0/9.0*x*x-0.08;
      // } else {
      //   x = (L - (2.0 * (((ite + counter % ite - i * ite / 4) % ite) - 1.0 / 2.0 * ite) * L / (1.0 / 2.0 * ite))) * cos(orient / 180.0 * M_PI);
      //   y = coor * (L - (2.0 * (((ite + counter % ite - i * ite / 4) % ite) - 1.0 / 2.0 * ite) * L / (1.0 / 2.0 * ite))) * sin(orient / 180.0 * M_PI);
      //   z = -0.1;
      // }
      first_rotation(y_axis, z_axis, y, z, newz, theta1[i]);
      quad_IK_2D(x, newz, l1, l2, a1, a2, a3, a4, r, angle1, angle2, angle3, theta2[i], theta3[i]);
      if( i == 0){
        map_theta2[i] = -1 * theta2[i] + 55;//大腿 以90為基準向下減   //35
        map_theta3[i] = -1 * theta3[i] + 190;//小腿 以180為基準向上加  //10
        map_theta1[i] = 1 * theta1[i] + 90;
      }
      if( i == 1){
        map_theta2[i] = 1 * theta2[i] + 125;//大腿 以90為基準向上加 //35
        map_theta3[i] = 1 * theta3[i] - 10;//小腿 以0為基準 向下減  //10
        map_theta1[i] = 1* theta1[i] + 90;
      }
      if( i == 2 ){
        map_theta2[i] = -1 * theta2[i] + 55;//大腿 以90為基準向下減   //35
        map_theta3[i] = -1 * theta3[i] + 190;//小腿 以180為基準向上加   //10
        map_theta1[i] = -1 * theta1[i] + 90;
      }
      if( i == 3){
        map_theta2[i] = 1 * theta2[i] + 125;//大腿 以90為基準向上加   //35
        map_theta3[i] = 1 * theta3[i] - 10;//小腿 以0為基準 向下減   //10
        map_theta1[i] = -1 * theta1[i] + 90;
      }
      // Serial.print("theta1[0]:  ");Serial.println(map_theta1[0]);
      // Serial.print("  theta1[1]:  ");Serial.print(theta1[1] + 90);
      // Serial.print("  theta1[2]:  ");Serial.print(-theta1[2] +90);
      // Serial.print("  theta1[3]:  ");Serial.println(-theta1[3] +90);

      // Serial.print("theta2[0]:  ");Serial.print(-theta2[0] +70); //+75
      // Serial.print("  theta2[1]:  ");Serial.print(theta2[1] +110);//+105
      // Serial.print("  theta2[2]:  ");Serial.print(-theta2[2] +70);//+75
      // Serial.print("  theta2[3]:  ");Serial.println(theta2[3] +110);//+105

      // Serial.print("theta3[0]:  ");Serial.print(-theta3[0] +200);
      // Serial.print("  theta3[1]:  ");Serial.print(theta3[1] -20);
      // Serial.print("  theta3[2]:  ");Serial.print(-theta3[2] +200);
      // Serial.print("  theta3[3]:  ");Serial.println(theta3[3] -20);
    }
    change =false;
  }
}

void FSR_dectect(){
  FSR_V_1 = -FSR_read(FSR_1); // 讀取FSR1
  FSR_V_2 = -FSR_read(FSR_2); // 讀取FSR2
  FSR_V_3 = -FSR_read(FSR_3); // 讀取FSR3
  FSR_V_4 = -FSR_read(FSR_4); // 讀取FSR4
  Serial.print(-FSR_V_1);       //輸出A0腳位
  // Serial.print(-FSR_V_1 + FSR_V_1_offset);       //輸出A0腳位
  Serial.print(" , ");

  Serial.print(-FSR_V_2 );       //輸出A1腳位
  // Serial.print(-FSR_V_2 + FSR_V_2_offset);       //輸出A1腳位 
  Serial.print(" , ");

  Serial.print(-FSR_V_3 );       //輸出A2腳位
  // Serial.print(-FSR_V_3 + FSR_V_3_offset);       //輸出A2腳位
  Serial.print(" , ");

  Serial.println(-FSR_V_4 );      //輸出A3腳位
  // Serial.println(-FSR_V_4 + FSR_V_4_offset);      //輸出A3腳位
}

float FSR_read(int pin){
  const float Vcc = 3.25;
  const float ForceResDiv = 9870.0; //change it!
  float ForceVal;
  float Force;

  ForceVal = analogRead(pin);
  float ForceV = ForceVal * Vcc / 1023.0;
  float ForceR = ForceResDiv * (Vcc / ForceV - 1.0);
  float ForceG = 1.0 / ForceR;  // Calculate conductance

  if (ForceR <= 1000){
    Force = (ForceG - 0.00075) / 0.000000032639;
  }
  else{
    Force = ForceG / 0.0000000642857;
  }
  return Force;
}

float Filter(int DataPin) {
  int i;
  float filter_sum = 0;
  filter_buf[FILTER_N] = analogRead(DataPin);
  for(i = 0; i < FILTER_N; i++) {
    filter_buf[i] = filter_buf[i + 1]; // 所有數據左移，低位扔掉
    filter_sum += filter_buf[i] * coe[i];
  }
  filter_sum /= sum_coe;
  return filter_sum;
}