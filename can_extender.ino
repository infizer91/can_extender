/*
  CVM2 adapter and other improvements for cirocco:
  - speedlimits with blinking
  - afil
  - pseudo acc with distance alert

  Copyright 2022, Infizer <https://github.com/infizer91>
  Date: 26.09.2022

  This program is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, either version 3 of the License, or
  (at your option) any later version.
  This program is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU General Public License at <http://www.gnu.org/licenses/> for
  more details.
  The above copyright notice and this permission notice shall be included in
  all copies or substantial portions of the Software.


  Animation fix - Rodrigo Schroeder



  $$\      $$\  $$$$$$\  $$$$$$$\  $$$$$$$$\             $$$$$$\ $$\   $$\             $$$$$$$\  $$\   $$\  $$$$$$\   $$$$$$\  $$$$$$\  $$$$$$\
  $$$\    $$$ |$$  __$$\ $$  __$$\ $$  _____|            \_$$  _|$$$\  $$ |            $$  __$$\ $$ |  $$ |$$  __$$\ $$  __$$\ \_$$  _|$$  __$$\
  $$$$\  $$$$ |$$ /  $$ |$$ |  $$ |$$ |                    $$ |  $$$$\ $$ |            $$ |  $$ |$$ |  $$ |$$ /  \__|$$ /  \__|  $$ |  $$ /  $$ |
  $$\$$\$$ $$ |$$$$$$$$ |$$ |  $$ |$$$$$\                  $$ |  $$ $$\$$ |            $$$$$$$  |$$ |  $$ |\$$$$$$\  \$$$$$$\    $$ |  $$$$$$$$ |
  $$ \$$$  $$ |$$  __$$ |$$ |  $$ |$$  __|                 $$ |  $$ \$$$$ |            $$  __$$< $$ |  $$ | \____$$\  \____$$\   $$ |  $$  __$$ |
  $$ |\$  /$$ |$$ |  $$ |$$ |  $$ |$$ |                    $$ |  $$ |\$$$ |            $$ |  $$ |$$ |  $$ |$$\   $$ |$$\   $$ |  $$ |  $$ |  $$ |
  $$ | \_/ $$ |$$ |  $$ |$$$$$$$  |$$$$$$$$\             $$$$$$\ $$ | \$$ |            $$ |  $$ |\$$$$$$  |\$$$$$$  |\$$$$$$  |$$$$$$\ $$ |  $$ |
  \__|     \__|\__|  \__|\_______/ \________|            \______|\__|  \__|            \__|  \__| \______/  \______/  \______/ \______|\__|  \__|

*/

#include <can.h>      // https://github.com/autowp/arduino-mcp2515
#include <mcp2515.h>  // https://github.com/autowp/arduino-mcp2515
#include <SPI.h>
#include <Thread.h>
#define CAN_FREQ MCP_8MHZ

// can frames for reading can data
struct can_frame canMsg_0;
struct can_frame canMsg_1;

int reboot_timer = 0;
int reboot_counter = 0;

// VIN
char vin[18] = "Z8TND5FEAEM030597";
//char vin[18] = "VF7NX5FUBY522404";
//char vin[18] = "VF7KF5FEACS519474";
bool vin_accepted = false ;  // vin accepted


int vehicle_speed = 0;  // current vehicle speed

// SPEED LIMIT VARS
bool speed_limit = true;
struct can_frame speedlimit_msg;
int speed_correction = 3; // vehicle speed correction (standart +3)
int current_speed_limit = 0xFF;  // speedlimit current value
int speed_limit_cvm = 0;  // speedlimit from CVM
int speed_limit_nac = 0;  // speedlimit from NAC
int limit_type = 0x10;  // type of limit
int limit_type_counter = 0; // counter from CVM
int limit_correction = 20;  // limit correction for blinking if speed is more than ... 20, overspeed
uint32_t timer_blink_sign = 0;  // time counter for blinking if sign data was updated
uint32_t time_overspeed = 0;  // time counter for blinking in overspeed mode
bool overspeed = false; // overspeed mode
int overspeed_count = 0;
int blink_overspeed_count = 0;
int speed_limit_updater_count = 0;

//SAM
struct can_frame sam_msg;

////////////////////////
// AFIL
////////////////////////
struct can_frame afil_msg;  //
bool afil_on = false;
int afil_speed = 5; // speed for activate AFIL
// left line
bool afil_info_msg = false;
bool fix_left_line = false;
int type_left_line = 0;
int distance_to_left_line = 0;

// right line
bool fix_right_line = false;
int type_right_line = 0;
int distance_to_right_line = 0;

bool left = false;
bool right = false;
bool turning = false;

float distance = 0;

// ANIMATION
int Animation = 0x00;

//Change screen by LIST button
bool change_screen = false;
bool list_button = false;

// CLUSTER
struct can_frame cluster_msg;
bool cluster_msg_readed = false;

// FAKE ACC and RCW
struct can_frame cvm_diag_frame;  //
struct can_frame consecutive_frame;
bool cvm_diag = false;
//float distance = 0;
bool acc = false;

float speed_front_car = 0;
int speed_front_car_1 = 0;

int breaking = 0;
bool acc_active = false;

int distance_line = 0;
struct can_frame beep_msg;


MCP2515 can_0(9);
MCP2515 can_1(10);

Thread Thread_car = Thread();
Thread Thread_info = Thread();

void setup() {
  Serial.begin(115200);
  SPI.begin();

  can_0.reset();
  can_0.setBitrate(CAN_125KBPS, CAN_FREQ);
  can_0.setNormalMode();

  can_1.reset();
  can_1.setBitrate(CAN_125KBPS, CAN_FREQ);
  can_1.setNormalMode();

  speedlimit_msg.can_id = 0x268;
  speedlimit_msg.can_dlc = 8;
  speedlimit_msg.data[0] = 0xFF;
  speedlimit_msg.data[1] = 0x10;
  speedlimit_msg.data[2] = 0;
  speedlimit_msg.data[3] = 0;
  speedlimit_msg.data[4] = 0x7C;
  speedlimit_msg.data[5] = 0xF8;
  speedlimit_msg.data[6] = 0;
  speedlimit_msg.data[7] = 0;


  /// SAM
  sam_msg.can_id = 0x321;
  sam_msg.can_dlc = 5;
  sam_msg.data[0] = 0x0;
  sam_msg.data[1] = 0x0;
  sam_msg.data[2] = 0x0;
  sam_msg.data[3] = 0x0;
  sam_msg.data[4] = 0x0;

  // CLUSTER
  cluster_msg.can_id = 0x217;
  cluster_msg.can_dlc = 8;


  cvm_diag_frame.can_id = 0x74a;
  cvm_diag_frame.can_dlc = 4;
  cvm_diag_frame.data[0] = 0x3;
  cvm_diag_frame.data[1] = 0x22;
  cvm_diag_frame.data[2] = 0xD4;
  cvm_diag_frame.data[3] = 0x0A;

  consecutive_frame.can_id = 0x74a;
  consecutive_frame.can_dlc = 3;
  consecutive_frame.data[0] = 0x30;
  consecutive_frame.data[1] = 0;
  consecutive_frame.data[2] = 0xa;

  beep_msg.can_id = 0x102;
  beep_msg.can_dlc = 2;
  beep_msg.data[0] = 0xF;
  beep_msg.data[1] = 0xF0;

  Thread_car.onRun(read_car);
  Thread_info.onRun(read_info);

}


void loop() {
  if (Thread_car.shouldRun())
    Thread_car.run();

  if (Thread_info.shouldRun())
    Thread_info.run();

}

void read_car() {
  if (can_0.readMessage(&canMsg_0) == MCP2515::ERROR_OK)
  {
    check_msg_car(canMsg_0);
  }
}

void read_info() {

  if (can_1.readMessage(&canMsg_1) == MCP2515::ERROR_OK)
  {
    check_msg_info(canMsg_1);
  }
}


void check_msg_car(struct can_frame msg)
{


  ////////////////////////////////////////////
  //SPEED LIMIT
  ////////////////////////////////////////////

  if (msg.can_id == 0x11c) // frame from CVM2
  {
    //Serial.print("cvm_speed=");
    //Serial.println(msg.data[2]);

    if (msg.data[4] == 0 and msg.data[3] != 0)
    {
      // if (speed_limit_cvm != msg.data[2])
      Serial.println("update from cvm");
      speed_limit_cvm = msg.data[2];
      update_speed_limit(msg.data[2]);
    }
  }

  //Check left line
  if (msg.can_id == 0x5c)
  {
    //Serial.println(msg.data[5]);


    if ((msg.data[5] & 80 == 80) or (msg.data[5] & 1 == 1))
    {
      // fix left line
      type_left_line = msg.data[5] & 0xE;
      distance_to_left_line = ((msg.data[6] << 8) + msg.data[7]) / 4; // in mm



      fix_left_line = true;
    }
    else
    {
      fix_left_line = false;
    }
    /*
      Serial.print("left ");
      Serial.print(fix_left_line);
      Serial.print("-");
      Serial.println(distance_to_left_line);
    */
  }

  //Check right line
  if (msg.can_id == 0x9c)
  {
    if ((msg.data[5] & 80 == 80) or (msg.data[5] & 1 == 1))
    {
      // fix right line
      type_right_line = msg.data[5] & 0xE;
      distance_to_right_line = (0xFFFF - (msg.data[6] << 8) + msg.data[7]) / 4; // in mm

      fix_right_line = true;
    }
    else
    {
      fix_right_line = false;
    }

    //Serial.print("right ");
    //Serial.print(fix_right_line);
    //Serial.print("-");
    //Serial.println(distance_to_right_line);
  }



  //////////////////////////////////////// ACC

  if (acc == true)
  {
    if (msg.can_id == 0x74a)
    {
      //cvm_diag = true;
    }

    if (msg.can_id == 0x0f6 and cvm_diag == false and acc == true)
    {
      //Serial.println("start cvm diag");
      send_msg(cvm_diag_frame, 0);
    }

    if (msg.can_id == 0x64a)
    {
      //Serial.println("!!!!!!!!!!!!cvm started");
      //Serial.println(msg.can_dlc);
      if (msg.can_dlc == 8)
      {

        speed_front_car_1 = (msg.data[7] << 8);
        // consecutive frame
        send_msg(consecutive_frame, 0);
      }
      else
      {

        distance = (((msg.data[2] << 8) + msg.data[3]) >> 4) * 0.05; // in m
        Serial.print("distance to car ");
        Serial.println(distance);

        if (msg.data[0] << 4 == 0xF)
        {
          //Serial.println("asd");
          speed_front_car = (((msg.data[0] << 8) + msg.data[1]) - 0xFFFF >> 4) * 0.05; // in m
        }
        else
        {
          speed_front_car = ((speed_front_car_1 + msg.data[1]) >> 4) * 0.05; // in m
          //speed_front_car = msg.data[0];
        }

        /*
          Serial.println("----");
          Serial.println(msg.data[0]);
          Serial.println(msg.data[1]);
          Serial.println("----");
          Serial.print("speed to front car ");
          Serial.println(speed_front_car);
        */
        /*
          if(2 < distance  and distance < 30 and acc_active == true)
          {
          steer_buttons_msg.data[3] = 0x78;
          acc_active = false;

          //can_0.sendMessage(&steer_buttons_msg);
          }
          /*
          if (speed_front_car < -4)
          {
          can_0.sendMessage(&beep_msg);
          }
        */

      }
    }
  }
}

void check_msg_info(struct can_frame msg)
{

  ////////////////////////////////////////////
  // VIN
  ////////////////////////////////////////////
  if (vin_accepted == false and canMsg_1.can_id == 0x2b6)
  {
    if (canMsg_1.data[4] == int(vin[13]) and canMsg_1.data[5] == int(vin[14]) and canMsg_1.data[6] == int(vin[15]) and canMsg_1.data[7] == int(vin[16]))  // 15, 16, 17 символы вин
    {
      vin_accepted = true;
      Serial.println("vin OK!");
    }
    else
    {
      Serial.println("vin error");
    }
  }

  ////////////////////////////////////////////
  //COMMON VALUES
  ////////////////////////////////////////////
  if (msg.can_id == 0xb6)
  {
    vehicle_speed = (msg.data[2] << 8 | msg.data[3]) / 100;
    vehicle_speed = static_cast<int> (vehicle_speed) + speed_correction;
    //Serial.print("CAN_1 Vehicle speed = ");
    //Serial.println(vehicle_speed);
  }

  ////SPEED LIMIT
  if (msg.can_id == 0x1e9) // frame from NAC
  {
    //Serial.print("speed_limit_nac=");
    Serial.println(msg.data[1]);

    if (speed_limit_nac != msg.data[1])
    {
      Serial.println("update from nac");
      speed_limit_nac = msg.data[1];
      update_speed_limit(msg.data[1]);
    }
  }

  // ESC steer button
  if (msg.can_id == 0xa2)
  {
    if (msg.data[1] == 0x10) // dash indicator
    {
      Serial.println("----------");
      Serial.println(millis());
      Serial.println(reboot_timer);
      Serial.println("----------");
      if ((millis() - reboot_timer) > 3000)
      {
        Serial.println("reboot");
        delay(1000);
        asm volatile("jmp 0x00");
      }
      else
      {
        Serial.println("clear speed_limit");
        update_speed_limit(0xFF);
      }


    }
    else
    {
      reboot_timer = millis();
    }

  }


  if (msg.can_id == 0x268  and speed_limit == true)
  {
    if (speed_limit_updater_count == 0)
    {
      if (vehicle_speed > (current_speed_limit + limit_correction)) //
      {
        if (blink_overspeed_count <= 15)
        {
          blink_overspeed_count++;
          limit_type = 0x30;
        }
        else if (15 < blink_overspeed_count and blink_overspeed_count < 30)
        {
          limit_type = 0x10;
          blink_overspeed_count++;
        }
        else
        {
          blink_overspeed_count = 0;
        }
      }
      else
      {
        limit_type = 0x10;
        blink_overspeed_count = 0;
      }
    }
    else
    {
      speed_limit_updater_count--;
      limit_type = 0x20;
    }

    speedlimit_msg.data[0] = current_speed_limit;
    speedlimit_msg.data[1] = limit_type;
    send_msg(speedlimit_msg, 1);
  }

  ////////////////////////////////////////////
  //SAM INDICATOR
  ////////////////////////////////////////////

  if (msg.can_id == 0x2D1) // SAM ON
  {
    send_msg(sam_msg, 1);
  }

  ///////////////////////////////////
  ///AFIL
  ////////////////////////////////////////////

  //Check AFIL status
  if (canMsg_1.can_id == 0x227)
  {
    if (canMsg_1.data[1] == 0x10)
    {
      afil_on = true;
    }
    else
    {
      afil_on = false;
    }
  }


  ////////////////////////////////////////////
  // TURNING INDICATOR
  ////////////////////////////////////////////
  if (msg.can_id == 0xF6)  // dash indicator
  {
    if ((msg.data[7] & 2) == 2)
    {
      left = true;
      //Serial.println("left on");
    }
    else
    {
      left = false;
    }

    if ((msg.data[7] & 1) == 1)
    {
      right = true;
      //Serial.println("right on");
    }
    else
    {
      right = false;
    }
  }

  if (msg.can_id == 0x228)
  {
    afil_msg.can_id = 0x1E7;
    afil_msg.can_dlc = 8;

    if (distance > 0 and acc == true)
    {
      afil_msg.data[0] = 2;
    }
    else
    {
      afil_msg.data[0] = 0;
    }

    afil_msg.data[1] = 0x19;
    afil_msg.data[2] = 0xfe;
    afil_msg.data[3] = 0x65;
    afil_msg.data[4] = 0;
    afil_msg.data[5] = 0;
    afil_msg.data[6] = 0;
    afil_msg.data[7] = 0;

    if (fix_left_line == true)
    {
      afil_msg.data[5] = afil_msg.data[5] ^ 2;

      if (distance_to_left_line < 1300)
      {
        afil_msg.data[5] = afil_msg.data[5] ^ 1;

        if (left == false)
        {
          afil_msg.data[6] = 0x80;
        }
        else
        {
          afil_msg.data[6] = 0;
        }
      }
    }

    if (fix_right_line == true)
    {
      afil_msg.data[5] = afil_msg.data[5] ^ 8;

      if (distance_to_right_line < 1300)
      {
        afil_msg.data[5] = afil_msg.data[5] ^ 4;
        if (right == false)
        {
          afil_msg.data[6] = 0x80;
        }
        else
        {
          afil_msg.data[6] = 0;
        }
      }
    }

    if (afil_on == true)
    {
      send_msg(afil_msg, 1);
    }
  }

  ////////////////////////////////////////////
  //ANIMATION
  ////////////////////////////////////////////
  if (msg.can_id == 0x236)
  {
    if (Animation == 0x00 && millis() > 3500)
    {
      msg.data[5] = bitWrite(msg.data[5], 6, 1);
      send_msg(msg, 1);
      Animation = 0x01;
    }
  }

  ////////////////////////////////////////////
  //LIST BUTTON
  ////////////////////////////////////////////

  if (msg.can_id == 0x21F and msg.data[0] == 0x1 and list_button == true ) //
    //if (msg.can_id == 0x122 and msg.data[1] == 0x80 and list_button == true ) // for ds5 fmux button

  {
    change_screen = true;
  }

  if (msg.can_id == 0xA2 and list_button == true and change_screen == true)

    if (change_screen == true)
    {
      change_screen = false;

      if (msg.data[3] == 0xFF)
      {
        msg.data[3] = 0;
      }
      else
      {
        msg.data[3]++;
      }

      delay(250); //50 - disable list
      send_msg(msg, 1);
    }

  /// NAC BUTTONS
  if (msg.can_id == 0x217 and cluster_msg_readed == false)
  {
    cluster_msg.data[0] = canMsg_1.data[0];
    cluster_msg.data[1] = canMsg_1.data[1];
    cluster_msg.data[2] = canMsg_1.data[2];
    cluster_msg.data[3] = canMsg_1.data[3];
    cluster_msg.data[4] = canMsg_1.data[4];
    cluster_msg.data[5] = canMsg_1.data[5];
    cluster_msg.data[6] = canMsg_1.data[6];
    cluster_msg.data[7] = canMsg_1.data[7];
    cluster_msg_readed = true;

  }

  if (msg.can_id == 0x1A9 and cluster_msg_readed == true)
  {
    if (msg.data[3] == 0x20)
    {
      cluster_msg.data[3] = 0x8;
      send_msg(cluster_msg, 1);
    }
    else if (canMsg_1.data[3] == 0x10)
    {
      cluster_msg.data[3] = 0x40;
      send_msg(cluster_msg, 1);
    }
    else if (canMsg_1.data[3] == 0x08)
    {
      cluster_msg.data[3] = 0x80;
      send_msg(cluster_msg, 1);
    }

  }

  if (msg.can_id == 0x1e8 and (msg.data[2] && 0x20 == 0x20))
  {
    //mem_button = true;
    //Serial.println("MEM");

  }

  if (msg.can_id == 0x228 and acc == true)
  {
    /*
      Serial.print("ACC = :");
      Serial.println(canMsg_1.data[2]);
      if (canMsg_1.data[2] == 80)
      {
      Serial.println("ACTIVE");
      acc_active = true;
      }

      else
      {
      Serial.println("DISABLE");
      acc_active = false;
      }
    */

    if (msg.data[2] && 0x40 == 0x40)
      //if (canMsg_1.data[2] && 0x80 == 0x80)
    {
      msg.data[2] = msg.data[2] | 0x80;
      // canMsg_1.data[2] = canMsg_1.data[2] | 0xc0;

      //Serial.println(canMsg_1.data[4]);
      /*
        if (mem_button == true)
        {
        canMsg_1.data[4] = canMsg_1.data[4] &40;
        }

      */
      //Serial.println(canMsg_1.data[4]);

      if (distance >= 64)
      {
        msg.data[3] = 0x9f;
      }
      else
      {
        int distance_to_dislay = ((distance - 1) / 2.5 + 129);
        //Serial.print("distance_to_dislay ");
        //Serial.println(distance_to_dislay);
        msg.data[3] = msg.data[3] | distance_to_dislay; // 80- 9f   80 8a 8f  94 99
        msg.data[4] = msg.data[4] | distance_line;

      }

      send_msg(msg, 1);
    }
  }

  if (msg.can_id == 0x1a1)
  {
    if (vehicle_speed > 30 and distance < 30 and speed_front_car < -7)
    {
      Serial.println("BREAK!!!!!!!!!!");

      if (breaking == 0)
      {
        msg.data[0] = 0x7f;
        send_msg(msg, 1);
        breaking == 1;
      }

      msg.data[0] = 0x80;
      msg.data[1] = 0xa1;
      msg.data[2] = 0xa4;
      send_msg(msg, 1);

      send_msg(beep_msg, 0);
    }
    else {
      breaking = 0;
      int asd;
      //can_1.sendMessage(&canMsg_1);
    }
  }
}


void update_speed_limit(byte value)
{
  Serial.println("value for update");
  Serial.println(value);
  speed_limit_updater_count = 6;

  if (value == 0xFF or value == 0xFE or value == 0)
  {
    current_speed_limit = 0xFF;
  }
  else
  {
    current_speed_limit = value;
  }
  send_msg(speedlimit_msg, 1);
}

void send_msg(struct can_frame msg, int id)
{
  if (vin_accepted == true)
  {
    if (id == 0)
    {
      can_0.sendMessage(&msg);

    }
    else if (id == 1)
    {
      can_1.sendMessage(&msg);
    }
  }
}

void(* resetFunc) (void) = 0;
