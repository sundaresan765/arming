// /**
//  *    ||          ____  _ __
//  * +------+      / __ )(_) /_______________ _____  ___
//  * | 0xBC |     / __  / / __/ ___/ ___/ __ `/_  / / _ \*/
//  * +------+    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
//  *  ||  ||    /_____/_/\__/\___/_/   \__,_/ /___/\___/
//  *
//  * Crazyflie Firmware
//  *
//  * Copyright (C) 2011-2017 Bitcraze AB
//  *
//  * This program is free software: you can redistribute it and/or modify
//  * it under the terms of the GNU General Public License as published by
//  * the Free Software Foundation, in version 3.
//  *
//  * This program is distributed in the hope that it will be useful,
//  * but WITHOUT ANY WARRANTY; without even the implied warranty of
//  * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
//  * GNU General Public License for more details.
//  *
//  * You should have received a copy of the GNU General Public License
//  * along with this program. If not, see <http://www.gnu.org/licenses/>.
//  *
//  *
//  */
// #include <stdbool.h>
// #include <stddef.h>

// #include "crtp_commander.h"

// #include "cfassert.h"
// #include "commander.h"
// #include "crtp.h"
// #include "stdio.h"


// static bool isInit;

// static void commanderCrtpCB(CRTPPacket* pk);

// void crtpCommanderInit(void)
// {
//   if(isInit) {
//     return;
//   }

//   crtpInit();
//   crtpRegisterPortCB(CRTP_PORT_SETPOINT, commanderCrtpCB);
//   crtpRegisterPortCB(CRTP_PORT_SETPOINT_GENERIC, commanderCrtpCB);
//   isInit = true;
// }

// enum crtpSetpointGenericChannel {
//   SET_SETPOINT_CHANNEL = 0,
//   META_COMMAND_CHANNEL = 1,
// };

// /* Channel 1 of the generic commander port is used for "meta-commands"
//  * that alter the behavior of the commander itself, e.g. mode switching.
//  * Although we use the generic commander port due to increasing pressure on the
//  * 4-bit space of ports numbers, meta-commands that are unrelated to
//  * streaming generic setpoint control modes are permitted.
//  *
//  * The packet format for meta-commands is:
//  * +------+==========================+
//  * | TYPE |     DATA                 |
//  * +------+==========================+
//  *
//  * TYPE is an 8-bit value. The remainder of the data depends on the command.
//  * The maximum data size is 29 bytes.
//  */

// /* To add a new packet:
//  *   1 - Add a new type in the metaCommand_e enum.
//  *   2 - Implement a decoder function with good documentation about the data
//  *       structure and the intent of the packet.
//  *   3 - Add the decoder function to the metaCommandDecoders array.
//  *   4 - Create a new params group for your handler if necessary
//  *   5 - Pull-request your change :-)
//  */

// /* ---===== 1 - metaCommand_e enum =====--- */
// enum metaCommand_e {
//   metaNotifySetpointsStop = 0,
//   nMetaCommands,
// };

// typedef void (*metaCommandDecoder_t)(const void *data, size_t datalen);

// /* ---===== 2 - Decoding functions =====--- */

// /* notifySetpointsStop meta-command. See commander.h function
//  * commanderNotifySetpointsStop() for description and motivation.
//  */
// struct notifySetpointsStopPacket {
//   uint32_t remainValidMillisecs;
// } __attribute__((packed));
// void notifySetpointsStopDecoder(const void *data, size_t datalen)
// {
//   ASSERT(datalen == sizeof(struct notifySetpointsStopPacket));
//   const struct notifySetpointsStopPacket *values = data;
//   commanderNotifySetpointsStop(values->remainValidMillisecs);
// }

//  /* ---===== packetDecoders array =====--- */
// const static metaCommandDecoder_t metaCommandDecoders[] = {
//   [metaNotifySetpointsStop] = notifySetpointsStopDecoder,
// };

// /* Decoder switch */
// static void commanderCrtpCB(CRTPPacket* pk)
// {
//   static setpoint_t setpoint;

//   if(pk->port == CRTP_PORT_SETPOINT && pk->channel == 0) {
//     printf("CRTP commander packet received\n");
//     crtpCommanderRpytDecodeSetpoint(&setpoint, pk);
//     commanderSetSetpoint(&setpoint, COMMANDER_PRIORITY_CRTP);
//   } else if (pk->port == CRTP_PORT_SETPOINT_GENERIC) {
//     switch (pk->channel) {
//     case SET_SETPOINT_CHANNEL:
//       crtpCommanderGenericDecodeSetpoint(&setpoint, pk);
//       commanderSetSetpoint(&setpoint, COMMANDER_PRIORITY_CRTP);
//       break;
//     case META_COMMAND_CHANNEL: {
//         uint8_t metaCmd = pk->data[0];
//         if (metaCmd < nMetaCommands && (metaCommandDecoders[metaCmd] != NULL)) {
//           metaCommandDecoders[metaCmd](pk->data + 1, pk->size - 1);
//         }
//       }
//       break;
//     default:
//       /* Do nothing */
//       break;
//     }
//   }
// }

/**
 *    ||          ____  _ __
 * +------+      / __ )(_) /_______________ _____  ___
 * | 0xBC |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
 * +------+    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
 *  ||  ||    /_____/_/\__/\___/_/   \__,_/ /___/\___/
 *
 * Crazyflie Firmware
 *
 * Copyright (C) 2011-2017 Bitcraze AB
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, in version 3.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program. If not, see <http://www.gnu.org/licenses/>.
 *
 *
 */
#include <stdbool.h>
#include <stddef.h>

#include "crtp_commander.h"

#include "cfassert.h"
#include "commander.h"
#include "crtp.h"

//below added by dks
#include "ledseq.h"
#include "system.h"
#include "platform.h"
#include "debug_cf.h"

#include "FreeRTOS.h"
#include "task.h"

#include "position_controller.h"


#define APP_DISCONNECT_TIMEOUT_MS 5000  //  timeout for app disconnection

static bool isInit;
static bool wasConnected = false;
static bool emergencyLandingActive = false;

static void commanderCrtpCB(CRTPPacket* pk);
static void resetAllCommands();

static TickType_t lastPacketTick = 0;

static void checkAppConnectionTask(void *param) {
  while (1) {
      TickType_t now = xTaskGetTickCount();
      bool isConnected = ((now - lastPacketTick) <= pdMS_TO_TICKS(APP_DISCONNECT_TIMEOUT_MS));

      // Handle disconnection
      if (wasConnected && !isConnected) {
          printf("[WARNING] App Disconnected! Resetting commands and turning off LED.\n");
          ledSet(1, 0);  // Turn OFF LED
          resetAllCommands();
          
          // // If in flight, initiate emergency landing
          // if (commanderGetActivePriority() == COMMANDER_PRIORITY_CRTP) {
          //     emergencyLandingActive = true;
          //     printf("[EMERGENCY] Initiating auto-landing sequence!\n");
          // }
      }
      // Handle reconnection
      else if (!wasConnected && isConnected) {
          printf("[INFO] App Reconnected!\n");
          emergencyLandingActive = false;
      }

      wasConnected = isConnected;

      // If emergency landing is active, handle it
      // if (emergencyLandingActive) {
      //     // Implement your emergency landing logic here
      //     // For example, gradually reduce altitude
      //     printf("[EMERGENCY] Performing auto-landing...\n");
          
      //     // TODO: Add actual landing logic
      //     // Once landed, you can set:
      //     // emergencyLandingActive = false;
      // }

      vTaskDelay(pdMS_TO_TICKS(100));  // Check more frequently (100ms)
  }
}

// Function to reset all commands
static void resetAllCommands() {
    // setpoint_t setpoint;
    // memset(&setpoint, 0, sizeof(setpoint_t)); // Zero out the setpoint
    
    // // Send zero setpoint to stop the drone
    // commanderSetSetpoint(&setpoint, COMMANDER_PRIORITY_DISABLE);

    disarmMotor();
    //isArmed = false;
    isArmsuccess = false;
    armMode = false;
    isTakeOff = false;
    landMode = false;
    takeoff_completed = false;
    land_completed = false;

    printf("[INFO] All commands have been reset to zero\n");
}

void crtpCommanderInit(void)
{
  if(isInit) {
    return;
  }

  crtpInit();
  crtpRegisterPortCB(CRTP_PORT_SETPOINT, commanderCrtpCB);
  crtpRegisterPortCB(CRTP_PORT_SETPOINT_GENERIC, commanderCrtpCB);
  isInit = true;

  // Start the connection monitoring task with higher priority
  xTaskCreate(checkAppConnectionTask, "CheckAppConnection", 1024, NULL, 3, NULL);
}

enum crtpSetpointGenericChannel {
  SET_SETPOINT_CHANNEL = 0,
  META_COMMAND_CHANNEL = 1,
};

/* Channel 1 of the generic commander port is used for "meta-commands"
 * that alter the behavior of the commander itself, e.g. mode switching.
 * Although we use the generic commander port due to increasing pressure on the
 * 4-bit space of ports numbers, meta-commands that are unrelated to
 * streaming generic setpoint control modes are permitted.
 *
 * The packet format for meta-commands is:
 * +------+==========================+
 * | TYPE |     DATA                 |
 * +------+==========================+
 *
 * TYPE is an 8-bit value. The remainder of the data depends on the command.
 * The maximum data size is 29 bytes.
 */

/* To add a new packet:
 *   1 - Add a new type in the metaCommand_e enum.
 *   2 - Implement a decoder function with good documentation about the data
 *       structure and the intent of the packet.
 *   3 - Add the decoder function to the metaCommandDecoders array.
 *   4 - Create a new params group for your handler if necessary
 *   5 - Pull-request your change :-)
 */

/* ---===== 1 - metaCommand_e enum =====--- */
enum metaCommand_e {
  metaNotifySetpointsStop = 0,
  nMetaCommands,
};

typedef void (*metaCommandDecoder_t)(const void *data, size_t datalen);

/* ---===== 2 - Decoding functions =====--- */

/* notifySetpointsStop meta-command. See commander.h function
 * commanderNotifySetpointsStop() for description and motivation.
 */
struct notifySetpointsStopPacket {
  uint32_t remainValidMillisecs;
} __attribute__((packed));
void notifySetpointsStopDecoder(const void *data, size_t datalen)
{
  ASSERT(datalen == sizeof(struct notifySetpointsStopPacket));
  const struct notifySetpointsStopPacket *values = data;
  commanderNotifySetpointsStop(values->remainValidMillisecs);
}

 /* ---===== packetDecoders array =====--- */
const static metaCommandDecoder_t metaCommandDecoders[] = {
  [metaNotifySetpointsStop] = notifySetpointsStopDecoder,
};

/* Decoder switch */
static void commanderCrtpCB(CRTPPacket* pk)
{
  // Update last packet timestamp on any received packet
  lastPacketTick = xTaskGetTickCount();
  
  static setpoint_t setpoint;
  //printf("Received CRTP Packet: Port=%d, Channel=%d, Data=", pk->port, pk->channel);

  if(pk->port == CRTP_PORT_SETPOINT && pk->channel == 0) {
    //printf("Legacy RPYT setpoint\n");
    ledSet(1,1); // Visual feedback for connection
    crtpCommanderRpytDecodeSetpoint(&setpoint, pk);
    commanderSetSetpoint(&setpoint, COMMANDER_PRIORITY_CRTP);
  } else if (pk->port == CRTP_PORT_SETPOINT_GENERIC) {
    printf("Generic commander packet\n");
    switch (pk->channel) {
    case SET_SETPOINT_CHANNEL:
      printf("Setpoint channel\n");
      crtpCommanderGenericDecodeSetpoint(&setpoint, pk);
      commanderSetSetpoint(&setpoint, COMMANDER_PRIORITY_CRTP);
      break;
    case META_COMMAND_CHANNEL: {
        printf("Meta command channel\n");
        uint8_t metaCmd = pk->data[0];
        printf("Meta command: %u\n", metaCmd);
        if (metaCmd == 0x11) { // Custom LED command
          if (pk->data[1] == 1) {
            printf("LED ON command\n");
            ledSet(1, 1);  // Turn LED ON 
          } else if(pk->data[1] == 0) {
            printf("LED OFF command\n");
            ledSet(1, 0);  // Turn LED OFF
          } 
        }
        else if (metaCmd < nMetaCommands && (metaCommandDecoders[metaCmd] != NULL)) {
          metaCommandDecoders[metaCmd](pk->data + 1, pk->size - 1);
        }
        else {
          printf("Unknown meta command: %02X\n", metaCmd);
        }
      }
      break;
    default:
      printf("Unknown channel\n");
      break;
    }
  }
}