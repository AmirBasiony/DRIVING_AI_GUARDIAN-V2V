

#ifndef __IPCMSG__
#define __IPCMSG__
#include "../Algo/PositionCalculations.h"




// uint16 receiveMessage_from_PY(uint32 msgid, uint8 *message_str);
void send_message_to_PY(uint32 msgid,long mtype, uint16 *message);

#endif