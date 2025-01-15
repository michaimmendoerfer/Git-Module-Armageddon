JSON Send-Structure Armageddon

SEND_CMD_PAIR_ME
Node       = "NodeName;Uptime;Status"
Type       =  Module.GetType()
Version    =  Module.GetVersion()
Order      = SEND_CMD_PAIR_ME
  Periph0  = "Type;Name"                                Type: SENS_TYPE_SWITCH, SENS_TYPE_AMP, SENS_TYPE_LT_AMP...
  Periph1  = "Type;Name"

SEND_CMD_STATUS (1/s)
Node       = "NodeName;Uptime;Status"
Order      = SEND_CMD_STATUS
  Periph0  = "Type;Name;Value0;Value1;Value2;Value3"
  Periph1  = "Type;Name;Value0;Value1;Value2;Value3"

SEND_CONFIRM (if received message contains a TSConfirm-Timestamp (i.e. Toggle-Command))
Node       = "NodeName;Uptime;Status"
Order      = SEND_CMD_CONFIRM
TSConfirm  = TSConfirm 


JSON Receive-Structure 

SEND_CMD_SWITCH_ON
From        = Peer from which order was sent
Order       = SEND_CMD_SWITCH_ON
PeriphName  = name of switch
Pos         = pos of switch 

SEND_CMD_SWITCH_OFF
From        = Peer from which order was sent
Order       = SEND_CMD_SWITCH_OFF
PeriphName  = name of switch
Pos         = pos of switch 

SEND_CMD_SWITCH_TOGGLE
From        = Peer from which order was sent
Order       = SEND_CMD_SWITCH_TOGGLE
PeriphName  = name of switch
Pos         = pos of switch 

SEND_CMD_CURRENT_CALIB
From        = Peer from which order was sent
Order       = SEND_CMD_CURRENT_CALIB

SEND_CMD_VOLTAGE_CALIB
From        = Peer from which order was sent
Order       = SEND_CMD_VOLTAGE_CALIB