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

SEND_CONFIRM
Node       = "NodeName;Uptime;Status"
Order      = SEND_CMD_CONFIRM
TSConfirm  = TSConfirm                              
