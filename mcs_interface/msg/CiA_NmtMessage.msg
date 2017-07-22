# possible values for Mode:
#
# NMT_SWITCH_ON      0x10
# NMT_SWITCH_OFF     0x11
# NMT_ENABLE_OP      0x12
# NMT_DISABLE_OP     0x13
# NMT_QUICK_STOP     0x14
# NME_RESET_FAULT    0x15
# NMT_RESET_NODE     0x80
# NMT_RESET_COMM     0x81
#
# see CANOpenDataTypeDef.h for details

uint8  NodeID     # ID of the target node
uint8  State      # target state, see above for definition
