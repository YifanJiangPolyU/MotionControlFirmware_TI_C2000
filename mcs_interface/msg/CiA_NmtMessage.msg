# possible values for Mode:
#
#    NMT_TO_OP          0x01
#    NMT_TO_STOP        0x02
#    NMT_TO_PREOP       0x80
#    NMT_RESET          0x81
#    NMT_RESET_COMM     0x82
#    NMT_CLSW           0xA0
#    NMT_PLSW           0xA1
#    NMT_POLARITY       0xA2
#
# see CANOpenDataTypeDef.h for details

uint8  NodeID     # ID of the target node
uint8  Mode       # target mode, see above for definition
