#include "vcu_board_io.h"

#include "board.h"
#include "vcfg_protocol.h"

void vcu_board_route_compute_interface(uint8_t compute_interface_id)
{
    if (compute_interface_id == VCFG_COMPUTE_INTERFACE_SERIAL) {
        init_uart13_pins();
        return;
    }

    if (compute_interface_id == VCFG_COMPUTE_INTERFACE_CAN
        || compute_interface_id == VCFG_COMPUTE_INTERFACE_SOCKETCAN) {
        HPM_IOC->PAD[IOC_PAD_PE31].FUNC_CTL = IOC_PE31_FUNC_CTL_CAN1_TXD;
        HPM_IOC->PAD[IOC_PAD_PE30].FUNC_CTL = IOC_PE30_FUNC_CTL_CAN1_RXD;
    }
}
