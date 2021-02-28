#pragma once

#include "../rv5s/rv5s_memwb.h"

namespace vsrtl {
namespace core {
using namespace Ripes;

class RV5S_MEMWB_DUAL : public RV5S_MEMWB {
public:
    RV5S_MEMWB_DUAL(std::string name, SimComponent* parent) : RV5S_MEMWB(name, parent) {
        CONNECT_REGISTERED_INPUT(wr_reg_idx_data);
        CONNECT_REGISTERED_INPUT(reg_do_write_data);
        CONNECT_REGISTERED_INPUT(reg_wr_src_ctrl_dual);
    }

    REGISTERED_INPUT(wr_reg_idx_data, RV_REGS_BITS);
    REGISTERED_INPUT(reg_do_write_data, 1);
    REGISTERED_INPUT(reg_wr_src_ctrl_dual, RegWrSrcDual::width());
};

}  // namespace core
}  // namespace vsrtl
