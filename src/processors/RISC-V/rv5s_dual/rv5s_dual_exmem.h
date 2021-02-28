#pragma once

#include "../rv5s/rv5s_exmem.h"

namespace vsrtl {
namespace core {
using namespace Ripes;

class RV5S_EXMEM_DUAL : public RV5S_EXMEM {
public:
    RV5S_EXMEM_DUAL(std::string name, SimComponent* parent) : RV5S_EXMEM(name, parent) {
        CONNECT_REGISTERED_CLEN_INPUT(wr_reg_idx_data, clear, enable);
        CONNECT_REGISTERED_CLEN_INPUT(reg_do_write_data, clear, enable);
        CONNECT_REGISTERED_INPUT(reg_wr_src_ctrl_dual);
    }

    REGISTERED_CLEN_INPUT(wr_reg_idx_data, RV_REGS_BITS);
    REGISTERED_CLEN_INPUT(reg_do_write_data, 1);
    REGISTERED_INPUT(reg_wr_src_ctrl_dual, RegWrSrcDual::width());
};

}  // namespace core
}  // namespace vsrtl
