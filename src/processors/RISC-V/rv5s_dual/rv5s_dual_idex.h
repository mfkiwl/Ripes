#pragma once

#include "../rv5s/rv5s_idex.h"

namespace vsrtl {
namespace core {
using namespace Ripes;

/**
 * @brief The RV5S_IDEX class
 * A specialization of the default IDEX stage separating register utilized by the rv5s_no_fw_hz processor. Storage of
 * register read indices is added, which are required by the forwarding unit.
 */
class RV5S_IDEX_DUAL : public RV5S_IDEX {
public:
    RV5S_IDEX_DUAL(std::string name, SimComponent* parent) : RV5S_IDEX(name, parent) {
        CONNECT_REGISTERED_CLEN_INPUT(r1_data, clear, enable);
        CONNECT_REGISTERED_CLEN_INPUT(r2_data, clear, enable);

        CONNECT_REGISTERED_CLEN_INPUT(wr_reg_idx_data, clear, enable);
        CONNECT_REGISTERED_CLEN_INPUT(reg_do_write_data, clear, enable);

        CONNECT_REGISTERED_CLEN_INPUT(alu_op1_ctrl_data, clear, enable);
        CONNECT_REGISTERED_CLEN_INPUT(alu_op2_ctrl_data, clear, enable);
        CONNECT_REGISTERED_CLEN_INPUT(alu_ctrl_data, clear, enable);
        CONNECT_REGISTERED_CLEN_INPUT(imm_data, clear, enable);
        CONNECT_REGISTERED_CLEN_INPUT(reg_wr_src_ctrl_dual, clear, enable);
        CONNECT_REGISTERED_CLEN_INPUT(pc_data, clear, enable);
        CONNECT_REGISTERED_CLEN_INPUT(rd_reg1_idx_data, clear, enable);
        CONNECT_REGISTERED_CLEN_INPUT(rd_reg2_idx_data, clear, enable);
        CONNECT_REGISTERED_CLEN_INPUT(exec_valid, clear, enable);
        CONNECT_REGISTERED_CLEN_INPUT(data_valid, clear, enable);
        CONNECT_REGISTERED_CLEN_INPUT(way_stall, clear, enable);
    }

    REGISTERED_CLEN_INPUT(pc_data, RV_REG_WIDTH);

    REGISTERED_CLEN_INPUT(r1_data, RV_REG_WIDTH);
    REGISTERED_CLEN_INPUT(r2_data, RV_REG_WIDTH);

    REGISTERED_CLEN_INPUT(rd_reg1_idx_data, RV_REGS_BITS);
    REGISTERED_CLEN_INPUT(rd_reg2_idx_data, RV_REGS_BITS);

    REGISTERED_CLEN_INPUT(wr_reg_idx_data, RV_REGS_BITS);
    REGISTERED_CLEN_INPUT(reg_do_write_data, 1);
    REGISTERED_CLEN_INPUT(reg_wr_src_ctrl_dual, RegWrSrcDual::width());

    REGISTERED_CLEN_INPUT(imm_data, RV_REG_WIDTH);

    REGISTERED_CLEN_INPUT(alu_op1_ctrl_data, AluSrc1::width());
    REGISTERED_CLEN_INPUT(alu_op2_ctrl_data, AluSrc2::width());
    REGISTERED_CLEN_INPUT(alu_ctrl_data, ALUOp::width());

    REGISTERED_CLEN_INPUT(exec_valid, 1);
    REGISTERED_CLEN_INPUT(data_valid, 1);

    REGISTERED_CLEN_INPUT(way_stall, 1);
};

}  // namespace core
}  // namespace vsrtl
