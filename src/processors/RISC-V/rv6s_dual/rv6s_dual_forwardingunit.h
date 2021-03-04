#pragma once

#include "../riscv.h"

#include "VSRTL/core/vsrtl_component.h"

namespace Ripes {
Enum(ForwardingSrcDual, IdStage, MemStage, WbStageExec, WbStageMem);
}

namespace vsrtl {
namespace core {
using namespace Ripes;

class ForwardingUnit_DUAL : public Component {
    ForwardingSrcDual calculateForwarding(const VSRTL_VT_U& readidx) {
        if (readidx == 0) {
            return ForwardingSrcDual::IdStage;
        } else if (readidx == mem_reg_wr_idx_exec.uValue() && mem_reg_wr_en_exec.uValue()) {
            return ForwardingSrcDual::MemStage;
        } else if (readidx == wb_reg_wr_idx_exec.uValue() && wb_reg_wr_en_exec.uValue()) {
            return ForwardingSrcDual::WbStageExec;
        } else if (readidx == wb_reg_wr_idx_data.uValue() && wb_reg_wr_en_data.uValue()) {
            return ForwardingSrcDual::WbStageMem;
        }
        return ForwardingSrcDual::IdStage;
    }

public:
    ForwardingUnit_DUAL(std::string name, SimComponent* parent) : Component(name, parent) {
        alu_reg1_fw_ctrl_exec << [=] { return calculateForwarding(id_reg1_idx_exec.uValue()); };
        alu_reg2_fw_ctrl_exec << [=] { return calculateForwarding(id_reg2_idx_exec.uValue()); };
        alu_reg1_fw_ctrl_data << [=] { return calculateForwarding(id_reg1_idx_data.uValue()); };
        alu_reg2_fw_ctrl_data << [=] { return calculateForwarding(id_reg2_idx_data.uValue()); };
    }

    INPUTPORT(id_reg1_idx_data, RV_REGS_BITS);
    INPUTPORT(id_reg2_idx_data, RV_REGS_BITS);
    INPUTPORT(id_reg1_idx_exec, RV_REGS_BITS);
    INPUTPORT(id_reg2_idx_exec, RV_REGS_BITS);

    INPUTPORT(mem_reg_wr_idx_exec, RV_REGS_BITS);
    INPUTPORT(mem_reg_wr_en_exec, 1);

    INPUTPORT(wb_reg_wr_idx_exec, RV_REGS_BITS);
    INPUTPORT(wb_reg_wr_en_exec, 1);

    INPUTPORT(wb_reg_wr_idx_data, RV_REGS_BITS);
    INPUTPORT(wb_reg_wr_en_data, 1);

    OUTPUTPORT_ENUM(alu_reg1_fw_ctrl_exec, ForwardingSrcDual);
    OUTPUTPORT_ENUM(alu_reg2_fw_ctrl_exec, ForwardingSrcDual);

    OUTPUTPORT_ENUM(alu_reg1_fw_ctrl_data, ForwardingSrcDual);
    OUTPUTPORT_ENUM(alu_reg2_fw_ctrl_data, ForwardingSrcDual);
};
}  // namespace core
}  // namespace vsrtl
