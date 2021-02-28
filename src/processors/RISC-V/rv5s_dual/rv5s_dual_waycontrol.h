#pragma once

#include "../riscv.h"
#include "../rv_control.h"
#include "VSRTL/core/vsrtl_component.h"

namespace vsrtl {
namespace core {
using namespace Ripes;

class WayControl : public Component {
public:
    static bool is_controlflow(const VSRTL_VT_U& opcode) {
        return Control::do_jump_ctrl(opcode) || Control::do_branch_ctrl(opcode);
    }

    bool stall_exec() const {
        // Load-use
    }

    bool stall_data() const {
        // Do not issue anything in data slot during control flow
        if (is_controlflow(opcode_way1.uValue())) {
            return true;
        }
    }

public:
    WayControl(std::string name, SimComponent* parent) : Component(name, parent) {
        data_way_valid << [=] { return !stall_data(); };
    }

    INPUTPORT_ENUM(opcode_way1, RVInstr);
    INPUTPORT_ENUM(opcode_way2, RVInstr);

    INPUTPORT(r1_reg_idx_way1, RV_REGS_BITS);
    INPUTPORT(r2_reg_idx_way1, RV_REGS_BITS);
    INPUTPORT(r1_reg_idx_way2, RV_REGS_BITS);
    INPUTPORT(r2_reg_idx_way2, RV_REGS_BITS);

    OUTPUTPORT(exec_way_valid, 1);
    OUTPUTPORT(data_way_valid, 1);
};

}  // namespace core
}  // namespace vsrtl
