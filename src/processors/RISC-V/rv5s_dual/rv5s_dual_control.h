#pragma once

#include "../riscv.h"
#include "../rv_control.h"
#include "VSRTL/core/vsrtl_component.h"

namespace vsrtl {
namespace core {
using namespace Ripes;

class Control_2I : public Component {
public:
    Control_2I(std::string name, SimComponent* parent) : Component(name, parent) {
        comp_ctrl << [=] { return Control::do_comp_ctrl(opcode_exec.uValue()); };
        do_branch << [=] { return Control::do_branch_ctrl(opcode_exec.uValue()); };
        do_jump << [=] { return Control::do_jump_ctrl(opcode_exec.uValue()); };
        mem_ctrl << [=] { return Control::do_mem_ctrl(opcode_data.uValue()); };

        reg_do_write_ctrl_exec << [=] { return Control::do_reg_wr_src_ctrl(opcode_exec.uValue()); };
        reg_do_write_ctrl_data << [=] { return Control::do_reg_wr_src_ctrl(opcode_data.uValue()); };

        reg_wr_src_ctrl << [=] { return Control::do_reg_wr_src_ctrl(opcode_exec.uValue()); };

        alu_op1_ctrl_exec << [=] { return Control::do_alu_op1_ctrl(opcode_exec.uValue()); };
        alu_op2_ctrl_exec << [=] { return Control::do_alu_op2_ctrl(opcode_exec.uValue()); };
        alu_ctrl_exec << [=] { return Control::do_alu_ctrl(opcode_exec.uValue()); };

        alu_op1_ctrl_data << [=] { return Control::do_alu_op1_ctrl(opcode_data.uValue()); };
        alu_op2_ctrl_data << [=] { return Control::do_alu_op2_ctrl(opcode_data.uValue()); };
        alu_ctrl_data << [=] { return Control::do_alu_ctrl(opcode_data.uValue()); };

        mem_do_write_ctrl << [=] { return Control::do_do_mem_write_ctrl(opcode_data.uValue()); };
        mem_do_read_ctrl << [=] { return Control::do_do_read_ctrl(opcode_data.uValue()); };
    }

    INPUTPORT_ENUM(opcode_exec, RVInstr);
    INPUTPORT_ENUM(opcode_data, RVInstr);

    OUTPUTPORT(reg_do_write_ctrl_exec, 1);
    OUTPUTPORT_ENUM(reg_wr_src_ctrl, RegWrSrcDual);

    OUTPUTPORT(reg_do_write_ctrl_data, 1);

    OUTPUTPORT(mem_do_write_ctrl, 1);
    OUTPUTPORT(mem_do_read_ctrl, 1);
    OUTPUTPORT(do_branch, 1);
    OUTPUTPORT(do_jump, 1);
    OUTPUTPORT_ENUM(comp_ctrl, CompOp);
    OUTPUTPORT_ENUM(mem_ctrl, MemOp);

    OUTPUTPORT_ENUM(alu_op1_ctrl_exec, AluSrc1);
    OUTPUTPORT_ENUM(alu_op2_ctrl_exec, AluSrc2);
    OUTPUTPORT_ENUM(alu_ctrl_exec, ALUOp);

    OUTPUTPORT_ENUM(alu_op1_ctrl_data, AluSrc1);
    OUTPUTPORT_ENUM(alu_op2_ctrl_data, AluSrc2);
    OUTPUTPORT_ENUM(alu_ctrl_data, ALUOp);
};

}  // namespace core
}  // namespace vsrtl
