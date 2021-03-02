#pragma once

#include "../riscv.h"
#include "../rv_control.h"
#include "VSRTL/core/vsrtl_component.h"

namespace vsrtl {
namespace core {
using namespace Ripes;

class WayControl : public Component {
private:
    enum class WayClass { Data, Exec };

    static bool isControlflow(const VSRTL_VT_U& opcode) {
        return Control::do_jump_ctrl(opcode) || Control::do_branch_ctrl(opcode);
    }

    static bool isLoadStore(const VSRTL_VT_U& opcode) { return Control::do_mem_ctrl(opcode) != MemOp::NOP; }

    // clang-format off
    static bool isWriteRegInstr(const VSRTL_VT_U& opcode) {
        switch(opcode) {
            case RVInstr::LUI:
            case RVInstr::AUIPC:

            // Arithmetic-immediate instructions
            case RVInstr::ADDI: case RVInstr::SLTI: case RVInstr::SLTIU: case RVInstr::XORI:
            case RVInstr::ORI: case RVInstr::ANDI: case RVInstr::SLLI: case RVInstr::SRLI:
            case RVInstr::SRAI:

            // Arithmetic instructions
            case RVInstr::MUL: case RVInstr::MULH: case RVInstr:: MULHSU: case RVInstr::MULHU:
            case RVInstr::DIV: case RVInstr::DIVU: case RVInstr::REM: case RVInstr::REMU:
            case RVInstr::ADD: case RVInstr::SUB: case RVInstr::SLL: case RVInstr::SLT:
            case RVInstr::SLTU: case RVInstr::XOR: case RVInstr::SRL: case RVInstr::SRA:
            case RVInstr::OR: case RVInstr::AND:

            // Load instructions
            case RVInstr::LB: case RVInstr::LH: case RVInstr::LW: case RVInstr::LBU: case RVInstr::LHU:

            // Jump instructions
            case RVInstr::JALR:
            case RVInstr::JAL:
                return true;
            default: return false;
        }
    }
    // clang-format on

    bool warHazard() const {
        const unsigned wridx_1 = wr_reg_idx_way1.uValue();
        const unsigned wridx_2 = wr_reg_idx_way2.uValue();

        const unsigned idx1_1 = r1_reg_idx_way1.uValue();
        const unsigned idx2_1 = r2_reg_idx_way1.uValue();
        const unsigned idx1_2 = r1_reg_idx_way2.uValue();
        const unsigned idx2_2 = r2_reg_idx_way2.uValue();

        // way1 write => way 2 read?
        const bool hazard_1 = (wridx_1 == idx1_2 || wridx_1 == idx2_2) && isWriteRegInstr(opcode_way1.uValue());
        // way2 write => way 1 read?
        const bool hazard_2 = (wridx_2 == idx1_1 || wridx_2 == idx2_1) && isWriteRegInstr(opcode_way1.uValue());

        return hazard_1 || hazard_2;
    }

    /**
     * @brief computeCycle
     * Computes internal information which may be used in the calculation of all output ports of this component.
     */
    void computeCycle() {
        if (m_design->getCycleCount() == cachedCycle)
            return;

        // Default assignments
        WayClass way1Type = isLoadStore(opcode_way1.uValue()) ? WayClass::Data : WayClass::Exec;
        WayClass way2Type = isLoadStore(opcode_way2.uValue()) ? WayClass::Data : WayClass::Exec;
        m_pcaddSrc = PcSrcDual::PC8;

        if (isControlflow(opcode_way1.uValue())) {
            m_pcaddSrc = PcSrcDual::PC4;
            m_dataWayValid = false;
            m_execWayValid = true;
            m_execWaySrc = WaySrc::WAY1;
        } else if (way1Type == way2Type) {
            // Always schedule way 1 instruction (execute in-order)
            m_dataWayValid = way1Type == WayClass::Data;
            m_dataWaySrc = WaySrc::WAY1;
            m_execWayValid = way1Type == WayClass::Exec;
            m_execWaySrc = WaySrc::WAY1;
            m_pcaddSrc = PcSrcDual::PC4;
        } else if (warHazard()) {
            // WAR hazard
            m_dataWayValid = way1Type == WayClass::Data;
            m_execWayValid = way1Type == WayClass::Exec;
            m_dataWaySrc = WaySrc::WAY1;
            m_execWaySrc = WaySrc::WAY1;
            m_pcaddSrc = PcSrcDual::PC4;
        } else {
            // Can schedule both
            m_dataWayValid = true;
            m_execWayValid = true;
            m_dataWaySrc = way1Type == WayClass::Data ? WaySrc::WAY1 : WaySrc::WAY2;
            m_execWaySrc = way1Type == WayClass::Exec ? WaySrc::WAY1 : WaySrc::WAY2;
            Q_ASSERT(m_dataWaySrc != m_execWaySrc);
        }

        cachedCycle = m_design->getCycleCount();
    }

public:
    WayControl(std::string name, SimComponent* parent) : Component(name, parent) {
        data_way_valid << [=] {
            computeCycle();
            return m_dataWayValid;
        };
        exec_way_valid << [=] {
            computeCycle();
            return m_execWayValid;
        };
        data_way_src << [=] {
            computeCycle();
            return m_dataWaySrc;
        };
        exec_way_src << [=] {
            computeCycle();
            return m_execWaySrc;
        };

        pcadd_src << [=] {
            computeCycle();
            return m_pcaddSrc;
        };

        m_design = getDesign();
        Q_ASSERT(m_design != nullptr);
    }

    INPUTPORT_ENUM(opcode_way1, RVInstr);
    INPUTPORT_ENUM(opcode_way2, RVInstr);

    INPUTPORT(r1_reg_idx_way1, RV_REGS_BITS);
    INPUTPORT(r2_reg_idx_way1, RV_REGS_BITS);
    INPUTPORT(r1_reg_idx_way2, RV_REGS_BITS);
    INPUTPORT(r2_reg_idx_way2, RV_REGS_BITS);

    INPUTPORT(wr_reg_idx_way1, RV_REGS_BITS);
    INPUTPORT(wr_reg_idx_way2, RV_REGS_BITS);

    OUTPUTPORT_ENUM(data_way_src, WaySrc);
    OUTPUTPORT_ENUM(exec_way_src, WaySrc);

    OUTPUTPORT(exec_way_valid, 1);
    OUTPUTPORT(data_way_valid, 1);

    OUTPUTPORT_ENUM(pcadd_src, PcSrcDual);

private:
    SimDesign* m_design;

    unsigned cachedCycle = -1;

    // Cached data for next-state computation
    bool m_dataWayValid;
    bool m_execWayValid;
    WaySrc m_execWaySrc = WaySrc::WAY1;
    WaySrc m_dataWaySrc = WaySrc::WAY1;
    PcSrcDual m_pcaddSrc = PcSrcDual::PC8;
};

}  // namespace core
}  // namespace vsrtl
