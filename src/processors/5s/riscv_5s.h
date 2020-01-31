#pragma once

#include "VSRTL/core/vsrtl_adder.h"
#include "VSRTL/core/vsrtl_constant.h"
#include "VSRTL/core/vsrtl_design.h"
#include "VSRTL/core/vsrtl_logicgate.h"
#include "VSRTL/core/vsrtl_multiplexer.h"

#include "../ripesprocessor.h"

// Functional units
#include "../ss/alu.h"
#include "../ss/branch.h"
#include "../ss/control.h"
#include "../ss/decode.h"
#include "../ss/ecallchecker.h"
#include "../ss/immediate.h"
#include "../ss/registerfile.h"
#include "../ss/riscv.h"
#include "../ss/rvmemory.h"

// Stage separating registers
#include "5s_exmem.h"
#include "5s_idex.h"
#include "5s_ifid.h"
#include "5s_memwb.h"

namespace vsrtl {
namespace core {
using namespace Ripes;

struct FinishingCounter {
    void start(int _target) {
        if (finishing || finished)
            return;  // Already running
        target = _target;
        finishing = true;
        count = 0;
        finished = false;
    }
    void reset() {
        finishing = false;
        finished = false;
    }

    bool finishing = false;
    int count;
    int target;
    bool finished = false;
    void operator++(int) {
        if (!finishing || finished)
            return;
        count++;
        if (count == target) {
            finishing = false;
            finished = true;
        }
    }
    void operator--(int) {
        if (finished) {
            finished = false;
            finishing = true;
        }
        if (!finishing)
            return;
        count--;
        if (count == 0)
            finishing = false;
    }
};

class FiveStageRISCV : public RipesProcessor {
public:
    FiveStageRISCV() : RipesProcessor("5-Stage RISC-V Processor") {
        // -----------------------------------------------------------------------
        // Program counter
        pc_reg->out >> pc_4->op1;
        4 >> pc_4->op2;
        pc_src->out >> pc_reg->in;

        // Note: pc_src works uses the PcSrc enum, but is selected by the boolean signal
        // from the controlflow OR gate. PcSrc enum values must adhere to the boolean
        // 0/1 values.
        controlflow_or->out >> pc_src->select;

        controlflow_or->out >> *efsc_or->in[0];
        ecallChecker->syscallExit >> *efsc_or->in[1];

        // -----------------------------------------------------------------------
        // Instruction memory
        pc_reg->out >> instr_mem->addr;
        instr_mem->setMemory(m_memory);

        // -----------------------------------------------------------------------
        // Decode
        ifid_reg->instr_out >> decode->instr;

        // -----------------------------------------------------------------------
        // Control signals
        decode->opcode >> control->opcode;

        // -----------------------------------------------------------------------
        // Immediate
        decode->opcode >> immediate->opcode;
        ifid_reg->instr_out >> immediate->instr;

        // -----------------------------------------------------------------------
        // Registers
        decode->r1_reg_idx >> registerFile->r1_addr;
        decode->r2_reg_idx >> registerFile->r2_addr;
        reg_wr_src->out >> registerFile->data_in;

        memwb_reg->wr_reg_idx_out >> registerFile->wr_addr;
        memwb_reg->reg_do_write_out >> registerFile->wr_en;
        memwb_reg->mem_read_out >> reg_wr_src->get(RegWrSrc::MEMREAD);
        memwb_reg->alures_out >> reg_wr_src->get(RegWrSrc::ALURES);
        memwb_reg->pc4_out >> reg_wr_src->get(RegWrSrc::PC4);
        memwb_reg->reg_wr_src_ctrl_out >> reg_wr_src->select;

        registerFile->setMemory(m_regMem);

        // -----------------------------------------------------------------------
        // Branch
        idex_reg->br_op_out >> branch->comp_op;
        idex_reg->r1_out >> branch->op1;
        idex_reg->r2_out >> branch->op2;

        branch->res >> *br_and->in[0];
        idex_reg->do_br_out >> *br_and->in[1];
        br_and->out >> *controlflow_or->in[0];
        idex_reg->do_jmp_out >> *controlflow_or->in[1];

        pc_4->out >> pc_src->get(PcSrc::PC4);
        alu->res >> pc_src->get(PcSrc::ALU);

        // -----------------------------------------------------------------------
        // ALU
        idex_reg->r1_out >> alu_op1_src->get(AluSrc1::REG1);
        idex_reg->pc_out >> alu_op1_src->get(AluSrc1::PC);
        idex_reg->alu_op1_ctrl_out >> alu_op1_src->select;

        idex_reg->r2_out >> alu_op2_src->get(AluSrc2::REG2);
        idex_reg->imm_out >> alu_op2_src->get(AluSrc2::IMM);
        idex_reg->alu_op2_ctrl_out >> alu_op2_src->select;

        alu_op1_src->out >> alu->op1;
        alu_op2_src->out >> alu->op2;

        idex_reg->alu_ctrl_out >> alu->ctrl;

        // -----------------------------------------------------------------------
        // Data memory
        exmem_reg->alures_out >> data_mem->addr;
        exmem_reg->mem_do_write_out >> data_mem->wr_en;
        exmem_reg->r2_out >> data_mem->data_in;
        exmem_reg->mem_op_out >> data_mem->op;
        data_mem->mem->setMemory(m_memory);

        // -----------------------------------------------------------------------
        // Ecall checker
        decode->opcode >> ecallChecker->opcode;
        ecallChecker->setSysCallSignal(&handleSysCall);

        // -----------------------------------------------------------------------
        // IF/ID
        pc_4->out >> ifid_reg->pc4_in;
        pc_reg->out >> ifid_reg->pc_in;
        instr_mem->data_out >> ifid_reg->instr_in;
        1 >> ifid_reg->enable;
        efsc_or->out >> ifid_reg->clear;
        1 >> ifid_reg->valid_in;  // Always valid unless register is cleared

        // -----------------------------------------------------------------------
        // ID/EX
        1 >> idex_reg->enable;
        controlflow_or->out >> idex_reg->clear;

        // Data
        ifid_reg->pc4_out >> idex_reg->pc4_in;
        ifid_reg->pc_out >> idex_reg->pc_in;
        registerFile->r1_out >> idex_reg->r1_in;
        registerFile->r2_out >> idex_reg->r2_in;
        immediate->imm >> idex_reg->imm_in;

        // Control
        decode->wr_reg_idx >> idex_reg->wr_reg_idx_in;
        control->reg_wr_src_ctrl >> idex_reg->reg_wr_src_ctrl_in;
        control->reg_do_write_ctrl >> idex_reg->reg_do_write_in;
        control->alu_op1_ctrl >> idex_reg->alu_op1_ctrl_in;
        control->alu_op2_ctrl >> idex_reg->alu_op2_ctrl_in;
        control->mem_do_write_ctrl >> idex_reg->mem_do_write_in;
        control->alu_ctrl >> idex_reg->alu_ctrl_in;
        control->mem_ctrl >> idex_reg->mem_op_in;
        control->comp_ctrl >> idex_reg->br_op_in;
        control->do_branch >> idex_reg->do_br_in;
        control->do_jump >> idex_reg->do_jmp_in;

        ifid_reg->valid_out >> idex_reg->valid_in;

        // -----------------------------------------------------------------------
        // EX/MEM

        // Data
        idex_reg->pc_out >> exmem_reg->pc_in;
        idex_reg->pc4_out >> exmem_reg->pc4_in;
        idex_reg->r2_out >> exmem_reg->r2_in;
        alu->res >> exmem_reg->alures_in;

        // Control
        idex_reg->reg_wr_src_ctrl_out >> exmem_reg->reg_wr_src_ctrl_in;
        idex_reg->wr_reg_idx_out >> exmem_reg->wr_reg_idx_in;
        idex_reg->reg_do_write_out >> exmem_reg->reg_do_write_in;
        idex_reg->mem_do_write_out >> exmem_reg->mem_do_write_in;
        idex_reg->mem_op_out >> exmem_reg->mem_op_in;

        idex_reg->valid_out >> exmem_reg->valid_in;

        // -----------------------------------------------------------------------
        // MEM/WB

        // Data
        exmem_reg->pc_out >> memwb_reg->pc_in;
        exmem_reg->pc4_out >> memwb_reg->pc4_in;
        exmem_reg->alures_out >> memwb_reg->alures_in;
        data_mem->data_out >> memwb_reg->mem_read_in;

        // Control
        exmem_reg->reg_wr_src_ctrl_out >> memwb_reg->reg_wr_src_ctrl_in;
        exmem_reg->wr_reg_idx_out >> memwb_reg->wr_reg_idx_in;
        exmem_reg->reg_do_write_out >> memwb_reg->reg_do_write_in;

        exmem_reg->valid_out >> memwb_reg->valid_in;
    }

    // Design subcomponents
    SUBCOMPONENT(registerFile, RegisterFile);
    SUBCOMPONENT(alu, ALU);
    SUBCOMPONENT(control, Control);
    SUBCOMPONENT(immediate, Immediate);
    SUBCOMPONENT(decode, Decode);
    SUBCOMPONENT(branch, Branch);
    SUBCOMPONENT(pc_4, Adder<RV_REG_WIDTH>);

    // Registers
    SUBCOMPONENT(pc_reg, Register<RV_REG_WIDTH>);

    // Stage seperating registers
    SUBCOMPONENT(ifid_reg, IFID);
    SUBCOMPONENT(idex_reg, IDEX);
    SUBCOMPONENT(exmem_reg, EXMEM);
    SUBCOMPONENT(memwb_reg, MEMWB);

    // Multiplexers
    SUBCOMPONENT(reg_wr_src, TYPE(EnumMultiplexer<RegWrSrc, RV_REG_WIDTH>));
    SUBCOMPONENT(pc_src, TYPE(EnumMultiplexer<PcSrc, RV_REG_WIDTH>));
    SUBCOMPONENT(alu_op1_src, TYPE(EnumMultiplexer<AluSrc1, RV_REG_WIDTH>));
    SUBCOMPONENT(alu_op2_src, TYPE(EnumMultiplexer<AluSrc2, RV_REG_WIDTH>));

    // Memories
    SUBCOMPONENT(instr_mem, TYPE(ROM<RV_REG_WIDTH, RV_INSTR_WIDTH>));
    SUBCOMPONENT(data_mem, TYPE(RVMemory<RV_REG_WIDTH, RV_REG_WIDTH>));

    // Gates
    // True if branch instruction and branch taken
    SUBCOMPONENT(br_and, TYPE(And<1, 2>));
    // True if branch taken or jump instruction
    SUBCOMPONENT(controlflow_or, TYPE(Or<1, 2>));
    // True if controlflow action or performing syscall finishing
    SUBCOMPONENT(efsc_or, TYPE(Or<1, 2>));

    // Address spaces
    ADDRESSSPACE(m_memory);
    ADDRESSSPACE(m_regMem);

    SUBCOMPONENT(ecallChecker, EcallChecker);

    // Ripes interface compliance
    virtual const ISAInfoBase* implementsISA() const override { return ISAInfo<ISA::RV32IM>::instance(); }
    unsigned int stageCount() const override { return 5; }
    unsigned int getPcForStage(unsigned int idx) const override {
        // clang-format off
        switch (idx) {
            case 0: return pc_reg->out.uValue();
            case 1: return ifid_reg->pc_out.uValue();
            case 2: return idex_reg->pc_out.uValue();
            case 3: return exmem_reg->pc_out.uValue();
            case 4: return memwb_reg->pc_out.uValue();
            default: assert(false && "Processor does not contain stage");
        }
        // clang-format on
    }
    unsigned int nextFetchedAddress() const override { return pc_src->out.uValue(); }
    QString stageName(unsigned int idx) const override {
        // clang-format off
        switch (idx) {
            case 0: return "IF";
            case 1: return "ID";
            case 2: return "EX";
            case 3: return "MEM";
            case 4: return "WB";
            default: assert(false && "Processor does not contain stage");
        }
        // clang-format on
    }
    StageInfo stageInfo(unsigned int idx) const override {
        bool stageValid = true;
        // Has the pipeline stage been filled?
        stageValid &= idx <= m_cycleCount;

        // clang-format off
        // Has the stage been cleared?
        switch(idx){
        case 1: stageValid &= ifid_reg->valid_out.uValue(); break;
        case 2: stageValid &= idex_reg->valid_out.uValue(); break;
        case 3: stageValid &= exmem_reg->valid_out.uValue(); break;
        case 4: stageValid &= memwb_reg->valid_out.uValue(); break;
        default: case 0: break;
        }

        // Is the stage carrying a valid (executable) PC?
        switch(idx){
        case 1: stageValid &= isExecutableAddress(ifid_reg->pc_out.uValue()); break;
        case 2: stageValid &= isExecutableAddress(idex_reg->pc_out.uValue()); break;
        case 3: stageValid &= isExecutableAddress(exmem_reg->pc_out.uValue()); break;
        case 4: stageValid &= isExecutableAddress(memwb_reg->pc_out.uValue()); break;
        default: case 0: stageValid &= isExecutableAddress(pc_reg->out.uValue()); break;
        }

        // Are we currently clearing the pipeline due to a syscall exit? if such, the first stage (IF) is never valid
        if(idx == 0){
            stageValid &= !ecallChecker->isSysCallExiting();
        }

        // clang-format on
        return StageInfo({getPcForStage(idx), stageValid});
    }

    void setProgramCounter(uint32_t address) override {
        pc_reg->forceValue(0, address);
        propagateDesign();
    }
    void setPCInitialValue(uint32_t address) override { pc_reg->setInitValue(address); }
    SparseArray& getMemory() override { return *m_memory; }
    unsigned int getRegister(unsigned i) const override { return registerFile->getRegister(i); }
    SparseArray& getRegisters() override { return *m_regMem; }
    void finalize(const FinalizeReason& fr) override {
        // If we have received an exit syscall, the finishing sequence cannot be stopped, and the processor will empty
        // its pipeline. Else, a finalize sequence may occur through deassertion of the finalization reason(s).
        ecallChecker->setSysCallExiting(ecallChecker->isSysCallExiting() || fr.exitSyscall);
        if (ecallChecker->isSysCallExiting() || fr.any()) {
            m_fcntr.start(5);
        } else {
            m_fcntr.reset();
        }
    }
    bool finished() const override { return m_fcntr.finished; }

    void setRegister(unsigned i, uint32_t v) override { setSynchronousValue(registerFile->_wr_mem, i, v); }

    void clock() override {
        // An instruction has been retired if the instruction in the WB stage is valid and the PC is within the
        // executable range of the program
        if (memwb_reg->valid_out.uValue() != 0 && isExecutableAddress(memwb_reg->pc_out.uValue())) {
            m_instructionsRetired++;
        }

        RipesProcessor::clock();
        m_fcntr++;
    }

    void reverse() override {
        RipesProcessor::reverse();
        if (memwb_reg->valid_out.uValue() != 0 && isExecutableAddress(memwb_reg->pc_out.uValue())) {
            m_instructionsRetired--;
        }

        m_fcntr--;
    }

    void reset() override {
        RipesProcessor::reset();
        m_fcntr.reset();
        ecallChecker->setSysCallExiting(false);
    }

private:
    FinishingCounter m_fcntr;
};

}  // namespace core
}  // namespace vsrtl
