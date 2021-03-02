#pragma once

#include "VSRTL/core/vsrtl_adder.h"
#include "VSRTL/core/vsrtl_constant.h"
#include "VSRTL/core/vsrtl_design.h"
#include "VSRTL/core/vsrtl_logicgate.h"
#include "VSRTL/core/vsrtl_multiplexer.h"

#include "../../ripesprocessor.h"

#include "../riscv.h"
#include "../rv_alu.h"
#include "../rv_control.h"
#include "../rv_decode.h"
#include "../rv_ecallchecker.h"
#include "../rv_immediate.h"
#include "../rv_memory.h"

// Specialized dual-issue components
#include "rv5s_dual_control.h"
#include "rv5s_dual_instr_mem.h"
#include "rv5s_dual_registerfile.h"
#include "rv5s_dual_waycontrol.h"

// Stage separating registers
#include "rv5s_dual_branch.h"
#include "rv5s_dual_exmem.h"
#include "rv5s_dual_idex.h"
#include "rv5s_dual_ifid.h"
#include "rv5s_dual_memwb.h"

// Forwarding & Hazard detection unit
#include "rv5s_dual_forwardingunit.h"
#include "rv5s_dual_hazardunit.h"

namespace vsrtl {
namespace core {
using namespace Ripes;

class RV5S_DUAL : public RipesProcessor {
public:
    enum Stage { IF_1, IF_2, ID_1, ID_2, EX_EXEC, EX_DATA, MEM_EXEC, MEM_DATA, WB_EXEC, WB_DATA, STAGECOUNT };
    RV5S_DUAL(const QStringList& extensions) : RipesProcessor("5-Stage Static Dual-issue RISC-V Processor") {
        m_enabledISA = std::make_shared<ISAInfo<ISA::RV32I>>(extensions);
        decode_way2->setISA(m_enabledISA);
        decode_way1->setISA(m_enabledISA);

        // -----------------------------------------------------------------------
        // Program counter
        pc_reg->out >> pc_8->op1;
        8 >> pc_8->op2;
        pc_reg->out >> pc_4->op1;
        4 >> pc_4->op2;

        pc_src->out >> pc_reg->in;
        0 >> pc_reg->clear;
        hzunit->hazardFEEnable >> pc_reg->enable;

        waycontrol->pcadd_src >> pcadd_src->select;
        pc_4->out >> pcadd_src->get(PcSrcDual::PC4);
        pc_8->out >> pcadd_src->get(PcSrcDual::PC8);
        alu->res >> pc_src->get(PcSrc::ALU);
        pcadd_src->out >> pc_src->get(PcSrc::PC4);

        // Note: pc_src works uses the PcSrc enum, but is selected by the boolean signal
        // from the controlflow OR gate. PcSrc enum values must adhere to the boolean
        // 0/1 values.
        branch->pc_src >> pc_src->select;

        branch->did_controlflow >> *efsc_or->in[0];
        ecallChecker->syscallExit >> *efsc_or->in[1];

        efsc_or->out >> *efschz_or->in[0];
        hzunit->hazardIDEXClear >> *efschz_or->in[1];

        // -----------------------------------------------------------------------
        // Instruction memory
        pc_reg->out >> instr_mem->addr;
        instr_mem->setMemory(m_memory);

        // -----------------------------------------------------------------------
        // Decode
        ifid_reg->instr_out >> decode_way1->instr;
        ifid_reg->instr2_out >> decode_way2->instr;

        // -----------------------------------------------------------------------
        // Control signals
        exec_way_opcode->out >> control->opcode_exec;
        data_way_opcode->out >> control->opcode_data;

        // -----------------------------------------------------------------------
        // Immediate
        exec_way_opcode->out >> imm_exec->opcode;
        exec_way_instr->out >> imm_exec->instr;

        data_way_opcode->out >> imm_data->opcode;
        data_way_instr->out >> imm_data->instr;

        // -----------------------------------------------------------------------
        // Way control
        decode_way1->opcode >> waycontrol->opcode_way1;
        decode_way1->wr_reg_idx >> waycontrol->wr_reg_idx_way1;
        decode_way1->r1_reg_idx >> waycontrol->r1_reg_idx_way1;
        decode_way1->r2_reg_idx >> waycontrol->r2_reg_idx_way1;

        decode_way2->opcode >> waycontrol->opcode_way2;
        decode_way2->wr_reg_idx >> waycontrol->wr_reg_idx_way2;
        decode_way2->r1_reg_idx >> waycontrol->r1_reg_idx_way2;
        decode_way2->r2_reg_idx >> waycontrol->r2_reg_idx_way2;

        // -----------------------------------------------------------------------
        // Way selection multiplexers
        waycontrol->data_way_src >> data_way_instr->select;
        ifid_reg->instr_out >> data_way_instr->get(WaySrc::WAY1);
        ifid_reg->instr2_out >> data_way_instr->get(WaySrc::WAY2);

        waycontrol->data_way_src >> data_way_opcode->select;
        decode_way1->opcode >> data_way_opcode->get(WaySrc::WAY1);
        decode_way2->opcode >> data_way_opcode->get(WaySrc::WAY2);

        waycontrol->data_way_src >> data_way_r1_reg_idx->select;
        decode_way1->r1_reg_idx >> data_way_r1_reg_idx->get(WaySrc::WAY1);
        decode_way2->r1_reg_idx >> data_way_r1_reg_idx->get(WaySrc::WAY2);

        waycontrol->data_way_src >> data_way_r2_reg_idx->select;
        decode_way1->r2_reg_idx >> data_way_r2_reg_idx->get(WaySrc::WAY1);
        decode_way2->r2_reg_idx >> data_way_r2_reg_idx->get(WaySrc::WAY2);

        waycontrol->data_way_src >> data_way_wr_reg_idx->select;
        decode_way1->wr_reg_idx >> data_way_wr_reg_idx->get(WaySrc::WAY1);
        decode_way2->wr_reg_idx >> data_way_wr_reg_idx->get(WaySrc::WAY2);

        waycontrol->data_way_src >> data_way_pc->select;
        ifid_reg->pc_out >> data_way_pc->get(WaySrc::WAY1);
        ifid_reg->pc4_out >> data_way_pc->get(WaySrc::WAY2);

        waycontrol->exec_way_src >> exec_way_instr->select;
        ifid_reg->instr_out >> exec_way_instr->get(WaySrc::WAY1);
        ifid_reg->instr2_out >> exec_way_instr->get(WaySrc::WAY2);

        waycontrol->exec_way_src >> exec_way_opcode->select;
        decode_way1->opcode >> exec_way_opcode->get(WaySrc::WAY1);
        decode_way2->opcode >> exec_way_opcode->get(WaySrc::WAY2);

        waycontrol->exec_way_src >> exec_way_r1_reg_idx->select;
        decode_way1->r1_reg_idx >> exec_way_r1_reg_idx->get(WaySrc::WAY1);
        decode_way2->r1_reg_idx >> exec_way_r1_reg_idx->get(WaySrc::WAY2);

        waycontrol->exec_way_src >> exec_way_r2_reg_idx->select;
        decode_way1->r2_reg_idx >> exec_way_r2_reg_idx->get(WaySrc::WAY1);
        decode_way2->r2_reg_idx >> exec_way_r2_reg_idx->get(WaySrc::WAY2);

        waycontrol->exec_way_src >> exec_way_pc->select;
        ifid_reg->pc_out >> exec_way_pc->get(WaySrc::WAY1);
        ifid_reg->pc4_out >> exec_way_pc->get(WaySrc::WAY2);

        waycontrol->exec_way_src >> exec_way_wr_reg_idx->select;
        decode_way1->wr_reg_idx >> exec_way_wr_reg_idx->get(WaySrc::WAY1);
        decode_way2->wr_reg_idx >> exec_way_wr_reg_idx->get(WaySrc::WAY2);

        // -----------------------------------------------------------------------
        // Registers

        // Exec way
        exec_way_r1_reg_idx->out >> registerFile->r1_1_addr;
        exec_way_r2_reg_idx->out >> registerFile->r2_1_addr;
        reg_wr_src->out >> registerFile->data_1_in;
        memwb_reg->wr_reg_idx_out >> registerFile->wr_1_addr;
        memwb_reg->reg_do_write_out >> registerFile->wr_1_en;

        // Data way
        data_way_r1_reg_idx->out >> registerFile->r1_2_addr;
        data_way_r2_reg_idx->out >> registerFile->r2_2_addr;
        memwb_reg->mem_read_out >> registerFile->data_2_in;
        memwb_reg->wr_reg_idx_data_out >> registerFile->wr_2_addr;
        memwb_reg->reg_do_write_data_out >> registerFile->wr_2_en;

        registerFile->setMemory(m_regMem);

        // -----------------------------------------------------------------------
        // Branch
        idex_reg->br_op_out >> branch->comp_op;
        exec_reg1_fw_src->out >> branch->op1;
        exec_reg2_fw_src->out >> branch->op2;

        idex_reg->do_jmp_out >> branch->do_jump;
        idex_reg->do_br_out >> branch->do_branch;

        // -----------------------------------------------------------------------
        // Execution way ALU

        // Forwarding multiplexers
        idex_reg->r1_out >> exec_reg1_fw_src->get(ForwardingSrcDual::IdStage);
        exmem_reg->alures_out >> exec_reg1_fw_src->get(ForwardingSrcDual::MemStage);
        reg_wr_src->out >> exec_reg1_fw_src->get(ForwardingSrcDual::WbStageExec);
        memwb_reg->mem_read_out >> exec_reg1_fw_src->get(ForwardingSrcDual::WbStageMem);
        funit->alu_reg1_fw_ctrl_exec >> exec_reg1_fw_src->select;

        idex_reg->r2_out >> exec_reg2_fw_src->get(ForwardingSrcDual::IdStage);
        exmem_reg->alures_out >> exec_reg2_fw_src->get(ForwardingSrcDual::MemStage);
        reg_wr_src->out >> exec_reg2_fw_src->get(ForwardingSrcDual::WbStageExec);
        memwb_reg->mem_read_out >> exec_reg2_fw_src->get(ForwardingSrcDual::WbStageMem);
        funit->alu_reg2_fw_ctrl_exec >> exec_reg2_fw_src->select;

        // ALU operand multiplexers
        exec_reg1_fw_src->out >> alu_op1_exec_src->get(AluSrc1::REG1);
        idex_reg->pc_out >> alu_op1_exec_src->get(AluSrc1::PC);
        idex_reg->alu_op1_ctrl_out >> alu_op1_exec_src->select;

        exec_reg2_fw_src->out >> alu_op2_exec_src->get(AluSrc2::REG2);
        idex_reg->imm_out >> alu_op2_exec_src->get(AluSrc2::IMM);
        idex_reg->alu_op2_ctrl_out >> alu_op2_exec_src->select;

        alu_op1_exec_src->out >> alu->op1;
        alu_op2_exec_src->out >> alu->op2;

        idex_reg->alu_ctrl_out >> alu->ctrl;

        // -----------------------------------------------------------------------
        // Data way ALU

        // Forwarding multiplexers
        idex_reg->r1_data_out >> data_reg1_fw_src->get(ForwardingSrcDual::IdStage);
        exmem_reg->alures_out >> data_reg1_fw_src->get(ForwardingSrcDual::MemStage);
        reg_wr_src->out >> data_reg1_fw_src->get(ForwardingSrcDual::WbStageExec);
        memwb_reg->mem_read_out >> data_reg1_fw_src->get(ForwardingSrcDual::WbStageMem);
        funit->alu_reg1_fw_ctrl_data >> data_reg1_fw_src->select;

        idex_reg->r2_data_out >> data_reg2_fw_src->get(ForwardingSrcDual::IdStage);
        exmem_reg->alures_out >> data_reg2_fw_src->get(ForwardingSrcDual::MemStage);
        reg_wr_src->out >> data_reg2_fw_src->get(ForwardingSrcDual::WbStageExec);
        memwb_reg->mem_read_out >> data_reg2_fw_src->get(ForwardingSrcDual::WbStageMem);
        funit->alu_reg2_fw_ctrl_data >> data_reg2_fw_src->select;

        // ALU operand multiplexers
        data_reg1_fw_src->out >> alu_op1_data_src->get(AluSrc1::REG1);  // Todo: fix
        idex_reg->pc_out >> alu_op1_data_src->get(AluSrc1::PC);
        idex_reg->alu_op1_ctrl_data_out >> alu_op1_data_src->select;

        data_reg2_fw_src->out >> alu_op2_data_src->get(AluSrc2::REG2);  // Todo: fix
        idex_reg->imm_data_out >> alu_op2_data_src->get(AluSrc2::IMM);
        idex_reg->alu_op2_ctrl_data_out >> alu_op2_data_src->select;

        // ALU inputs
        alu_op1_data_src->out >> alu_data->op1;
        alu_op2_data_src->out >> alu_data->op2;
        idex_reg->alu_ctrl_data_out >> alu_data->ctrl;

        // -----------------------------------------------------------------------
        // Data memory
        exmem_reg->alures_data_out >> data_mem->addr;
        exmem_reg->mem_do_write_out >> data_mem->wr_en;
        exmem_reg->r2_out >> data_mem->data_in;
        exmem_reg->mem_op_out >> data_mem->op;
        data_mem->mem->setMemory(m_memory);

        // -----------------------------------------------------------------------
        // Ecall checker

        idex_reg->opcode_out >> ecallChecker->opcode;
        ecallChecker->setSysCallSignal(&handleSysCall);
        hzunit->stallEcallHandling >> ecallChecker->stallEcallHandling;

        // -----------------------------------------------------------------------
        // IF/ID
        pc_8->out >> ifid_reg->pc4_in;
        pc_reg->out >> ifid_reg->pc_in;
        instr_mem->data_out >> ifid_reg->instr_in;
        instr_mem->data_out2 >> ifid_reg->instr2_in;
        hzunit->hazardFEEnable >> ifid_reg->enable;
        efsc_or->out >> ifid_reg->clear;
        1 >> ifid_reg->valid_in;  // Always valid unless register is cleared

        // -----------------------------------------------------------------------
        // ID/EX
        hzunit->hazardIDEXEnable >> idex_reg->enable;
        hzunit->hazardIDEXClear >> idex_reg->stalled_in;
        efschz_or->out >> idex_reg->clear;

        // Data
        ifid_reg->pc4_out >> idex_reg->pc4_in;  // actually pc8!
        exec_way_pc->out >> idex_reg->pc_in;
        data_way_pc->out >> idex_reg->pc_data_in;
        registerFile->r1_1_out >> idex_reg->r1_in;
        registerFile->r2_1_out >> idex_reg->r2_in;
        registerFile->r1_2_out >> idex_reg->r1_data_in;
        registerFile->r2_2_out >> idex_reg->r2_data_in;

        imm_exec->imm >> idex_reg->imm_in;
        imm_data->imm >> idex_reg->imm_data_in;

        // Control
        exec_way_wr_reg_idx->out >> idex_reg->wr_reg_idx_in;
        0 >> idex_reg->reg_wr_src_ctrl_in;  // unused - we're using the specialized RegWrSrcDual
        control->reg_wr_src_ctrl >> idex_reg->reg_wr_src_ctrl_dual_in;
        control->reg_do_write_ctrl_exec >> idex_reg->reg_do_write_in;
        exec_way_r1_reg_idx->out >> idex_reg->rd_reg1_idx_in;
        exec_way_r2_reg_idx->out >> idex_reg->rd_reg2_idx_in;
        exec_way_opcode->out >> idex_reg->opcode_in;
        data_way_r1_reg_idx->out >> idex_reg->rd_reg1_idx_data_in;
        data_way_r2_reg_idx->out >> idex_reg->rd_reg2_idx_data_in;

        waycontrol->exec_way_valid >> control->exec_valid;
        waycontrol->data_way_valid >> control->data_valid;

        control->alu_op1_ctrl_exec >> idex_reg->alu_op1_ctrl_in;
        control->alu_op2_ctrl_exec >> idex_reg->alu_op2_ctrl_in;
        control->alu_ctrl_exec >> idex_reg->alu_ctrl_in;

        control->alu_op1_ctrl_data >> idex_reg->alu_op1_ctrl_data_in;
        control->alu_op2_ctrl_data >> idex_reg->alu_op2_ctrl_data_in;
        control->alu_ctrl_data >> idex_reg->alu_ctrl_data_in;

        control->mem_do_write_ctrl >> idex_reg->mem_do_write_in;
        control->mem_ctrl >> idex_reg->mem_op_in;
        control->comp_ctrl >> idex_reg->br_op_in;
        control->do_branch >> idex_reg->do_br_in;
        control->do_jump >> idex_reg->do_jmp_in;

        decode_way1->wr_reg_idx >> idex_reg->wr_reg_idx_data_in;
        control->reg_do_write_ctrl_data >> idex_reg->reg_do_write_data_in;
        control->mem_do_read_ctrl >> idex_reg->mem_do_read_in;

        ifid_reg->valid_out >> idex_reg->valid_in;

        // -----------------------------------------------------------------------
        // EX/MEM
        1 >> exmem_reg->enable;
        hzunit->hazardEXMEMClear >> exmem_reg->clear;
        hzunit->hazardEXMEMClear >> *mem_stalled_or->in[0];
        idex_reg->stalled_out >> *mem_stalled_or->in[1];
        mem_stalled_or->out >> exmem_reg->stalled_in;

        // Data
        idex_reg->pc_out >> exmem_reg->pc_in;        // @todo: Fix this - needs multiplexer aswell
        idex_reg->pc4_out >> exmem_reg->pc_data_in;  //@todo: Fix this - needs multiplexer aswell
        idex_reg->pc4_out >> exmem_reg->pc4_in;
        data_reg2_fw_src->out >> exmem_reg->r2_in;
        alu->res >> exmem_reg->alures_in;
        alu_data->res >> exmem_reg->alures_data_in;

        // Control
        0 >> exmem_reg->reg_wr_src_ctrl_in;  // unused - we're using the specialized RegWrSrcDual
        idex_reg->reg_wr_src_ctrl_dual_out >> exmem_reg->reg_wr_src_ctrl_dual_in;
        idex_reg->wr_reg_idx_out >> exmem_reg->wr_reg_idx_in;
        idex_reg->reg_do_write_out >> exmem_reg->reg_do_write_in;
        idex_reg->mem_do_write_out >> exmem_reg->mem_do_write_in;
        idex_reg->mem_do_read_out >> exmem_reg->mem_do_read_in;
        idex_reg->mem_op_out >> exmem_reg->mem_op_in;
        idex_reg->reg_do_write_data_out >> exmem_reg->reg_do_write_data_in;
        idex_reg->wr_reg_idx_data_out >> exmem_reg->wr_reg_idx_data_in;

        idex_reg->valid_out >> exmem_reg->valid_in;

        // -----------------------------------------------------------------------
        // MEM/WB

        exmem_reg->stalled_out >> memwb_reg->stalled_in;
        memwb_reg->alures_out >> reg_wr_src->get(RegWrSrcDual::ALURES);
        memwb_reg->pc4_out >> reg_wr_src->get(RegWrSrcDual::PC4);
        memwb_reg->reg_wr_src_ctrl_dual_out >> reg_wr_src->select;

        // Data
        exmem_reg->pc_out >> memwb_reg->pc_in;
        exmem_reg->pc_data_out >> memwb_reg->pc_data_in;
        exmem_reg->pc4_out >> memwb_reg->pc4_in;
        exmem_reg->alures_out >> memwb_reg->alures_in;
        data_mem->data_out >> memwb_reg->mem_read_in;

        // Control
        0 >> memwb_reg->reg_wr_src_ctrl_in;  // unused - we're using the specialized RegWrSrcDual
        exmem_reg->reg_wr_src_ctrl_dual_out >> memwb_reg->reg_wr_src_ctrl_dual_in;
        exmem_reg->wr_reg_idx_out >> memwb_reg->wr_reg_idx_in;
        exmem_reg->reg_do_write_out >> memwb_reg->reg_do_write_in;
        exmem_reg->reg_do_write_data_out >> memwb_reg->reg_do_write_data_in;
        exmem_reg->wr_reg_idx_data_out >> memwb_reg->wr_reg_idx_data_in;

        exmem_reg->valid_out >> memwb_reg->valid_in;

        // -----------------------------------------------------------------------
        // Forwarding unit
        idex_reg->rd_reg1_idx_out >> funit->id_reg1_idx_exec;
        idex_reg->rd_reg2_idx_out >> funit->id_reg2_idx_exec;
        idex_reg->rd_reg1_idx_data_out >> funit->id_reg1_idx_data;
        idex_reg->rd_reg2_idx_data_out >> funit->id_reg2_idx_data;

        exmem_reg->wr_reg_idx_out >> funit->mem_reg_wr_idx_exec;
        exmem_reg->reg_do_write_out >> funit->mem_reg_wr_en_exec;

        memwb_reg->wr_reg_idx_out >> funit->wb_reg_wr_idx_exec;
        memwb_reg->reg_do_write_out >> funit->wb_reg_wr_en_exec;
        memwb_reg->wr_reg_idx_data_out >> funit->wb_reg_wr_idx_data;
        memwb_reg->reg_do_write_data_out >> funit->wb_reg_wr_en_data;

        // -----------------------------------------------------------------------
        // Hazard detection unit
        exec_way_r1_reg_idx->out >> hzunit->id_reg1_idx_exec;
        exec_way_r2_reg_idx->out >> hzunit->id_reg2_idx_exec;
        data_way_r1_reg_idx->out >> hzunit->id_reg1_idx_data;
        data_way_r2_reg_idx->out >> hzunit->id_reg2_idx_data;

        idex_reg->mem_do_read_out >> hzunit->ex_do_mem_read_en;
        idex_reg->wr_reg_idx_data_out >> hzunit->ex_reg_wr_idx_data;

        exmem_reg->reg_do_write_out >> hzunit->mem_do_reg_write_exec;
        exmem_reg->reg_do_write_data_out >> hzunit->mem_do_reg_write_data;

        memwb_reg->reg_do_write_out >> hzunit->wb_do_reg_write_exec;
        memwb_reg->reg_do_write_data_out >> hzunit->wb_do_reg_write_data;

        idex_reg->opcode_out >> hzunit->opcode;
    }

    // Design subcomponents
    SUBCOMPONENT(registerFile, RegisterFile_DUAL<true>);
    SUBCOMPONENT(alu, ALU);
    SUBCOMPONENT(alu_data, ALU);
    SUBCOMPONENT(control, Control_DUAL);
    SUBCOMPONENT(waycontrol, WayControl);
    SUBCOMPONENT(imm_exec, Immediate);
    SUBCOMPONENT(imm_data, Immediate);
    SUBCOMPONENT(decode_way2, Decode);
    SUBCOMPONENT(decode_way1, Decode);
    SUBCOMPONENT(branch, Branch_DUAL);
    SUBCOMPONENT(pc_4, Adder<RV_REG_WIDTH>);
    SUBCOMPONENT(pc_8, Adder<RV_REG_WIDTH>);

    // Registers
    SUBCOMPONENT(pc_reg, RegisterClEn<RV_REG_WIDTH>);

    // Stage seperating registers
    SUBCOMPONENT(ifid_reg, IFID_DUAL);
    SUBCOMPONENT(idex_reg, RV5S_IDEX_DUAL);
    SUBCOMPONENT(exmem_reg, RV5S_EXMEM_DUAL);
    SUBCOMPONENT(memwb_reg, RV5S_MEMWB_DUAL);

    // Multiplexers
    SUBCOMPONENT(reg_wr_src, TYPE(EnumMultiplexer<RegWrSrcDual, RV_REG_WIDTH>));
    SUBCOMPONENT(pc_src, TYPE(EnumMultiplexer<PcSrc, RV_REG_WIDTH>));
    SUBCOMPONENT(pcadd_src, TYPE(EnumMultiplexer<PcSrcDual, RV_REG_WIDTH>));
    SUBCOMPONENT(alu_op1_exec_src, TYPE(EnumMultiplexer<AluSrc1, RV_REG_WIDTH>));
    SUBCOMPONENT(alu_op2_exec_src, TYPE(EnumMultiplexer<AluSrc2, RV_REG_WIDTH>));
    SUBCOMPONENT(alu_op1_data_src, TYPE(EnumMultiplexer<AluSrc1, RV_REG_WIDTH>));
    SUBCOMPONENT(alu_op2_data_src, TYPE(EnumMultiplexer<AluSrc2, RV_REG_WIDTH>));

    SUBCOMPONENT(exec_reg1_fw_src, TYPE(EnumMultiplexer<ForwardingSrcDual, RV_REG_WIDTH>));
    SUBCOMPONENT(exec_reg2_fw_src, TYPE(EnumMultiplexer<ForwardingSrcDual, RV_REG_WIDTH>));

    SUBCOMPONENT(data_reg1_fw_src, TYPE(EnumMultiplexer<ForwardingSrcDual, RV_REG_WIDTH>));
    SUBCOMPONENT(data_reg2_fw_src, TYPE(EnumMultiplexer<ForwardingSrcDual, RV_REG_WIDTH>));

    SUBCOMPONENT(data_way_pc, TYPE(EnumMultiplexer<WaySrc, RV_REG_WIDTH>));
    SUBCOMPONENT(data_way_opcode, TYPE(EnumMultiplexer<WaySrc, RVInstr::width()>));
    SUBCOMPONENT(data_way_instr, TYPE(EnumMultiplexer<WaySrc, RV_REG_WIDTH>));
    SUBCOMPONENT(data_way_wr_reg_idx, TYPE(EnumMultiplexer<WaySrc, RV_REGS_BITS>));
    SUBCOMPONENT(data_way_r1_reg_idx, TYPE(EnumMultiplexer<WaySrc, RV_REGS_BITS>));
    SUBCOMPONENT(data_way_r2_reg_idx, TYPE(EnumMultiplexer<WaySrc, RV_REGS_BITS>));

    SUBCOMPONENT(exec_way_pc, TYPE(EnumMultiplexer<WaySrc, RV_REG_WIDTH>));
    SUBCOMPONENT(exec_way_opcode, TYPE(EnumMultiplexer<WaySrc, RVInstr::width()>));
    SUBCOMPONENT(exec_way_instr, TYPE(EnumMultiplexer<WaySrc, RV_REG_WIDTH>));
    SUBCOMPONENT(exec_way_wr_reg_idx, TYPE(EnumMultiplexer<WaySrc, RV_REGS_BITS>));
    SUBCOMPONENT(exec_way_r1_reg_idx, TYPE(EnumMultiplexer<WaySrc, RV_REGS_BITS>));
    SUBCOMPONENT(exec_way_r2_reg_idx, TYPE(EnumMultiplexer<WaySrc, RV_REGS_BITS>));

    // Memories
    SUBCOMPONENT(instr_mem, TYPE(ROM_DUAL<RV_REG_WIDTH, RV_INSTR_WIDTH>));
    SUBCOMPONENT(data_mem, TYPE(RVMemory<RV_REG_WIDTH, RV_REG_WIDTH>));

    // Forwarding & hazard detection units
    SUBCOMPONENT(funit, ForwardingUnit_DUAL);
    SUBCOMPONENT(hzunit, HazardUnit_DUAL);

    // Gates
    // True if controlflow action or performing syscall finishing
    SUBCOMPONENT(efsc_or, TYPE(Or<1, 2>));
    // True if above or stalling due to load-use hazard
    SUBCOMPONENT(efschz_or, TYPE(Or<1, 2>));

    SUBCOMPONENT(mem_stalled_or, TYPE(Or<1, 2>));

    // Address spaces
    ADDRESSSPACE(m_memory);
    ADDRESSSPACE(m_regMem);

    SUBCOMPONENT(ecallChecker, EcallChecker);

    // Ripes interface compliance
    unsigned int stageCount() const override { return STAGECOUNT; }
    unsigned int getPcForStage(unsigned int idx) const override {
        // clang-format off
        switch (idx) {
            case IF_1: return pc_reg->out.uValue();
            case IF_2: return pc_reg->out.uValue() + 4;
            case ID_1: return ifid_reg->pc_out.uValue();
            case ID_2: return ifid_reg->pc4_out.uValue();
            case EX_EXEC: return idex_reg->pc_out.uValue();
            case EX_DATA: return idex_reg->pc4_out.uValue();
            case MEM_EXEC: return exmem_reg->pc_out.uValue();
            case MEM_DATA: return exmem_reg->pc_data_out.uValue();
            case WB_EXEC: return memwb_reg->pc_out.uValue();
            case WB_DATA: return memwb_reg->pc_data_out.uValue();
            default: assert(false && "Processor does not contain stage");
        }
        Q_UNREACHABLE();
        // clang-format on
    }
    unsigned int nextFetchedAddress() const override { return pc_src->out.uValue(); }
    QString stageName(unsigned int idx) const override {
        // clang-format off
        switch (idx) {
            case IF_1: case IF_2: return "IF";
            case ID_1: case ID_2: return "ID";
            case EX_EXEC: case EX_DATA: return "EX";
            case MEM_EXEC: case MEM_DATA: return "MEM";
            case WB_EXEC: case WB_DATA: return "WB";
            default: assert(false && "Processor does not contain stage");
        }
        Q_UNREACHABLE();
        // clang-format on
    }
    StageInfo stageInfo(unsigned int stage) const override {
        bool stageValid = true;
        // Has the pipeline stage been filled?
        stageValid &= (stage / 2) <= m_cycleCount;

        // clang-format off
        // Has the stage been cleared?
        switch(stage){
        case ID_1: case ID_2: stageValid &= ifid_reg->valid_out.uValue(); break;
        case EX_EXEC: case EX_DATA: stageValid &= idex_reg->valid_out.uValue(); break;
        case MEM_EXEC: case MEM_DATA: stageValid &= exmem_reg->valid_out.uValue(); break;
        case WB_EXEC: case WB_DATA: stageValid &= memwb_reg->valid_out.uValue(); break;
        default: case IF_1: case IF_2: break;
        }

        // Is the stage carrying a valid (executable) PC?
        /// @todo: also check for valid way (not cleared)
        switch(stage){
        case ID_1: stageValid &= isExecutableAddress(ifid_reg->pc_out.uValue()); break;
        case ID_2: stageValid &= isExecutableAddress(ifid_reg->pc4_out.uValue()); break;
        case EX_EXEC: stageValid &= isExecutableAddress(idex_reg->pc_out.uValue()); break;
        case EX_DATA: stageValid &= isExecutableAddress(idex_reg->pc4_out.uValue()); break;
        case MEM_EXEC: stageValid &= isExecutableAddress(exmem_reg->pc_out.uValue()); break;
        case MEM_DATA: stageValid &= isExecutableAddress(exmem_reg->pc_data_out.uValue()); break;
        case WB_EXEC: stageValid &= isExecutableAddress(memwb_reg->pc_out.uValue()); break;
        case WB_DATA: stageValid &= isExecutableAddress(memwb_reg->pc_data_out.uValue()); break;
        default: case IF_1: case IF_2: stageValid &= isExecutableAddress(pc_reg->out.uValue()); break;
        }

        // Are we currently clearing the pipeline due to a syscall exit? if such, all stages before the EX stage are invalid
        if(stage < EX_EXEC){
            stageValid &= !ecallChecker->isSysCallExiting();
        }
        // clang-format on

        // Gather stage state info
        StageInfo::State state = StageInfo ::State::None;
        switch (stage) {
            case IF_1:
            case IF_2:
                break;
            case ID_1:
            case ID_2:
                if (m_cycleCount > (ID_1 / 2) && ifid_reg->valid_out.uValue() == 0) {
                    state = StageInfo::State::Flushed;
                }
                break;
            case EX_EXEC:
            case EX_DATA: {
                if (idex_reg->stalled_out.uValue() == 1) {
                    state = StageInfo::State::Stalled;
                } else if (m_cycleCount > (EX_EXEC / 2) && idex_reg->valid_out.uValue() == 0) {
                    state = StageInfo::State::Flushed;
                }
                break;
            }
            case MEM_DATA:
            case MEM_EXEC: {
                if (exmem_reg->stalled_out.uValue() == 1) {
                    state = StageInfo::State::Stalled;
                } else if (m_cycleCount > (MEM_EXEC / 2) && exmem_reg->valid_out.uValue() == 0) {
                    state = StageInfo::State::Flushed;
                }
                break;
            }
            case WB_DATA:
            case WB_EXEC: {
                if (memwb_reg->stalled_out.uValue() == 1) {
                    state = StageInfo::State::Stalled;
                } else if (m_cycleCount > (WB_EXEC / 2) && memwb_reg->valid_out.uValue() == 0) {
                    state = StageInfo::State::Flushed;
                }
                break;
            }
        }

        return StageInfo({getPcForStage(stage), stageValid, state});
    }

    void setProgramCounter(uint32_t address) override {
        pc_reg->forceValue(0, address);
        propagateDesign();
    }
    void setPCInitialValue(uint32_t address) override { pc_reg->setInitValue(address); }
    SparseArray& getMemory() override { return *m_memory; }
    unsigned int getRegister(RegisterFileType rfid, unsigned i) const override { return registerFile->getRegister(i); }
    SparseArray& getArchRegisters() override { return *m_regMem; }
    void finalize(const FinalizeReason& fr) override {
        if (fr.exitSyscall && !ecallChecker->isSysCallExiting()) {
            // An exit system call was executed. Record the cycle of the execution, and enable the ecallChecker's system
            // call exiting signal.
            m_syscallExitCycle = m_cycleCount;
        }
        ecallChecker->setSysCallExiting(ecallChecker->isSysCallExiting() || fr.exitSyscall);
    }

    const Component* getDataMemory() const override { return data_mem; }
    const Component* getInstrMemory() const override { return instr_mem; }

    bool finished() const override {
        // The processor is finished when there are no more valid instructions in the pipeline
        bool allStagesInvalid = true;
        for (int stage = IF_1; stage < STAGECOUNT; stage++) {
            allStagesInvalid &= !stageInfo(stage).stage_valid;
            if (!allStagesInvalid)
                break;
        }
        return allStagesInvalid;
    }

    void setRegister(RegisterFileType rfid, unsigned i, uint32_t v) override {
        setSynchronousValue(registerFile->rf_1->_wr_mem, i, v);
    }

    void clock() override {
        // An instruction has been retired if the instruction in the WB stage is valid and the PC is within the
        // executable range of the program
        if (memwb_reg->valid_out.uValue() != 0 && isExecutableAddress(memwb_reg->pc_out.uValue())) {
            m_instructionsRetired++;
        }

        RipesProcessor::clock();
    }

    void reverse() override {
        if (m_syscallExitCycle != -1 && (m_cycleCount - 1) == m_syscallExitCycle) {
            // We are about to undo an exit syscall instruction. In this case, the syscall exiting sequence should
            // be terminate
            ecallChecker->setSysCallExiting(false);
            m_syscallExitCycle = -1;
        }
        RipesProcessor::reverse();
        if (memwb_reg->valid_out.uValue() != 0 && isExecutableAddress(memwb_reg->pc_out.uValue())) {
            m_instructionsRetired--;
        }
    }

    void reset() override {
        ecallChecker->setSysCallExiting(false);
        RipesProcessor::reset();
        m_syscallExitCycle = -1;
    }

    static const ISAInfoBase* ISA() {
        static auto s_isa = ISAInfo<ISA::RV32I>(QStringList{"M"});
        return &s_isa;
    }

    const ISAInfoBase* supportsISA() const override { return ISA(); };
    const ISAInfoBase* implementsISA() const override { return m_enabledISA.get(); };

    const std::set<RegisterFileType> registerFiles() const override {
        std::set<RegisterFileType> rfs;
        rfs.insert(RegisterFileType::GPR);

        if (implementsISA()->extensionEnabled("F")) {
            rfs.insert(RegisterFileType::FPR);
        }
        return rfs;
    }

private:
    /**
     * @brief m_syscallExitCycle
     * The variable will contain the cycle of which an exit system call was executed. From this, we may determine
     * when we roll back an exit system call during rewinding.
     */
    long long m_syscallExitCycle = -1;
    std::shared_ptr<ISAInfo<ISA::RV32I>> m_enabledISA;
};

}  // namespace core
}  // namespace vsrtl
