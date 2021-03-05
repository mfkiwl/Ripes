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
#include "rv6s_dual_control.h"
#include "rv6s_dual_instr_mem.h"
#include "rv6s_dual_registerfile.h"
#include "rv6s_dual_waycontrol.h"

// Stage separating registers
#include "rv6s_dual_branch.h"
#include "rv6s_dual_exmem.h"
#include "rv6s_dual_idii.h"
#include "rv6s_dual_ifid.h"
#include "rv6s_dual_iiex.h"
#include "rv6s_dual_memwb.h"

// Forwarding & Hazard detection unit
#include "rv6s_dual_forwardingunit.h"
#include "rv6s_dual_hazardunit.h"

namespace vsrtl {
namespace core {
using namespace Ripes;

class RV6S_DUAL : public RipesProcessor {
public:
    enum Stage { IF_1, IF_2, ID_1, ID_2, EX_EXEC, EX_DATA, MEM_EXEC, MEM_DATA, WB_EXEC, WB_DATA, STAGECOUNT };
    RV6S_DUAL(const QStringList& extensions) : RipesProcessor("6-Stage Dual-issue RISC-V Processor") {
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

        ifid_reg->valid_out >> *wayhazard->in[0];
        waycontrol->stall_out >> *wayhazard->in[1];
        wayhazard->out >> *wayhazard_inv->in[0];

        hzunit->hazardFEEnable >> *pc_enabled->in[0];
        wayhazard_inv->out >> *pc_enabled->in[1];
        pc_enabled->out >> pc_reg->enable;

        alu->res >> pc_src->get(PcSrc::ALU);
        pc_8->out >> pc_src->get(PcSrc::PC4);

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
        // Immediate
        idii_reg->opcode_exec_out >> imm_exec->opcode;
        idii_reg->instr_exec_out >> imm_exec->instr;

        idii_reg->opcode_data_out >> imm_data->opcode;
        idii_reg->instr_data_out >> imm_data->instr;

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
        idii_reg->way_stall_out >> waycontrol->stall_in;
        ifid_reg->valid_out >> waycontrol->ifid_valid;

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
        idii_reg->rd_reg1_idx_exec_out >> registerFile->r1_1_addr;
        idii_reg->rd_reg1_idx_exec_out >> registerFile->r2_1_addr;
        reg_wr_src->out >> registerFile->data_1_in;
        memwb_reg->wr_reg_idx_out >> registerFile->wr_1_addr;
        memwb_reg->reg_do_write_out >> registerFile->wr_1_en;

        // Data way
        idii_reg->rd_reg1_idx_data_out >> registerFile->r1_2_addr;
        idii_reg->rd_reg2_idx_data_out >> registerFile->r2_2_addr;
        memwb_reg->mem_read_out >> registerFile->data_2_in;
        memwb_reg->wr_reg_idx_data_out >> registerFile->wr_2_addr;
        memwb_reg->reg_do_write_data_out >> registerFile->wr_2_en;

        registerFile->setMemory(m_regMem);

        // -----------------------------------------------------------------------
        // Branch
        iiex_reg->br_op_out >> branch->comp_op;
        exec_reg1_fw_src->out >> branch->op1;
        exec_reg2_fw_src->out >> branch->op2;

        iiex_reg->do_jmp_out >> branch->do_jump;
        iiex_reg->do_br_out >> branch->do_branch;

        // -----------------------------------------------------------------------
        // Execution way ALU

        // Forwarding multiplexers
        iiex_reg->r1_out >> exec_reg1_fw_src->get(ForwardingSrcDual::IdStage);
        exmem_reg->alures_out >> exec_reg1_fw_src->get(ForwardingSrcDual::MemStage);
        reg_wr_src->out >> exec_reg1_fw_src->get(ForwardingSrcDual::WbStageExec);
        memwb_reg->mem_read_out >> exec_reg1_fw_src->get(ForwardingSrcDual::WbStageMem);
        funit->alu_reg1_fw_ctrl_exec >> exec_reg1_fw_src->select;

        iiex_reg->r2_out >> exec_reg2_fw_src->get(ForwardingSrcDual::IdStage);
        exmem_reg->alures_out >> exec_reg2_fw_src->get(ForwardingSrcDual::MemStage);
        reg_wr_src->out >> exec_reg2_fw_src->get(ForwardingSrcDual::WbStageExec);
        memwb_reg->mem_read_out >> exec_reg2_fw_src->get(ForwardingSrcDual::WbStageMem);
        funit->alu_reg2_fw_ctrl_exec >> exec_reg2_fw_src->select;

        // ALU operand multiplexers
        exec_reg1_fw_src->out >> alu_op1_exec_src->get(AluSrc1::REG1);
        iiex_reg->pc_out >> alu_op1_exec_src->get(AluSrc1::PC);
        iiex_reg->alu_op1_ctrl_out >> alu_op1_exec_src->select;

        exec_reg2_fw_src->out >> alu_op2_exec_src->get(AluSrc2::REG2);
        iiex_reg->imm_out >> alu_op2_exec_src->get(AluSrc2::IMM);
        iiex_reg->alu_op2_ctrl_out >> alu_op2_exec_src->select;

        alu_op1_exec_src->out >> alu->op1;
        alu_op2_exec_src->out >> alu->op2;

        iiex_reg->alu_ctrl_out >> alu->ctrl;

        // -----------------------------------------------------------------------
        // Data way ALU

        // Forwarding multiplexers
        iiex_reg->r1_data_out >> data_reg1_fw_src->get(ForwardingSrcDual::IdStage);
        exmem_reg->alures_out >> data_reg1_fw_src->get(ForwardingSrcDual::MemStage);
        reg_wr_src->out >> data_reg1_fw_src->get(ForwardingSrcDual::WbStageExec);
        memwb_reg->mem_read_out >> data_reg1_fw_src->get(ForwardingSrcDual::WbStageMem);
        funit->alu_reg1_fw_ctrl_data >> data_reg1_fw_src->select;

        iiex_reg->r2_data_out >> data_reg2_fw_src->get(ForwardingSrcDual::IdStage);
        exmem_reg->alures_out >> data_reg2_fw_src->get(ForwardingSrcDual::MemStage);
        reg_wr_src->out >> data_reg2_fw_src->get(ForwardingSrcDual::WbStageExec);
        memwb_reg->mem_read_out >> data_reg2_fw_src->get(ForwardingSrcDual::WbStageMem);
        funit->alu_reg2_fw_ctrl_data >> data_reg2_fw_src->select;

        // ALU operand multiplexers
        data_reg1_fw_src->out >> alu_op1_data_src->get(AluSrc1::REG1);  // Todo: fix
        iiex_reg->pc_out >> alu_op1_data_src->get(AluSrc1::PC);
        iiex_reg->alu_op1_ctrl_data_out >> alu_op1_data_src->select;

        data_reg2_fw_src->out >> alu_op2_data_src->get(AluSrc2::REG2);  // Todo: fix
        iiex_reg->imm_data_out >> alu_op2_data_src->get(AluSrc2::IMM);
        iiex_reg->alu_op2_ctrl_data_out >> alu_op2_data_src->select;

        // ALU inputs
        alu_op1_data_src->out >> alu_data->op1;
        alu_op2_data_src->out >> alu_data->op2;
        iiex_reg->alu_ctrl_data_out >> alu_data->ctrl;

        // -----------------------------------------------------------------------
        // Data memory
        exmem_reg->alures_data_out >> data_mem->addr;
        exmem_reg->mem_do_write_out >> data_mem->wr_en;
        exmem_reg->r2_out >> data_mem->data_in;
        exmem_reg->mem_op_out >> data_mem->op;
        data_mem->mem->setMemory(m_memory);

        // -----------------------------------------------------------------------
        // Ecall checker

        iiex_reg->opcode_out >> ecallChecker->opcode;
        ecallChecker->setSysCallSignal(&handleSysCall);
        hzunit->stallEcallHandling >> ecallChecker->stallEcallHandling;

        // -----------------------------------------------------------------------
        // IF/ID
        pc_4->out >> ifid_reg->pc4_in;
        pc_reg->out >> ifid_reg->pc_in;
        instr_mem->data_out >> ifid_reg->instr_in;
        instr_mem->data_out2 >> ifid_reg->instr2_in;
        hzunit->hazardFEEnable >> ifid_reg->enable;
        efsc_or->out >> ifid_reg->clear;
        1 >> ifid_reg->valid_in;  // Always valid unless register is cleared

        // -----------------------------------------------------------------------
        // ID/II
        ifid_reg->pc_out >> idii_reg->pc_in;
        ifid_reg->pc4_out >> idii_reg->pc4_in;  // actually pc8!
        ifid_reg->valid_out >> idii_reg->valid_in;
        waycontrol->stall_out >> idii_reg->way_stall_in;
        efsc_or->out >> idii_reg->clear;

        exec_way_pc->out >> idii_reg->pc_exec_in;
        exec_way_r1_reg_idx->out >> idii_reg->rd_reg1_idx_exec_in;
        exec_way_r2_reg_idx->out >> idii_reg->rd_reg2_idx_exec_in;
        exec_way_wr_reg_idx->out >> idii_reg->wr_reg_idx_exec_in;
        exec_way_opcode->out >> idii_reg->opcode_exec_in;
        exec_way_instr->out >> idii_reg->instr_exec_in;

        data_way_pc->out >> idii_reg->pc_data_in;
        data_way_r1_reg_idx->out >> idii_reg->rd_reg1_idx_data_in;
        data_way_r2_reg_idx->out >> idii_reg->rd_reg2_idx_data_in;
        data_way_wr_reg_idx->out >> idii_reg->wr_reg_idx_data_in;
        data_way_opcode->out >> idii_reg->opcode_data_in;
        waycontrol->data_way_valid >> idii_reg->data_valid_in;
        waycontrol->exec_way_valid >> idii_reg->exec_valid_in;
        data_way_instr->out >> idii_reg->instr_data_in;
        hzunit->hazardFEEnable >> idii_reg->enable;

        // -----------------------------------------------------------------------
        // II/EX
        hzunit->hazardIDEXEnable >> iiex_reg->enable;
        hzunit->hazardIDEXClear >> iiex_reg->stalled_in;
        efschz_or->out >> iiex_reg->clear;

        // Data
        idii_reg->pc4_out >> iiex_reg->pc4_in;  // actually pc8!
        idii_reg->pc_exec_out >> iiex_reg->pc_in;
        idii_reg->pc_data_out >> iiex_reg->pc_data_in;
        registerFile->r1_1_out >> iiex_reg->r1_in;
        registerFile->r2_1_out >> iiex_reg->r2_in;
        registerFile->r1_2_out >> iiex_reg->r1_data_in;
        registerFile->r2_2_out >> iiex_reg->r2_data_in;

        imm_exec->imm >> iiex_reg->imm_in;
        imm_data->imm >> iiex_reg->imm_data_in;

        // Control
        idii_reg->wr_reg_idx_exec_out >> iiex_reg->wr_reg_idx_in;
        0 >> iiex_reg->reg_wr_src_ctrl_in;  // unused - we're using the specialized RegWrSrcDual
        control->reg_wr_src_ctrl >> iiex_reg->reg_wr_src_ctrl_dual_in;
        control->reg_do_write_ctrl_exec >> iiex_reg->reg_do_write_in;
        idii_reg->rd_reg1_idx_exec_out >> iiex_reg->rd_reg1_idx_in;
        idii_reg->rd_reg2_idx_exec_out >> iiex_reg->rd_reg2_idx_in;
        idii_reg->rd_reg1_idx_data_out >> iiex_reg->rd_reg1_idx_data_in;
        idii_reg->rd_reg2_idx_data_out >> iiex_reg->rd_reg2_idx_data_in;
        idii_reg->opcode_exec_out >> iiex_reg->opcode_in;

        idii_reg->exec_valid_out >> control->exec_valid;
        idii_reg->data_valid_out >> control->data_valid;
        idii_reg->opcode_data_out >> control->opcode_data;
        idii_reg->opcode_exec_out >> control->opcode_exec;

        control->alu_op1_ctrl_exec >> iiex_reg->alu_op1_ctrl_in;
        control->alu_op2_ctrl_exec >> iiex_reg->alu_op2_ctrl_in;
        control->alu_ctrl_exec >> iiex_reg->alu_ctrl_in;

        control->alu_op1_ctrl_data >> iiex_reg->alu_op1_ctrl_data_in;
        control->alu_op2_ctrl_data >> iiex_reg->alu_op2_ctrl_data_in;
        control->alu_ctrl_data >> iiex_reg->alu_ctrl_data_in;

        control->mem_do_write_ctrl >> iiex_reg->mem_do_write_in;
        control->mem_ctrl >> iiex_reg->mem_op_in;
        control->comp_ctrl >> iiex_reg->br_op_in;
        control->do_branch >> iiex_reg->do_br_in;
        control->do_jump >> iiex_reg->do_jmp_in;

        idii_reg->wr_reg_idx_data_out >> iiex_reg->wr_reg_idx_data_in;
        control->reg_do_write_ctrl_data >> iiex_reg->reg_do_write_data_in;
        control->mem_do_read_ctrl >> iiex_reg->mem_do_read_in;

        idii_reg->valid_out >> iiex_reg->valid_in;
        idii_reg->data_valid_out >> iiex_reg->data_valid_in;
        idii_reg->exec_valid_out >> iiex_reg->exec_valid_in;

        // -----------------------------------------------------------------------
        // EX/MEM
        1 >> exmem_reg->enable;
        hzunit->hazardEXMEMClear >> exmem_reg->clear;
        hzunit->hazardEXMEMClear >> *mem_stalled_or->in[0];
        iiex_reg->stalled_out >> *mem_stalled_or->in[1];
        mem_stalled_or->out >> exmem_reg->stalled_in;

        // Data
        iiex_reg->pc_out >> exmem_reg->pc_in;        // @todo: Fix this - needs multiplexer aswell
        iiex_reg->pc4_out >> exmem_reg->pc_data_in;  //@todo: Fix this - needs multiplexer aswell
        iiex_reg->pc4_out >> exmem_reg->pc4_in;
        data_reg2_fw_src->out >> exmem_reg->r2_in;
        alu->res >> exmem_reg->alures_in;
        alu_data->res >> exmem_reg->alures_data_in;

        // Control
        0 >> exmem_reg->reg_wr_src_ctrl_in;  // unused - we're using the specialized RegWrSrcDual
        iiex_reg->reg_wr_src_ctrl_dual_out >> exmem_reg->reg_wr_src_ctrl_dual_in;
        iiex_reg->wr_reg_idx_out >> exmem_reg->wr_reg_idx_in;
        iiex_reg->reg_do_write_out >> exmem_reg->reg_do_write_in;
        iiex_reg->mem_do_write_out >> exmem_reg->mem_do_write_in;
        iiex_reg->mem_do_read_out >> exmem_reg->mem_do_read_in;
        iiex_reg->mem_op_out >> exmem_reg->mem_op_in;
        iiex_reg->reg_do_write_data_out >> exmem_reg->reg_do_write_data_in;
        iiex_reg->wr_reg_idx_data_out >> exmem_reg->wr_reg_idx_data_in;

        iiex_reg->valid_out >> exmem_reg->valid_in;
        iiex_reg->data_valid_out >> exmem_reg->data_valid_in;
        iiex_reg->exec_valid_out >> exmem_reg->exec_valid_in;

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
        exmem_reg->data_valid_out >> memwb_reg->data_valid_in;
        exmem_reg->exec_valid_out >> memwb_reg->exec_valid_in;

        // -----------------------------------------------------------------------
        // Forwarding unit
        iiex_reg->rd_reg1_idx_out >> funit->id_reg1_idx_exec;
        iiex_reg->rd_reg2_idx_out >> funit->id_reg2_idx_exec;
        iiex_reg->rd_reg1_idx_data_out >> funit->id_reg1_idx_data;
        iiex_reg->rd_reg2_idx_data_out >> funit->id_reg2_idx_data;

        exmem_reg->wr_reg_idx_out >> funit->mem_reg_wr_idx_exec;
        exmem_reg->reg_do_write_out >> funit->mem_reg_wr_en_exec;

        memwb_reg->wr_reg_idx_out >> funit->wb_reg_wr_idx_exec;
        memwb_reg->reg_do_write_out >> funit->wb_reg_wr_en_exec;
        memwb_reg->wr_reg_idx_data_out >> funit->wb_reg_wr_idx_data;
        memwb_reg->reg_do_write_data_out >> funit->wb_reg_wr_en_data;

        // -----------------------------------------------------------------------
        // Hazard detection unit
        idii_reg->rd_reg1_idx_exec_out >> hzunit->id_reg1_idx_exec;
        idii_reg->rd_reg2_idx_exec_out >> hzunit->id_reg2_idx_exec;
        idii_reg->rd_reg1_idx_data_out >> hzunit->id_reg1_idx_data;
        idii_reg->rd_reg2_idx_data_out >> hzunit->id_reg2_idx_data;

        iiex_reg->mem_do_read_out >> hzunit->ex_do_mem_read_en;
        iiex_reg->wr_reg_idx_data_out >> hzunit->ex_reg_wr_idx_data;

        exmem_reg->reg_do_write_out >> hzunit->mem_do_reg_write_exec;
        exmem_reg->reg_do_write_data_out >> hzunit->mem_do_reg_write_data;

        memwb_reg->reg_do_write_out >> hzunit->wb_do_reg_write_exec;
        memwb_reg->reg_do_write_data_out >> hzunit->wb_do_reg_write_data;

        iiex_reg->opcode_out >> hzunit->opcode;
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
    SUBCOMPONENT(idii_reg, RV5S_IDII_DUAL);
    SUBCOMPONENT(iiex_reg, RV5S_IIEX_DUAL);
    SUBCOMPONENT(exmem_reg, RV5S_EXMEM_DUAL);
    SUBCOMPONENT(memwb_reg, RV5S_MEMWB_DUAL);

    // Multiplexers
    SUBCOMPONENT(reg_wr_src, TYPE(EnumMultiplexer<RegWrSrcDual, RV_REG_WIDTH>));
    SUBCOMPONENT(pc_src, TYPE(EnumMultiplexer<PcSrc, RV_REG_WIDTH>));
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

    // True if no hazard in the ID stage or no hazard unit induced stall
    SUBCOMPONENT(pc_enabled, TYPE(And<1, 2>));
    SUBCOMPONENT(wayhazard, TYPE(And<1, 2>));
    SUBCOMPONENT(wayhazard_inv, TYPE(Not<1, 1>));

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
            case EX_EXEC: return iiex_reg->pc_out.uValue();
            case EX_DATA: return iiex_reg->pc4_out.uValue();
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
        case EX_EXEC: case EX_DATA: stageValid &= iiex_reg->valid_out.uValue(); break;
        case MEM_EXEC: case MEM_DATA: stageValid &= exmem_reg->valid_out.uValue(); break;
        case WB_EXEC: case WB_DATA: stageValid &= memwb_reg->valid_out.uValue(); break;
        default: case IF_1: case IF_2: break;
        }

        // Is the stage carrying a valid (executable) PC?
        /// @todo: also check for valid way (not cleared)
        switch(stage){
        case ID_1: stageValid &= isExecutableAddress(ifid_reg->pc_out.uValue()); break;
        case ID_2: stageValid &= isExecutableAddress(ifid_reg->pc4_out.uValue()); break;
        case EX_EXEC: stageValid &= isExecutableAddress(iiex_reg->pc_out.uValue()); break;
        case EX_DATA: stageValid &= isExecutableAddress(iiex_reg->pc4_out.uValue()); break;
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
                if (iiex_reg->stalled_out.uValue() == 1) {
                    state = StageInfo::State::Stalled;
                } else if (m_cycleCount > (EX_EXEC / 2) && iiex_reg->valid_out.uValue() == 0) {
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

    int instructionsRetired() const {
        int amount = 0;
        // An instruction has been retired if the instruction in the WB stage is valid and the PC is within the
        // executable range of the program
        if (memwb_reg->valid_out.uValue() != 0) {
            if (isExecutableAddress(memwb_reg->pc_out.uValue()) && memwb_reg->exec_valid_out.uValue()) {
                amount++;
            }
            if (isExecutableAddress(memwb_reg->pc4_out.uValue()) && memwb_reg->data_valid_out.uValue()) {
                amount++;
            }
        }
        return amount;
    }

    void clock() override {
        // An instruction has been retired if the instruction in the WB stage is valid and the PC is within the
        // executable range of the program
        m_instructionsRetired += instructionsRetired();

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
        m_instructionsRetired -= instructionsRetired();
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
