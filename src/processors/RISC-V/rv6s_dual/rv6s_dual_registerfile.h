#pragma once

#include "VSRTL/core/vsrtl_constant.h"
#include "VSRTL/core/vsrtl_memory.h"
#include "VSRTL/core/vsrtl_wire.h"

#include "../riscv.h"
#include "../rv_registerfile.h"

namespace vsrtl {
namespace core {
using namespace Ripes;

template <bool readBypass>
class RegisterFile_DUAL : public Component {
public:
    SetGraphicsType(ClockedComponent);
    RegisterFile_DUAL(std::string name, SimComponent* parent) : Component(name, parent) {
        // Way 1
        r1_1_addr >> rf_1->r1_addr;
        r2_1_addr >> rf_1->r2_addr;
        wr_1_addr >> rf_1->wr_addr;
        rf_1->r1_out >> r1_1_out;
        rf_1->r2_out >> r2_1_out;
        data_1_in >> rf_1->data_in;
        wr_1_en >> rf_1->wr_en;

        // Way 2
        r1_2_addr >> rf_2->r1_addr;
        r2_2_addr >> rf_2->r2_addr;
        wr_2_addr >> rf_2->wr_addr;
        rf_2->r1_out >> r1_2_out;
        rf_2->r2_out >> r2_2_out;
        data_2_in >> rf_2->data_in;
        wr_2_en >> rf_2->wr_en;
    }

    void setMemory(SparseArray* mem) {
        m_memory = mem;
        // All memory components must point to the same memory
        rf_1->setMemory(mem);
        rf_2->setMemory(mem);
    }

    SUBCOMPONENT(rf_1, TYPE(RegisterFile<readBypass>));
    SUBCOMPONENT(rf_2, TYPE(RegisterFile<readBypass>));

    // Way 1
    INPUTPORT(r1_1_addr, RV_REGS_BITS);
    INPUTPORT(r2_1_addr, RV_REGS_BITS);
    INPUTPORT(wr_1_addr, RV_REGS_BITS);
    OUTPUTPORT(r1_1_out, RV_REG_WIDTH);
    OUTPUTPORT(r2_1_out, RV_REG_WIDTH);
    INPUTPORT(data_1_in, RV_REG_WIDTH);
    INPUTPORT(wr_1_en, 1);

    // Way 2
    INPUTPORT(r1_2_addr, RV_REGS_BITS);
    INPUTPORT(r2_2_addr, RV_REGS_BITS);
    INPUTPORT(wr_2_addr, RV_REGS_BITS);
    OUTPUTPORT(r1_2_out, RV_REG_WIDTH);
    OUTPUTPORT(r2_2_out, RV_REG_WIDTH);
    INPUTPORT(data_2_in, RV_REG_WIDTH);
    INPUTPORT(wr_2_en, 1);

    VSRTL_VT_U getRegister(unsigned i) { return m_memory->readMem<false>(i); }

private:
    SparseArray* m_memory = nullptr;
};

}  // namespace core
}  // namespace vsrtl
