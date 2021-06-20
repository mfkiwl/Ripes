#pragma once

#include "isainfo.h"

namespace Ripes {

namespace RVISA {

extern const QStringList RegAliases;
extern const QStringList RegNames;
extern const QStringList RegDescs;
enum Opcode {
    LUI = 0b0110111,
    JAL = 0b1101111,
    JALR = 0b1100111,
    BRANCH = 0b1100011,
    LOAD = 0b0000011,
    STORE = 0b0100011,
    OPIMM = 0b0010011,
    OP = 0b0110011,
    OPIMM32 = 0b0011011,
    OP32 = 0b0111011,
    ECALL = 0b1110011,
    AUIPC = 0b0010111,
    INVALID = 0b0
};

}  // namespace RVISA

namespace RVABI {
// RISC-V ELF info
// Elf flag masks
enum RVElfFlags { RVC = 0b1, FloatABI = 0b110, RVE = 0b1000, TSO = 0b10000 };
extern const std::map<RVElfFlags, QString> ELFFlagStrings;

enum SysCall {
    None = 0,
    PrintInt = 1,
    PrintFloat = 2,
    PrintStr = 4,
    Exit = 10,
    PrintChar = 11,
    GetCWD = 17,
    TimeMs = 30,
    Cycles = 31,
    PrintIntHex = 34,
    PrintIntBinary = 35,
    PrintIntUnsigned = 36,
    Close = 57,
    LSeek = 62,
    Read = 63,
    Write = 64,
    FStat = 80,
    Exit2 = 93,
    brk = 214,
    Open = 1024
};

}  // namespace RVABI

class RVISAInfoBase : public ISAInfoBase {
public:
    unsigned int regCnt() const override { return 32; }
    QString regName(unsigned i) const override {
        return RVISA::RegNames.size() > static_cast<int>(i) ? RVISA::RegNames.at(static_cast<int>(i)) : QString();
    }
    QString regAlias(unsigned i) const override {
        return RVISA::RegAliases.size() > static_cast<int>(i) ? RVISA::RegAliases.at(static_cast<int>(i)) : QString();
    }
    QString regInfo(unsigned i) const override {
        return RVISA::RegDescs.size() > static_cast<int>(i) ? RVISA::RegDescs.at(static_cast<int>(i)) : QString();
    }
    bool regIsReadOnly(unsigned i) const override { return i == 0; }
    int spReg() const override { return 2; }
    int gpReg() const override { return 3; }
    int syscallReg() const override { return 17; }
    unsigned instrBits() const override { return 32; }
    unsigned elfMachineId() const override { return EM_RISCV; }
    unsigned int regNumber(const QString& reg, bool& success) const override {
        QString regRes = reg;
        success = true;
        if (reg[0] == 'x' && (RVISA::RegNames.count(reg) != 0)) {
            regRes.remove('x');
            return regRes.toInt(&success, 10);
        } else if (RVISA::RegAliases.contains(reg)) {
            return RVISA::RegAliases.indexOf(reg);
        }
        success = false;
        return 0;
    }

    QString elfSupportsFlags(unsigned flags) const override {
        /** We expect no flags for RV32IM compiled RISC-V executables.
         *  Refer to: https://github.com/riscv/riscv-elf-psabi-doc/blob/master/riscv-elf.md#-elf-object-files
         */
        if (flags == 0)
            return QString();
        QString err;
        for (const auto& flag : RVABI::ELFFlagStrings) {
            if (flags & flag.first) {
                err += "ELF flag '" + RVABI::ELFFlagStrings.at(flag.first) + "' unsupported<br/>";
            }
        }
        return err;
    }

    const QStringList& supportedExtensions() const override { return m_supportedExtensions; }
    const QStringList& enabledExtensions() const override { return m_enabledExtensions; }

protected:
    QStringList m_enabledExtensions;
    QStringList m_supportedExtensions = {"M"};
};

}  // namespace Ripes