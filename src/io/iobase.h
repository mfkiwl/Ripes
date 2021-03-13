#pragma once

#include <QVariant>
#include <QWidget>
#include <typeindex>
#include <typeinfo>

namespace Ripes {

struct IOParam {
    IOParam(unsigned _id, QString _name, QVariant _defValue, bool _hasRange, QVariant _min = QVariant(),
            QVariant _max = QVariant())
        : id(_id), name(_name), value(_defValue), defaultValue(_defValue), hasRange(_hasRange), min(_min), max(_max) {}

    IOParam() {}

    unsigned id = -1;
    QString name;
    QVariant value;
    QVariant defaultValue;
    bool hasRange = false;
    QVariant min;
    QVariant max;
};

struct RegDesc {
    enum class RW { R, W, RW };
    QString name;
    RW rw;
    unsigned bitWidth;
    uint32_t address;
};

class IOBase : public QWidget {
    Q_OBJECT

public:
    IOBase(QWidget* parent) : QWidget(parent) { registerPeripheral(this); }
    virtual ~IOBase() { deregisterPeripheral(this); };

    const std::map<unsigned, IOParam>& parameters() const { return m_parameters; }
    virtual QString description() const = 0;
    virtual QString name() const = 0;

    /**
     * @brief registers
     * @return a description of the programmable interface of this peripheral
     */
    virtual const std::vector<RegDesc>& registers() const = 0;

    /**
     * @brief setParameter
     * Attempt to set the parameter @p ID to @p value. Returns true if the value was set successfully.
     */
    virtual bool setParameter(unsigned ID, const QVariant& value);
    virtual uint32_t size() const = 0;

    /**
     * Hardware read/write functions
     */
    virtual uint32_t ioRead8(uint32_t offset) = 0;
    virtual void ioWrite8(uint32_t offset, uint32_t value) = 0;
    virtual uint32_t ioRead16(uint32_t offset) = 0;
    virtual void ioWrite16(uint32_t offset, uint32_t value) = 0;
    virtual uint32_t ioRead32(uint32_t offset) = 0;
    virtual void ioWrite32(uint32_t offset, uint32_t value) = 0;

signals:
    /**
     * @brief regMapChanged
     * Should be emitted every time the register map of this peripheral changes.
     */
    void regMapChanged();

protected:
    virtual void parameterChanged(unsigned ID) = 0;

    std::map<unsigned, IOParam> m_parameters;

    static std::map<std::type_index, int> s_peripheralCount;
    static void registerPeripheral(const IOBase* p) {
        s_peripheralCount[typeid(p)]++;
        return;
    }
    static void deregisterPeripheral(const IOBase* p) {
        Q_ASSERT(s_peripheralCount.at(typeid(p)) > 0);
        s_peripheralCount[typeid(p)]--;
    }
};
}  // namespace Ripes
