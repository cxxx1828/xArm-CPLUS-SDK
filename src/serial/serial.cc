#include "serial/serial.h"

#include <algorithm>
#include <thread>
#include <mutex>
#include <functional>
#include <chrono>
#include <iostream>
#include <sstream>
#include <iomanip>

#if !defined(_WIN32) && !defined(__OpenBSD__) && !defined(__FreeBSD__)
# include <alloca.h>
#endif

#if defined (__MINGW32__)
# define alloca __builtin_alloca
#endif

#ifdef _WIN32
#include "serial/impl/win.h"
#else
#include "serial/impl/unix.h"
#endif

using namespace serial;
using namespace std;

// ==========================
//  Scoped Locks
// ==========================

class Serial::ScopedReadLock {
public:
    ScopedReadLock(SerialImpl *pimpl) : pimpl_(pimpl) {
        pimpl_->readLock();
    }
    ~ScopedReadLock() {
        pimpl_->readUnlock();
    }
private:
    SerialImpl *pimpl_;
};

class Serial::ScopedWriteLock {
public:
    ScopedWriteLock(SerialImpl *pimpl) : pimpl_(pimpl) {
        pimpl_->writeLock();
    }
    ~ScopedWriteLock() {
        pimpl_->writeUnlock();
    }
private:
    SerialImpl *pimpl_;
};

// ==========================
//  Async Reader
// ==========================

class Serial::AsyncReader {
public:
    AsyncReader(SerialImpl* impl)
        : pimpl_(impl), running_(false) {}

    void start(function<void(const string&)> cb) {
        running_ = true;
        thread_ = thread([this, cb]() {
            while (running_) {
                if (pimpl_->available()) {
                    uint8_t buffer[256];
                    size_t n = pimpl_->read(buffer, sizeof(buffer));
                    if (n > 0) {
                        cb(string((char*)buffer, n));
                    }
                }
                this_thread::sleep_for(chrono::milliseconds(5));
            }
        });
    }

    void stop() {
        running_ = false;
        if (thread_.joinable())
            thread_.join();
    }

private:
    SerialImpl* pimpl_;
    thread thread_;
    bool running_;
};

// ==========================
//  Constructor / Destructor
// ==========================

Serial::Serial(const string &port,
               uint32_t baudrate,
               Timeout timeout,
               bytesize_t bytesize,
               parity_t parity,
               stopbits_t stopbits,
               flowcontrol_t flowcontrol)
{
    pimpl_ = new SerialImpl(port, baudrate, bytesize, parity, stopbits, flowcontrol);
    pimpl_->setTimeout(timeout);
    asyncReader_ = nullptr;
}

Serial::~Serial() {
    stopAsync();
    delete pimpl_;
}

// ==========================
//  Basic control
// ==========================

void Serial::open() { pimpl_->open(); }
void Serial::close() { pimpl_->close(); }
bool Serial::isOpen() const { return pimpl_->isOpen(); }
size_t Serial::available() { return pimpl_->available(); }

// ==========================
//  BASIC READ/WRITE
// ==========================

size_t Serial::read(uint8_t *buffer, size_t size) {
    ScopedReadLock lock(pimpl_);
    return pimpl_->read(buffer, size);
}

size_t Serial::write(const uint8_t *data, size_t size) {
    ScopedWriteLock lock(pimpl_);
    return pimpl_->write(data, size);
}

size_t Serial::write(const string &data) {
    return write((uint8_t*)data.c_str(), data.size());
}

// ==========================
//  NEW FEATURES
// ==========================

// 🔹 Read exact bytes
string Serial::readExact(size_t size) {
    string result;
    result.reserve(size);

    while (result.size() < size) {
        uint8_t buf[256];
        size_t to_read = min(size - result.size(), sizeof(buf));
        size_t n = read(buf, to_read);
        if (n == 0) break;
        result.append((char*)buf, n);
    }
    return result;
}

//  Read until delimiter
string Serial::readUntil(const string& delim) {
    string buffer;
    char c;

    while (true) {
        size_t n = read((uint8_t*)&c, 1);
        if (n == 0) break;

        buffer += c;
        if (buffer.size() >= delim.size() &&
            buffer.substr(buffer.size() - delim.size()) == delim)
            break;
    }
    return buffer;
}

//  Read with timeout
string Serial::readWithTimeout(size_t size, int timeout_ms) {
    string result;
    auto start = chrono::steady_clock::now();

    while (result.size() < size) {
        if (available()) {
            uint8_t buf[256];
            size_t n = read(buf, sizeof(buf));
            result.append((char*)buf, n);
        }

        auto now = chrono::steady_clock::now();
        int elapsed = chrono::duration_cast<chrono::milliseconds>(now - start).count();

        if (elapsed > timeout_ms)
            break;
    }
    return result;
}

// Write line
size_t Serial::writeLine(const string& line) {
    return write(line + "\n");
}

// Request-response
string Serial::requestResponse(const string& cmd, int timeout_ms) {
    flushInput();
    writeLine(cmd);
    return readWithTimeout(1024, timeout_ms);
}

// HEX utils
string Serial::toHex(const string& input) {
    stringstream ss;
    for (unsigned char c : input)
        ss << hex << setw(2) << setfill('0') << (int)c << " ";
    return ss.str();
}

void Serial::hexDump(const string& data) {
    cout << "[HEX] " << toHex(data) << endl;
}

// Auto reconnect
bool Serial::reconnect(int retries, int delay_ms) {
    for (int i = 0; i < retries; i++) {
        try {
            if (!isOpen()) {
                open();
                return true;
            }
        } catch (...) {}
        this_thread::sleep_for(chrono::milliseconds(delay_ms));
    }
    return false;
}

// ==========================
// ASYNC API
// ==========================

void Serial::startAsync(function<void(const string&)> cb) {
    if (!asyncReader_)
        asyncReader_ = new AsyncReader(pimpl_);
    asyncReader_->start(cb);
}

void Serial::stopAsync() {
    if (asyncReader_) {
        asyncReader_->stop();
        delete asyncReader_;
        asyncReader_ = nullptr;
    }
}

// ==========================
// Flush
// ==========================

void Serial::flush() {
    ScopedReadLock r(pimpl_);
    ScopedWriteLock w(pimpl_);
    pimpl_->flush();
}

void Serial::flushInput() {
    ScopedReadLock r(pimpl_);
    pimpl_->flushInput();
}

void Serial::flushOutput() {
    ScopedWriteLock w(pimpl_);
    pimpl_->flushOutput();
}
