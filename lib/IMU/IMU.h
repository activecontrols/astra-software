#pragma once


class IMU{
    public:
    IMU(int cs);
    void begin();

    private:
    int cs;

    // see https://en.cppreference.com/w/cpp/language/pimpl.html
    struct Impl;
    std::unique_ptr<Impl> pimpl_;
};

