#pragma once
#include <fstream>
#include <string>
#include <iostream>
#include <mutex>

class MetricsLogger {
public:
    static void Initialize(const std::string& filename, int seed) {
        std::lock_guard<std::mutex> lock(logMutex);
        file.open(filename, std::ios::app);
        if (file.tellp() == 0) {
            file << "Seed,TimeSec,Event,ShooterID,TargetClass,Weapon,RangeNM,Outcome\n";
        }
        currentSeed = seed;
    }

    static void Log(float time, const std::string& event, uint32_t shooter,
                    int targetClass, const std::string& weapon, float range, const std::string& outcome) {

        std::lock_guard<std::mutex> lock(logMutex);

        if (file.is_open()) {
            file << currentSeed << "," << time << "," << event << "," << shooter << ","
                 << targetClass << "," << weapon << "," << range << "," << outcome << "\n";
        }
    }

    static void Shutdown() {
        std::lock_guard<std::mutex> lock(logMutex);
        if(file.is_open()) file.close();
     }

private:
    inline static std::ofstream file;
    inline static int currentSeed = 0;
    inline static std::mutex logMutex;
};