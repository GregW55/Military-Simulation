#pragma once
#include <string>

class TacticalDataLoader {
public:
    // Parses units.json and populates the TacticalDatabase
    static void Load(const std::string& filepath);
};