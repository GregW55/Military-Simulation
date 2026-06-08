#pragma once
#include <string>
#include "../models/ScenarioData.h"

class ScenarioParser {
public:
    static ScenarioData Load(const std::string& filepath);
};