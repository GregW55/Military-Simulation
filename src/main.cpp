#include "core/AegisEngine.h"
#include <string>
#include <iostream>

int main(int argc, char* argv[]) {
    bool headless = false;
    int seed = 1820;

    for (int i = 1; i < argc; ++i) {
        std::string arg = argv[i];
        if (arg == "--headless") headless = true;
        if (arg == "--seed" && i + 1 < argc){
            std::string seedStr =  argv[++i];
            try {
                seed = std::stoi(seedStr);
            } catch (const std::exception& e) {
                std::cerr << "WARNING: Invalid seed argument '" << seedStr
                          << "'. Defaulting to " << seed << "\n";
            }
        }
    }

    if (headless) std::cout << "Starting Headless Batch Run. Seed: " << seed << "\n";

    AegisEngine engine;
    if (engine.Initialize(headless, seed)) {
        engine.Run();
    }

    engine.Shutdown();
    return 0;
}