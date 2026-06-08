#include "core/AegisEngine.h"

int main() {
    AegisEngine engine;

    // Boot the simulation
    if (engine.Initialize()) {
        // Hand over control to the main loop
        engine.Run();
    }

    // Clean up memory and exit
    engine.Shutdown();
    return 0;
}