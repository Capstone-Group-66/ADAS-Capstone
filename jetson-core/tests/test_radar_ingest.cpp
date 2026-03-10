// File: tests/test_radar_ingest.cpp
#include "adas/stage_a/RadarIngest.hpp"
#include "adas/queues/SPSCQueue.hpp"

#include <iostream>
#include <string>
#include <vector>
#include <cassert>

using namespace adas;

int main() {
    SPSCQueue<RadarTargets, 8> queue;
    
    RadarConfig config;
    config.speed_ttl_ms = 900; // 900ms TTL
    config.speed_mag_threshold = 10;
    config.range_mag_threshold = 20;
    
    RadarIngest ingest(Mount::FrontRadar, "/dev/null", queue, config);

    std::cout << "Test 1: Normal speed update followed by rapid ranges (should be fresh)\n";
    std::string speed_msg = "{\"unit\":\"mps\", \"speed\":-15.0}\r\n";
    uint64_t t_base = 1000000000ULL; // 1 second
    
    // Inject speed
    ingest.parseFrame(reinterpret_cast<const uint8_t*>(speed_msg.data()), speed_msg.length(), t_base);
    
    // Inject range immediately
    std::string range_msg1 = "{\"unit\":\"m\", \"range\":25.0}\r\n";
    RadarTargets t1 = ingest.parseFrame(reinterpret_cast<const uint8_t*>(range_msg1.data()), range_msg1.length(), t_base + 1000000ULL); // +1ms
    
    assert(!t1.targets.empty());
    assert(t1.targets[0].radial_vel_mps == -15.0f);
    assert(t1.targets[0].speed_fresh == true);
    std::cout << "  Passed Phase 1 (speed applied within TTL)\n";

    std::cout << "Test 2: Range update at TTL boundary (899ms) (should be fresh)\n";
    RadarTargets t2 = ingest.parseFrame(reinterpret_cast<const uint8_t*>(range_msg1.data()), range_msg1.length(), t_base + 899000000ULL); // +899ms
    assert(!t2.targets.empty());
    assert(t2.targets[0].radial_vel_mps == -15.0f);
    assert(t2.targets[0].speed_fresh == true);
    std::cout << "  Passed Phase 2 (speed applied exactly within TTL)\n";

    std::cout << "Test 3: Range update AFTER TTL boundary (901ms) (should be STALE)\n";
    RadarTargets t3 = ingest.parseFrame(reinterpret_cast<const uint8_t*>(range_msg1.data()), range_msg1.length(), t_base + 901000000ULL); // +901ms
    assert(!t3.targets.empty());
    assert(t3.targets[0].radial_vel_mps == 0.0f);
    assert(t3.targets[0].speed_fresh == false);
    std::cout << "  Passed Phase 3 (speed timed out and became stale)\n";
    
    std::cout << "Test 4: Fragmented JSON lines over serial stream\n";
    std::string frag1 = "{\"uni";
    std::string frag2 = "t\":\"m\", \"range\":12.0}\r\n";
    
    RadarTargets f1 = ingest.parseFrame(reinterpret_cast<const uint8_t*>(frag1.data()), frag1.length(), t_base + 500000000ULL);
    assert(f1.targets.empty()); // Should not trigger target yet
    
    RadarTargets f2 = ingest.parseFrame(reinterpret_cast<const uint8_t*>(frag2.data()), frag2.length(), t_base + 1000000000ULL); // +1000ms
    assert(!f2.targets.empty());
    assert(f2.targets[0].range_m == 12.0f);
    assert(f2.targets[0].speed_fresh == false); // Still stale (1000ms > 900ms)
    std::cout << "  Passed Phase 4 (fragmented buffering logic works)\n";

    std::cout << "\nAll test_radar_ingest assertions passed!\n";
    return 0;
}
