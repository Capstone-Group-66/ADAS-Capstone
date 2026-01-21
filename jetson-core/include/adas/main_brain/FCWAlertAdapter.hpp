// File: include/adas/main_brain/FCWAlertAdapter.hpp
// Converts FCWAlert from Stage E to Alert for BLE transmission
#pragma once

#include "adas/main_brain/Alert.hpp"
#include "adas/stage_e/FCWMonitor.hpp"
#include "adas/common/Clock.hpp"

#include <sstream>
#include <iomanip>

namespace adas {

/**
 * Adapts FCWAlert (from Stage E sensor fusion) to Alert (for BLE/mobile)
 */
class FCWAlertAdapter {
public:
    /// Convert FCWAlert to BLE Alert
    static Alert convert(const FCWAlert& fcw, uint64_t timestamp_ns) {
        Alert alert;
        
        // Timestamp in milliseconds
        alert.t_ms = timestamp_ns / 1'000'000;
        
        // Unique ID (tick based, no track_id in FCWAlert)
        std::ostringstream id;
        id << "fcw-" << (timestamp_ns / 50'000'000) << "-" << fcw.object_class;
        alert.id = id.str();
        
        // Type is always FCW
        alert.type = AlertType::FCW;
        
        // Direction is front
        alert.direction = "front";
        
        // Severity based on TTC
        if (fcw.ttc_s < 1.0f) {
            alert.severity = Severity::Critical;
        } else if (fcw.ttc_s < 2.0f) {
            alert.severity = Severity::Warning;
        } else {
            alert.severity = Severity::Info;
        }
        
        // TTL: 1 second
        alert.ttl_ms = 1000;
        
        // Rationale as JSON
        std::ostringstream rationale;
        rationale << std::fixed << std::setprecision(2);
        rationale << "{\"ttc_s\":" << fcw.ttc_s 
                  << ",\"range_m\":" << fcw.range_m 
                  << ",\"class\":\"" << fcw.object_class << "\"}";
        alert.rationale = rationale.str();
        
        // Object ID (use object_class since track_id not available)
        alert.object_id = fcw.object_class;
        
        // Sources
        alert.sources = {"FrontCam", "FrontRadar"};
        
        // Schema version
        alert.schemaVersion = "v1.0";
        
        // Confidence (default since not in FCWAlert)
        alert.confidence = 0.9f;
        
        return alert;
    }
    
    /// Serialize Alert to JSON bytes for BLE transmission
    static std::vector<uint8_t> toJson(const Alert& alert) {
        std::ostringstream json;
        json << std::fixed << std::setprecision(2);
        json << "{";
        json << "\"t_ms\":" << alert.t_ms << ",";
        json << "\"id\":\"" << alert.id << "\",";
        json << "\"type\":\"" << alertTypeToString(alert.type) << "\",";
        if (alert.direction.has_value()) {
            json << "\"direction\":\"" << *alert.direction << "\",";
        }
        json << "\"severity\":\"" << severityToString(alert.severity) << "\",";
        json << "\"ttl_ms\":" << alert.ttl_ms << ",";
        json << "\"rationale\":" << alert.rationale << ",";
        if (alert.object_id.has_value()) {
            json << "\"object_id\":" << *alert.object_id << ",";
        }
        json << "\"sources\":[";
        for (size_t i = 0; i < alert.sources.size(); ++i) {
            if (i > 0) json << ",";
            json << "\"" << alert.sources[i] << "\"";
        }
        json << "],";
        json << "\"schemaVersion\":\"" << alert.schemaVersion << "\",";
        json << "\"confidence\":" << alert.confidence;
        json << "}";
        
        std::string s = json.str();
        return std::vector<uint8_t>(s.begin(), s.end());
    }
};

}  // namespace adas
