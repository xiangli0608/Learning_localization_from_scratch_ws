#include "lidar_localization/sensor_data/gnss_data.hpp"


GeographicLib::LocalCartesian lidar_localization::GNSSData::geo_converter;

namespace lidar_localization {

void GNSSData::InitOriginPosition() {
    geo_converter.Reset(latitude, longitude, altitude);
}

void GNSSData::UpdateXYZ() {
    geo_converter.Forward(latitude, longitude, altitude, local_E, local_N, local_U);
}
}