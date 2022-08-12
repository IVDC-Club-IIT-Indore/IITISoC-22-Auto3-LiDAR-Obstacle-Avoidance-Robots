#pragma once

#include "pugixml.hpp"

namespace hebi {
namespace xml {

bool trySetFloatParameter(pugi::xml_attribute&& attr, float& param);
bool trySetBoolParameter(pugi::xml_attribute&& attr, bool& param);
bool trySetStringParameter(pugi::xml_attribute&& attr, std::string& param);

} // namespace xml
} // namespace hebi
