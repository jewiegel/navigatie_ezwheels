// Copyright 2026 Improvia
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

// Offline check of a behavior tree XML against the node types of BT plugin libraries.
// Not a ROS node: it registers the plugins (without instantiating any node, so no
// Nav2 servers are needed), runs BT::VerifyXML and prints the node model XML
// (ports per node type) on stdout so callers can also check port names.
//
// Usage: check_bt_xml <tree.xml> <plugin_lib> [<plugin_lib> ...]

#include <fstream>
#include <iostream>
#include <sstream>
#include <string>

#include "behaviortree_cpp_v3/bt_factory.h"
#include "behaviortree_cpp_v3/utils/shared_library.h"
#include "behaviortree_cpp_v3/xml_parsing.h"

int main(int argc, char ** argv)
{
  if (argc < 3) {
    std::cerr << "Usage: check_bt_xml <tree.xml> <plugin_lib> [<plugin_lib> ...]" << std::endl;
    return 2;
  }

  std::ifstream file(argv[1]);
  if (!file) {
    std::cerr << "Cannot read " << argv[1] << std::endl;
    return 2;
  }
  std::stringstream xml;
  xml << file.rdbuf();

  BT::BehaviorTreeFactory factory;
  try {
    for (int i = 2; i < argc; ++i) {
      factory.registerFromPlugin(BT::SharedLibrary::getOSName(argv[i]));
    }
    std::unordered_map<std::string, BT::NodeType> registered;
    for (const auto & [id, manifest] : factory.manifests()) {
      registered[id] = manifest.type;
    }
    BT::VerifyXML(xml.str(), registered);
  } catch (const std::exception & e) {
    std::cerr << "Invalid behavior tree " << argv[1] << ": " << e.what() << std::endl;
    return 1;
  }

  std::cout << BT::writeTreeNodesModelXML(factory) << std::endl;
  return 0;
}
