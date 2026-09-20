// Copyright 2025 Einride AB
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
//
// Build-time converter: model.sdf -> model.urdf.
//
// WHY THIS EXISTS
//
// robot_state_publisher happily takes the SDF directly - sdformat_urdf is
// registered as a urdf_parser plugin, so `robot_description` may hold either
// dialect and the TF tree comes out the same. That is what this project did,
// and it works for RViz, which resolves the description through the same
// pluggable parser.
//
// Foxglove does not. Its 3D panel has its own URDF reader and only understands
// <robot>; handed an <sdf> document it produces no model and no error anyone
// sees. So `robot_description` has to carry URDF for the robot to be visible
// there, and this program is what makes that possible without keeping a second
// description by hand.
//
// model.sdf stays the single source of truth. The conversion is exactly the one
// robot_state_publisher used to perform internally - same sdformat_urdf call -
// so the link, joint and frame names it produces are the ones the rest of the
// stack already uses, by construction rather than by agreement.
//
// Sensors are dropped, loudly: URDF has nowhere to put them. Nothing reads
// sensor definitions off `robot_description` - Gazebo loads model.sdf itself,
// and the hardware drivers are configured from their own parameter files - so
// what is lost here is lost only to the visualiser, which never wanted it.

#include <algorithm>
#include <fstream>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>

#include <sdformat_urdf/sdformat_urdf.hpp>
#include <tinyxml2.h>
#include <urdf_parser/urdf_parser.h>

namespace
{

// sdformat_urdf builds a visual's colour by blending SDF's ambient and diffuse
// terms, and blends the alpha channel along with the other three. Two opaque
// terms therefore give alpha 1.2, which urdfdom writes out verbatim and its own
// parser then rejects - "Unable to parse component [1.2] to a double" - taking
// the whole material with it. Clamping here is what makes the exported file
// valid URDF rather than something only the exporter accepts.
void clamp_material_colors(const urdf::ModelInterfaceSharedPtr & model)
{
  const auto clamp = [](float v) {return std::min(1.0f, std::max(0.0f, v));};
  const auto fix = [&clamp](const urdf::MaterialSharedPtr & m) {
      if (!m) {
        return;
      }
      m->color.r = clamp(m->color.r);
      m->color.g = clamp(m->color.g);
      m->color.b = clamp(m->color.b);
      m->color.a = clamp(m->color.a);
    };

  for (const auto & entry : model->materials_) {
    fix(entry.second);
  }
  for (const auto & entry : model->links_) {
    if (entry.second->visual) {
      fix(entry.second->visual->material);
    }
    for (const auto & visual : entry.second->visual_array) {
      fix(visual->material);
    }
  }
}

// urdfdom's exporter emits a bare <texture/> for every material, whether or not
// one was set. It is meaningless, and a reader stricter than urdfdom's own -
// Foxglove's, for one - is entitled to treat a texture with no filename as an
// error. Cheaper to not write it.
void drop_empty_textures(tinyxml2::XMLElement * element)
{
  std::vector<tinyxml2::XMLElement *> doomed;
  for (auto * child = element->FirstChildElement(); child;
    child = child->NextSiblingElement())
  {
    if (std::string(child->Name()) == "texture" && !child->Attribute("filename")) {
      doomed.push_back(child);
    } else {
      drop_empty_textures(child);
    }
  }
  for (auto * child : doomed) {
    element->DeleteChild(child);
  }
}

}  // namespace

int main(int argc, char ** argv)
{
  if (argc != 3) {
    std::cerr << "usage: sdf_to_urdf <input.sdf> <output.urdf>\n";
    return 2;
  }

  std::ifstream input(argv[1]);
  if (!input) {
    std::cerr << "sdf_to_urdf: cannot read " << argv[1] << "\n";
    return 1;
  }
  std::stringstream buffer;
  buffer << input.rdbuf();

  sdf::Errors errors;
  const auto model = sdformat_urdf::parse(buffer.str(), errors);
  for (const auto & error : errors) {
    std::cerr << "sdf_to_urdf: " << error << "\n";
  }
  if (!model) {
    std::cerr << "sdf_to_urdf: " << argv[1] << " did not parse as SDFormat\n";
    return 1;
  }

  clamp_material_colors(model);

  // Deprecated upstream - "file an issue if you rely on this" - but it is the
  // only serialiser urdfdom has, and the alternative is writing URDF XML by
  // hand from the same structures it already walks. Pinned here with a warning
  // suppression rather than left to shout on every build; if urdfdom ever
  // removes it, this is the line that will say so.
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
  tinyxml2::XMLDocument * document = urdf::exportURDF(*model);
#pragma GCC diagnostic pop
  if (!document) {
    std::cerr << "sdf_to_urdf: URDF export failed\n";
    return 1;
  }
  drop_empty_textures(document->RootElement());

  tinyxml2::XMLPrinter printer;
  document->Print(&printer);

  // Read back through urdfdom's own parser before anything is written. A file
  // that only the exporter can read is worse than no file: robot_state_publisher
  // would fail at launch, long after the build that produced it said nothing.
  const std::string urdf(printer.CStr());
  if (!urdf::parseURDF(urdf)) {
    std::cerr << "sdf_to_urdf: the exported URDF does not parse; refusing to write it\n";
    return 1;
  }

  std::ofstream output(argv[2]);
  if (!output) {
    std::cerr << "sdf_to_urdf: cannot write " << argv[2] << "\n";
    return 1;
  }
  output << urdf;
  return 0;
}
