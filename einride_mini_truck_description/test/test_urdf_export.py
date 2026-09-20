# Copyright 2025 Einride AB
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""Checks the model.urdf the build generates against the model.sdf it came from.

The URDF is what `robot_description` carries, and what every visualiser builds
the robot out of - so a conversion that quietly drops a link produces a robot
that is missing a wheel in RViz and Foxglove while the TF tree, which
robot_state_publisher derives from the same file, is missing it too.

The names are the contract. Frame names appear in message headers, in the .rviz
config, and in the hardware drivers' parameters; they are the one thing that
must survive the conversion unchanged, so they are what is compared. Poses are
not: sdformat_urdf is the same code robot_state_publisher used to run on the SDF
directly, and re-deriving its arithmetic here would only test the test.

The two malformations the converter exists to clean up are checked positively,
because both are silent. An out-of-range alpha makes urdfdom reject the material
and fall back to no colour; an empty <texture/> is legal to urdfdom and not to
every other reader.
"""

import os
import pathlib
import xml.etree.ElementTree as ET

import pytest


def _path(var):
    value = os.environ.get(var)
    assert value, f'{var} is not set; the CMake test fixture passes it in'
    return pathlib.Path(value)


@pytest.fixture(scope='module')
def sdf():
    return ET.parse(_path('MODEL_SDF')).getroot()


@pytest.fixture(scope='module')
def urdf():
    return ET.parse(_path('MODEL_URDF')).getroot()


def test_root_is_a_urdf_robot(urdf, sdf):
    # The whole reason this file is generated. Foxglove's 3D panel reads <robot>
    # and nothing else; handed the <sdf> document it renders nothing at all and
    # says nothing about why.
    assert urdf.tag == 'robot'
    assert urdf.get('name') == sdf.find('model').get('name')


def test_link_names_survive_the_conversion(urdf, sdf):
    assert ({link.get('name') for link in urdf.findall('link')} ==
            {link.get('name') for link in sdf.find('model').findall('link')})


def test_joint_names_survive_the_conversion(urdf, sdf):
    assert ({joint.get('name') for joint in urdf.findall('joint')} ==
            {joint.get('name') for joint in sdf.find('model').findall('joint')})


def test_meshes_stay_package_relative_and_exist(urdf):
    # package:// is what lets RViz, Gazebo and the Foxglove bridge's asset
    # fetcher each resolve the same string. An absolute path would work on the
    # machine that built the file and nowhere else.
    models = _path('MODEL_SDF').parent.parent
    meshes = urdf.findall('.//geometry/mesh')
    assert meshes, 'no meshes in the exported URDF'
    for mesh in meshes:
        filename = mesh.get('filename')
        prefix = 'package://einride_mini_truck_description/models/'
        assert filename.startswith(prefix), filename
        assert (models / filename[len(prefix):]).is_file(), filename


def test_material_colors_are_in_range(urdf):
    for color in urdf.findall('.//material/color'):
        rgba = [float(v) for v in color.get('rgba').split()]
        assert len(rgba) == 4
        assert all(0.0 <= v <= 1.0 for v in rgba), color.get('rgba')


def test_no_empty_textures(urdf):
    for texture in urdf.findall('.//texture'):
        assert texture.get('filename'), 'material carries a texture with no file'
