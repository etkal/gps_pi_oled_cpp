from conan import ConanFile
from conan.tools.cmake import CMakeToolchain, CMakeDeps, cmake_layout
from conan.tools.env import VirtualBuildEnv
from conan.tools.files import copy
import os

required_conan_version = ">=2.21.0"

class GPS_OLED(ConanFile):
    settings = "os", "arch", "compiler", "build_type"
    # default_options = {"boost/*:shared": False}

    def layout(self):
        cmake_layout(self)
    
    # def requirements(self):
        # self.requires("boost/1.88.0")

    def generate(self):
        tc = CMakeToolchain(self)
        tc.generate()

        tc = CMakeDeps(self)
        tc.generate()

        tc = VirtualBuildEnv(self)
        tc.generate()

