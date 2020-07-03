from conans import ConanFile, CMake, tools
import os
import shutil

class SerialConan(ConanFile):
    name = "serial"
    version = "1.2.1"
    description = "Cross-platform, Serial Port library written in C++"
    license = ("MIT")
    url = "https://gitlab.com/openrov/thirdparty/conan-serial"
    repo_url = "https://github.com/wjwwood/serial"
    author = "Aqualink Developer <developer@aqualink.org>"

    settings = "os", "compiler", "arch", "build_type"
    options = {"shared": [True, False], "fPIC": [True, False]}
    default_options = "shared=False", "fPIC=True"

    sha1 = "fba8d81b5dbfaf6359af3b435bd16d9ec3629825"

    exports_sources = "CMakeLists.txt"

    _source_subfolder   = "%s-%s" % ( name, sha1 )

    def source(self):
        tools.get("%s/archive/%s.zip" % (self.repo_url, self.sha1))

        # Copy custom CMakeLists.txt which is Linux specific and doesn't depend on catkin
        shutil.copy( "CMakeLists.txt", self._source_subfolder )

    def _configure_cmake(self):
        cmake = CMake(self)
        cmake.configure( source_folder=os.path.join( self.build_folder, self._source_subfolder ) )
        return cmake

    def build(self):
        cmake = self._configure_cmake()
        cmake.build()

    def package(self):
        cmake = self._configure_cmake()
        cmake.install()

    def package_info(self):
        self.cpp_info.libs = tools.collect_libs(self)
        self.cpp_info.libs.append('pthread')
        self.cpp_info.libs.append('rt')