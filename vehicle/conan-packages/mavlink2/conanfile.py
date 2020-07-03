from conans import ConanFile, CMake, tools
import os

class Mavlink2Conan(ConanFile):
    name = "mavlink2"
    version = "1.0.0"
    settings = "os", "compiler", "arch", "build_type"
    description = "Mavlink2 utilities and messages"
    license = ("Simplified BSD", "zlib")
    repo_url = "https://github.com/mavlink/c_library_v2"
    author = "Aqualink Developer <developer@aqualink.org>"

    sha1 = "477a5b6d54ce7c269c90c4c2e73b9f57f9eba3de"
    repo_name="c_library_v2"
    _source_subfolder = "%s-%s" % ( repo_name, sha1 )

    def source(self):
        tools.get("%s/archive/%s.zip" % (self.repo_url, self.sha1))

    def package(self):
        self.copy( "*", src=self._source_subfolder, dst="include/mavlink2" )

    def package_id(self):
        self.info.header_only()