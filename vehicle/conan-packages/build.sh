set -e

conan create ./conan-concurrentqueue aqualink/stable
conan create ./conan-readerwriterqueue aqualink/stable
conan create ./conan-serial aqualink/stable
conan create ./mavlink2 aqualink/stable
conan create ./mavchannel aqualink/stable
