from conan import ConanFile
from conan.tools.cmake import CMakeDeps, CMakeToolchain, CMake, cmake_layout
from conan.tools.env import VirtualRunEnv

class FastLivo2Conan(ConanFile):
    name = "fast_livo"
    version = "2.0.1"

    # Config
    settings = "os", "compiler", "build_type", "arch"
    options = {
        "with_ros": [True, False],
        "with_openmp": [True, False],
        "with_mimalloc": [True, False],
        "enable_performance_timing": [True, False]
    }
    default_options = {
        "with_ros": True,
        "with_openmp": True,
        "with_mimalloc": True,
        "enable_performance_timing": False
    }

    export_sources = (
        "CMakeLists.txt",
        "src/*",
        "include/*",
        "config/*",
        "launch/*",
        "package.xml",
        "rpg_vikit/*"
    )

    # Dependencies
    def requirements(self):
        self.requires("eigen/3.4.0")
        self.requires("boost/1.83.0")
        self.requires("sophus/1.22.10")

        # self.requires("pcl/1.13.1") # PCL is compiled locally (to fix alignment issues)
        # self.requires("opencv/4.2.0")

        # if self.options.with_openmp:
        #     self.requires("openmp/11.0.0")

        if self.options.with_mimalloc:
            self.requires("mimalloc/1.7.6")

    def generate(self):
        deps = CMakeDeps(self)
        deps.generate()

        tc = CMakeToolchain(self)
        tc.variables["WITH_ROS"] = self.options.with_ros
        tc.variables["WITH_OPENMP"] = self.options.with_openmp
        tc.variables["WITH_MIMALLOC"] = self.options.with_mimalloc
        tc.variables["ENABLE_PERFORMANCE_TIMING"] = self.options.enable_performance_timing
        tc.cache_variables["CMAKE_BUILD_TYPE"] = str(self.settings.build_type)
        
        # Enhanced debug symbols for better backtrace analysis
        if self.settings.build_type == "Debug":
            # Full debug info, no optimizations
            tc.variables["CMAKE_CXX_FLAGS_DEBUG"] = "-O0 -g3 -ggdb -fno-omit-frame-pointer"
        elif self.settings.build_type == "RelWithDebInfo":
            # Debug info with optimizations, but preserve stack frames
            tc.variables["CMAKE_CXX_FLAGS_RELWITHDEBINFO"] = "-O2 -g3 -ggdb -fno-omit-frame-pointer"

        tc.generate()

        VirtualRunEnv(self).generate()
        
    def build(self):
        cmake = CMake(self)
        cmake.configure()
        cmake.build()

    def package(self):
        cmake = CMake(self)
        cmake.install()

    def package_info(self):
        self.cpp_info.libs = ["vio", "lio", "pre", "imu_proc", "laser_mapping"]
        self.cpp_info.includedirs = ["include"]
        self.cpp_info.bindirs = ["bin"]
    
    def layout(self):
        cmake_layout(self, build_folder="build")
