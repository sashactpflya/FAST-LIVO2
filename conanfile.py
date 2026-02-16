import os

from conan import ConanFile
from conan.tools.cmake import CMakeDeps, CMakeToolchain, CMake, cmake_layout
from conan.tools.env import VirtualRunEnv
from conan.tools.files import copy

class FastLivo2Conan(ConanFile):
    name = "fast_livo"
    version = "2.0.1"

    # Config
    settings = "os", "compiler", "build_type", "arch"
    options = {
        "with_ros": [True, False],
        "with_openmp": [True, False],
        "with_mimalloc": [True, False],
        "enable_performance_timing": [True, False],
        "enable_tsan": [True, False],
        "enable_msan": [True, False],
        "enable_asan": [True, False],
    }
    default_options = {
        "with_ros": True,
        "with_openmp": False,
        "with_mimalloc": True,
        "enable_performance_timing": False,
        "enable_tsan": False,
        "enable_msan": False,
        "enable_asan": False
    }

    # Dependencies
    def requirements(self):
        self.requires("boost/1.83.0")
        self.requires("sophus/1.22.10", transitive_headers=True)

        self.requires("eigen/5.0.1", transitive_headers=True)
        # self.requires("eigen/5.0.1", override=True, transitive_headers=True)

        # Requires PCL with option use_avx = True
        self.requires("pcl/1.15.1", transitive_headers=True)
        self.requires("vikit/1.0.0@flya")

        # if self.options.with_openmp:
        #     self.requires("openmp/11.0.0")
        self.requires("flya-std-extensions/[~3]@flya", transitive_headers=True)
        self.requires("cyclonedds/0.10.5")


        self.requires("opencv/3.4.20", transitive_headers=True)

        self.requires("ros2/jazzy.2", transitive_headers=True)
        self.requires("fmt/10.2.1", override=True)


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
        tc.cache_variables["ENABLE_TSAN"] = "ON" if self.options.enable_tsan else "OFF"
        tc.cache_variables["ENABLE_MSAN"] = "ON" if self.options.enable_msan else "OFF"
        tc.cache_variables["ENABLE_ASAN"] = "ON" if self.options.enable_asan else "OFF"
        
        # Enhanced debug symbols for better backtrace analysis
        if self.settings.build_type == "Debug":
            # Full debug info, no optimizations
            tc.variables["CMAKE_CXX_FLAGS_DEBUG"] = "-O0 -g3 -ggdb -fno-omit-frame-pointer"
        elif self.settings.build_type == "RelWithDebInfo":
            # Debug info with optimizations, but preserve stack frames
            tc.variables["CMAKE_CXX_FLAGS_RELWITHDEBINFO"] = "-O2 -g3 -ggdb -fno-omit-frame-pointer"

        tc.generate()

        runenv = VirtualRunEnv(self)
        # Ensure CycloneDDS is used and configured by default.
        runenv.environment().define("RMW_IMPLEMENTATION", "rmw_cyclonedds_cpp")
        runenv.environment().define(
            "CYCLONEDDS_URI",
            os.path.join(self.source_folder, "etc", "cyclonedds_host.xml"),
        )
        runenv.generate()
        self._update_latest_symlink()

    def configure(self):
        self.options["cv_bridge"].shared = True
        self.options["cv_bridge"].opencv = False
        
    def build(self):
        cmake = CMake(self)
        cmake.configure()
        cmake.build()

    def package(self):
        cmake = CMake(self)
        cmake.install()

    def package_info(self):

        self.cpp_info.libs = ["laser_mapping", "imu_proc", "pre", "lio", "vio"]
        self.cpp_info.system_libs.append("cv_bridge::cv_bridge")  # Declare cv_bridge as a system library

        reqs = [
            "boost::boost",
            "boost::filesystem",
            "boost::system",
            "pcl::pcl",
            "ros2::ros2",
            "sophus::sophus",
            "flya-std-extensions::flya-std-extensions",
            "vikit::vikit_common",
            "cyclonedds::cyclonedds",
            "eigen::eigen",
            "opencv::opencv"
        ]

        if self.options.with_mimalloc:
            reqs.append("mimalloc::mimalloc")

        self.cpp_info.requires = reqs
        self.cpp_info.includedirs = ["include"]
        self.cpp_info.bindirs = ["bin"]
    
    def layout(self):
        # Keep the repository root as the source dir for CMake
        cmake_layout(self, src_folder=".")

    def _update_latest_symlink(self):
        """Create/update build/latest symlink pointing to current build folder."""
        current_build = os.path.abspath(self.build_folder)
        build_root = os.path.abspath(os.path.join(self.build_folder, ".."))
        latest_link = os.path.join(build_root, "latest")

        try:
            if os.path.islink(latest_link) or os.path.exists(latest_link):
                # Only replace if it is a symlink
                if os.path.islink(latest_link):
                    os.unlink(latest_link)
                else:
                    return  # Do not override a real directory/file
            os.symlink(current_build, latest_link)
        except OSError:
            # Non-fatal; skip if filesystem does not support symlinks
            pass

    def export_sources(self):
        copy(self, "CMakeLists.txt", src=self.recipe_folder, dst=self.export_sources_folder)
        copy(self, "src/*",          src=self.recipe_folder, dst=self.export_sources_folder)
        copy(self, "include/*",      src=self.recipe_folder, dst=self.export_sources_folder)
        copy(self, "config/*",       src=self.recipe_folder, dst=self.export_sources_folder)
        copy(self, "etc/*",          src=self.recipe_folder, dst=self.export_sources_folder)
        copy(self, "launch/*",       src=self.recipe_folder, dst=self.export_sources_folder)
        copy(self, "package.xml",    src=self.recipe_folder, dst=self.export_sources_folder)
