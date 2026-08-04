#!/usr/bin/env python
import os
import sys

# Check if godot-cpp exists, if not clone it
if not os.path.exists("godot-cpp"):
    print("godot-cpp not found. Cloning...")
    os.system("git clone --depth 1 --branch 4.2-stable https://github.com/godotengine/godot-cpp.git")

env = SConscript("godot-cpp/SConstruct")

# Add MMV2 source files
env.Append(CPPPATH=["include"])

sources = Glob("src/**/*.cpp", strings=True)
sources += Glob("godot/*.cpp", strings=True)

# Create the library
if env["platform"] == "macos":
    library = env.SharedLibrary(
        "demo/bin/libmmv2.{}.{}.framework/libmmv2.{}.{}".format(
            env["platform"], env["target"], env["platform"], env["target"]
        ),
        source=sources,
    )
else:
    library = env.SharedLibrary(
        "demo/bin/libmmv2{}{}".format(env["suffix"], env["SHLIBSUFFIX"]),
        source=sources,
    )

Default(library)
