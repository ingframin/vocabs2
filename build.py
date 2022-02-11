from subprocess import run

run(["clang++", "main.cpp",
                "drone.cpp",
                "vec2.cpp",
                "comms.cpp",
                "in_out.cpp",
                "-o./build/vocabs2.exe",
                "-Wall",
                "-march=native",
                "-O3",
                "-finline-functions",
                "-fopenmp",
                "-std=c++20"])