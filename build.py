from subprocess import run

run(["clang++", "./src/main.cpp",
                "./src/drone.cpp",
                "./src/vec2.cpp",
                "./src/comms.cpp",
                "./src/in_out.cpp",
                "./src/obstacles.cpp",
                "-o./build/vocabs2",
                "-Wall",
                "-march=native",
                "-O3",
                "-finline-functions",
                "-fopenmp",
                "-std=c++20"])