from subprocess import run
run(["clang ./src/*.c",
     "-o./build/vocabs2",
     "-Wall",
     "-std=c11",
     "-march=native",
     "-Wall",
     "-Wextra",
     "-Wfatal-errors",
     "-O3",
     "-finline-functions",
     "-fopenmp",
     "-lm" ])
