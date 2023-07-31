from subprocess import run
run(["gcc","./src/*.c",
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
