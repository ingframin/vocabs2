import subprocess

cmd = ["gcc",
       "main.c",
       "vec2.c",
       "drone.c",
       "-lm",
       "-o./build/vocabs2",
       "-std=c11",
       "-w",
       "-Wextra",
       "-Wfatal-errors",
       "-Wall",
       "-g"]

s = subprocess.check_output(cmd)
if s=="":
    print("Build succeded!")
else:
    print(s)
