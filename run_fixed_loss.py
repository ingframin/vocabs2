from subprocess import run

for s in range(9):
    for loss in [i/10 for i in range(11)]:
        run(["./build/vocabs2.exe", "A", "0", str(loss), str(s)])
