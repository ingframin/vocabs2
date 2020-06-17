from subprocess import run

for s in range(5, 100, 5):
    for loss in [0.1, 0.3, 0.5, 0.7]:
        run(["./build/vocabs2.exe", "A", "0", str(loss), str(s)])

for s in range(100, 1000, 50):
    for loss in [0.1, 0.3, 0.5, 0.7]:
        run(["./build/vocabs2.exe", "A", "0", str(loss), str(s)])
