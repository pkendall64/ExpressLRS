import os

Import("env")

def generateCoverageInfo(source, target, env):
    for file in os.listdir("test"):
        os.system(".pio/build/coverage/program test/"+file)
    os.system("lcov -d .pio/build/coverage/ -c -o lcov.info")
    os.system("lcov --remove lcov.info '*/tool-unity/*' '*/test/*' '*/MockArduino/*' -o filtered_lcov.info")
    # os.system("genhtml -o cov/ --demangle-cpp filtered_lcov.info")

env.AddPostAction(".pio/build/coverage/program", generateCoverageInfo)
