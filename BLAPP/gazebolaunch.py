import sys
import os
import subprocess
import PyQt5
import roslaunch

if __name__ == "__main__":
    os.system("bash map.sh " + sys.argv[1])
    sys.exit()
