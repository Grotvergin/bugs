import sys
import os
import subprocess
import PyQt5
import roslaunch

if __name__ == "__main__":
    os.system("bash buglauncher.sh " + sys.argv[1] +" "+sys.argv[2]+" "+sys.argv[3])
    sys.exit()
