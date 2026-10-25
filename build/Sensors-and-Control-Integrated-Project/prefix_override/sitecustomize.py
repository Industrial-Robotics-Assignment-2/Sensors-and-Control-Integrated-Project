import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/root/git/Sensors-and-Control-Integrated-Project/install/Sensors-and-Control-Integrated-Project'
