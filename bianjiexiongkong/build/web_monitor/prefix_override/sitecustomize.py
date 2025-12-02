import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/zkxt/Documents/TK30/bianjiexiongkong/install/web_monitor'
