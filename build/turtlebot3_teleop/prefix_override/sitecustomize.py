import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/amankhan-riyasat-pathan/mobile_portfolio_ws/install/turtlebot3_teleop'
