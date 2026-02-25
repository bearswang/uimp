#! /usr/bin/env python3
from mpc_core import mpc_core
import sys

if __name__ == '__main__':
    agent_name = sys.argv[1]
    mp = mpc_core(agent_name) 
    mp.cal_vel()
