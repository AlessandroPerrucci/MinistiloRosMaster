#!/usr/bin/env python3
#coding=utf-8
import time
from Rosmaster_Lib import Rosmaster

bot = Rosmaster()


bot.set_car_motion(0.5, 0, 0)
time.sleep(5)
bot.set_car_motion(0, 0, 0)
time.sleep(1)
bot.set_car_motion(-0.5, 0, 0)
time.sleep(5)
bot.set_car_motion(0, 0, 0)
time.sleep(1)

bot.set_beep(50)

del bot