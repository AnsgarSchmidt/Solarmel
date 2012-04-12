from time import gmtime, strftime
from math import sin
import random

temp_roof = 23.2;
temp_in  = 42.2;
temp_out = 30.1;
temp_b   = 60.1;
temp_m   = 70.1;
temp_t   = 80.1;
light    = 2.3;
oil      = 102;
pump     = 1;
heater   = 1;

for i in range(1000):
	add= sin((i*1.0)/100.0)
	print strftime("%Y-%m-%d %H:%M:%S",gmtime())+","+str(temp_roof+(add*40.0))+","+str(temp_out+(add*10.0))+","+str(temp_in+(add*10.0))+","+str(temp_b+(add*40.0))+","+str(temp_m+(add*40.0))+","+str(temp_t+(add*40.0))+","+str(light+(add*2.3))+","+str(oil+i)+","+str(random.randint(0,1))+","+str(random.randint(0,1));

 
