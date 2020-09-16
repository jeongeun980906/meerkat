from math import pi
data=" A -99 B -199 C -285 D -460 E -760 F 15988 G"
d2=data.split('C')
d3=d2[1].split('D')
d4=d3[1].split('E')
d5=d4[1].split('F')
gz=float(d3[0])
ax=float(d4[0])
ay=float(d5[0])
gz=gz/(131*180)*pi
ax=ax/8192*9.8
ay=ay/8192*9.8
print(gz,ax,ay)