import SMPCR1
import APrioriMpcR1
import EMPC
import random
import numpy as np
import time
import math

# dist = [3,-3]*(44/2)
# sse = SMPCR1.Controller(dist)
# print sse

n = 3
t = 40

Tf = 41
T = range(0,Tf)

M = 1
z = 1

S = []
Sd = []

S[:] = map(lambda x: M/z*(1-math.exp(-x*z)),T)
Sd[:] = map(lambda x: M/z*(1-math.exp(-x*z)),T)

H = [M/z*(1-math.exp(-x*z))-M/z*(1-math.exp(-y*z)) for x, y in zip(T[1:], T)] 
H.insert(0,S[0])
Hd = [M/z*(1-math.exp(-x*z))-M/z*(1-math.exp(-y*z)) for x, y in zip(T[1:], T)]
Hd.insert(0,Sd[0]) 

dist = [[] for _ in xrange(n)]
dist[0] = [3]*t
dist[1] = [-3]*t
dist[2] = [3,-3]*(t/2)

for i in range(3,n):
	dist[i] = random.sample(range(int(-3*1000),int(3*1000)),t)
	dist[i] = [float(x)/1000 for x in dist[i]]

dist = np.array(dist)
# print dist[0]
SSE = [[] for _ in xrange(n)]
TIME = [[] for _ in xrange(n)]

for i in range(n):
	sse_SMPC,time_SMPC = SMPCR1.Controller(dist[i],t,H,Hd)
	sse_AMPC,time_AMPC = APrioriMpcR1.Controller(dist[i],t,H,Hd)
	SSE[i].extend([sse_SMPC,sse_AMPC])
	TIME[i].append([time_SMPC,time_AMPC])

ctrl_hrzn = 2
pred_hrzn = 2
del(H[0])
del(Hd[0])
action_rule = EMPC.Controller(ctrl_hrzn,pred_hrzn,H,Hd)
cycles = t/ctrl_hrzn
print action_rule
action = [[] for _ in xrange(n)]
output = np.zeros(shape = (n,t))

for N in range(n):
	start_time = time.time()
	for i in range(t):
		rule_cycle = int(i/ctrl_hrzn)
		action[N].append(action_rule[i-rule_cycle*ctrl_hrzn][0]+sum(action_rule[i-rule_cycle*ctrl_hrzn][j-rule_cycle*ctrl_hrzn+1]*dist[N][j] for j in range(rule_cycle*ctrl_hrzn,i+1)))
		print H[0]
		print action[0][0]
		print Hd[0]
		print dist[0][0]
		output[N][i] = H[0]*action[N][i]+sum(H[i-j+1]*action[N][j-1] for j in range(i,0,-1))+Hd[0]*dist[N][i]+sum(Hd[i-j+1]*dist[N][j-1] for j in range(i,0,-1))

	sse_EMPC = sum(output[N][j]**2 for j in range(t))
	SSE[N].append(sse_EMPC)
	TIME[N].append(time.time()-start_time)
print action
print output

print SSE
print TIME
print range(-1,0,-1)





























