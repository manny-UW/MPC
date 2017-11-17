import math
import xlsxwriter
import random
from pyomo.environ import *
from pyomo.opt import SolverStatus, TerminationCondition
import time
import numpy as np

start_time = time.time()

#create impulse response model
t = [10]

Tf = 30
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

print H
P = 4
M = 3
N = 5
Nd = N

Q = 1
W = 0
r = [0]*(t[0]+P)
# r.insert(5,10)
# r.insert(6,10)
# r.insert(7,10)
# r.insert(8,10)
# r.insert(9,10)
# r.insert(10,10)
# r.insert(11,10)
# r.insert(12,-10)
# r.insert(13,-10)
# r.insert(14,-10)
# r.insert(15,-10)
# r.insert(16,-10)
# r.insert(17,-10)
# r.insert(18,-10)
# r.insert(19,-10)
# r.insert(20,-10)

u_lb = -2.9
u_ub = 2.9

dist_lb = -3
dist_ub = 3

# noise_lb = -0.50
# noise_ub = 0.50
#initialize vectors for optimization
# noise = np.random.normal(0,0.05,t[0])
# noise = random.sample(range(int(noise_lb*1000),int(noise_ub*1000)),t[0])
# noise = [float(x)/10000 for x in noise]

# print noise

# dist = [0.05,-0.05]*(t[0]+P)

dist = [3,-3]*(t[0]+P)
# dist.insert(0,0)
print dist

u_p = [0]*(N)
dist_p = [0]*(Nd)
y_p = []
y_actual = []
err = [0]
Err = 0

for ii in range(0,t[0]):
	print dist_p
	print u_p
	

	m = ConcreteModel()

	m.y_index = RangeSet(1,P)
	m.u_index = RangeSet(0,P)

	m.t = Var([1])
	m.y = Var(m.y_index)
	m.u = Var(m.u_index)

	#constraint rule
	# def mpc_rule(m,j):
	# 	return m.y[j] == sum(H[i]*m.u[-i+j] for i in range(1,j+1))+sum(H[i]*u_p[-i+j] for i in range(j+1,N+1))+\
	# 					 sum(Hd[i]*dist[-i+j+ii] for i in range(1,j+1))+sum(Hd[i]*dist_p[-i+j] for i in range(j+1,Nd+1))

	# def mpc_rule(m,j):
	# 	return m.y[j] == sum(H[i]*m.u[-i+j] for i in range(1,j+1))+sum(H[i]*u_p[-i+j] for i in range(j+1,N+1))+\
	# 					 sum(Hd[i]*dist_p[-i+j] for i in range(j+1,Nd+1))						 

	def mpc_rule(m,j):
		return m.y[j] == sum(H[i]*m.u[-i+j] for i in range(1,j+1))+sum(H[i]*u_p[-i+j] for i in range(j+1,N+1))+Err
	
	def _constructCon(m,t):
		for i in range(0,P+1):
			m.con1.add((i,'lo'),-m.u[i]<=-u_lb)
			m.con1.add((i,'hi'),m.u[i]<=u_ub)
			if (i>M-1):
				m.con1.add((i,'NA'),m.u[i]==0)

	m.con_index = Set(dimen=2)
	m.con1 = Constraint(m.con_index,noruleinit=True)
	m.constructCon = BuildAction(t,rule=_constructCon)
	

	#constraint and objective functions
	m.con = Constraint(m.y_index,rule=mpc_rule)
	m.obj = Constraint(expr=m.t[1]>=Q*sum((r[i+ii-1]-m.y[i])**2 for i in m.y_index)+W*sum(m.u[i]**2 for i in m.u_index))
	m.sse = Objective(expr=m.t[1])

	#solve model
	m.pprint()
	opt = SolverFactory("ipopt")
	results = opt.solve(m)
	results.write()
	print(results)

	#print objective and decision variable values
	m.solutions.store_to(results)
	results.write()

	Y = H[1]*m.u[0].value+sum(H[i]*u_p[-i+1] for i in range(2,N+1))+\
	 	Hd[1]*dist[ii]+sum(Hd[i]*dist_p[-i+1] for i in range(2,Nd+1))#+\
	 	# noise[ii]

	y_actual.append(Y)
	print Y

	err.append(Y-m.y[1].value)
	Err = sum(err)
	print Err
	#prepare for next iteration by adjusting delu_p and u_p vectors
	u_p.append(m.u[0].value)
	dist_p.append(dist[ii])
	y_p.append(m.y[1].value)

print y_actual
del u_p[:(N)]
del dist_p[:(Nd)]
print err
print Err

print (time.time()-start_time)
#EXCEL
#Controller Specifications
workbook = xlsxwriter.Workbook('SMPC.xlsx')
worksheet = workbook.add_worksheet()

R = 1
worksheet.write('B{}'.format(R),'Control Horizon')
worksheet.write('C{}'.format(R),M)
worksheet.write('E{}'.format(R),'Weighting on Output (Q)')
worksheet.write('F{}'.format(R),Q)
if (results.solver.status == SolverStatus.ok) and (results.solver.termination_condition == TerminationCondition.optimal):
    worksheet.write('I1','Feasible and Optimal')
elif (results.solver.termination_condition == TerminationCondition.infeasible):
    worksheet.write('I1','Infeasible')

R += 1 #2

worksheet.write('B{}'.format(R),'Prediction Horizon')
worksheet.write('C{}'.format(R),P)
worksheet.write('E{}'.format(R),'Weighting on Input (W)')
worksheet.write('F{}'.format(R),W)

R += 1 #3
worksheet.write('B{}'.format(R),'Lower Bound')
worksheet.write('C{}'.format(R),'Upper Bound')

R += 1 #4
worksheet.write('A{}'.format(R),'U')
worksheet.write('B{}'.format(R),u_lb)
worksheet.write('C{}'.format(R),u_ub)

R += 1 #5
worksheet.write('A{}'.format(R),'Y')
worksheet.write('B{}'.format(R),'')
worksheet.write('C{}'.format(R),'')

R += 1 #6
worksheet.write('A{}'.format(R),'d')
worksheet.write('B{}'.format(R),dist_lb)
worksheet.write('C{}'.format(R),dist_ub)

R += 1
worksheet.write('A{}'.format(R),'SSE')
worksheet.write('B{}'.format(R),sum((r[i]-y_actual[i])**2 for i in range(0,len(y_actual))))

R += 2
worksheet.write('A{}'.format(R),'Time')
worksheet.write('B{}'.format(R),'Setpoint')
worksheet.write('C{}'.format(R),'Disturbance')
worksheet.write('D{}'.format(R),'Input')
worksheet.write('E{}'.format(R),'Predicted Output')
worksheet.write('F{}'.format(R),'Actual Output')
# worksheet.write('G{}'.format(R),'Noise')
for i in range(0,t[0]):
	worksheet.write('A{}'.format(R+i+1),i)
	worksheet.write('B{}'.format(R+i+1),r[i])
	worksheet.write('C{}'.format(R+i+1),dist_p[i])
	worksheet.write('D{}'.format(R+i+1),u_p[i])
	worksheet.write('E{}'.format(R+i+2),y_p[i])
	worksheet.write('F{}'.format(R+i+2),y_actual[i])
	# worksheet.write('G{}'.format(R+i+1),noise[i])
worksheet.write('A{}'.format(R+t[0]+1),t[0])

chart = workbook.add_chart({'type':'scatter'})
chart.add_series({'name':'=Sheet1!C{}'.format(R),'categories':'=Sheet1!A{}:A{}'.format(R+1,R+2+t[0]-1),'values':'=Sheet1!C{}:C{}'.format(R+1,R+2+t[0]-1)})
worksheet.insert_chart('H1',chart)
chart = workbook.add_chart({'type':'scatter'})
chart.add_series({'name':'=Sheet1!D{}'.format(R),'categories':'=Sheet1!A{}:A{}'.format(R+1,R+2+t[0]-1),'values':'=Sheet1!D{}:D{}'.format(R+1,R+2+t[0]-1)})
worksheet.insert_chart('H16',chart)
chart = workbook.add_chart({'type':'scatter'})
chart.add_series({'name':'=Sheet1!E{}'.format(R),'categories':'=Sheet1!A{}:A{}'.format(R+1,R+2+t[0]-1),'values':'=Sheet1!E{}:E{}'.format(R+1,R+2+t[0]-1)})
chart.add_series({'name':'=Sheet1!F{}'.format(R),'categories':'=Sheet1!A{}:A{}'.format(R+1,R+2+t[0]-1),'values':'=Sheet1!F{}:F{}'.format(R+1,R+2+t[0]-1)})
worksheet.insert_chart('H31',chart)


