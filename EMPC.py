from pyomo.environ import *
from pyomo.opt import SolverStatus, TerminationCondition
import numpy as np
import math
import xlsxwriter
import string
import time


def Controller(ctrl,pred,H,Hd):
	start_time = time.time()
	# # t = 10
	# Tf = 101
	# T = range(1,Tf)

	# z = 1
	# M = 1

	# S = []
	# Sd = []

	# S[:] = map(lambda x: M/z*(1-math.exp(-x*z)),T)
	# Sd[:] = map(lambda x: M/z*(1-math.exp(-x*z)),T)

	H = H#[M/z*(1-math.exp(-x*z))-M/z*(1-math.exp(-y*z)) for x, y in zip(T[1:], T)] 
	# H.insert(0,S[0])
	Hd = Hd#[M/z*(1-math.exp(-x*z))-M/z*(1-math.exp(-y*z)) for x, y in zip(T[1:], T)]
	# Hd.insert(0,Sd[0]) 

	# print H

	T = [0]			#time periods 
	# tt = len(T) 				#degree of recourse

	m = ConcreteModel()
	m.var_index = Set(dimen = 4)							#index for original variables. i - index, j - 0=deterministic 1=slope 2=intercept, k - time period, l - recourse period

	m.u = Var(m.var_index)
	m.y = Var(m.var_index)
	m.y1 = Var(m.var_index)

	m.di_index = Set(dimen = 4)
	m.di = Var(m.di_index)
	m.di_index.add((0,0,0,0))

	unc_vars = np.array([
						[m.di[(0,0,0,0)]],
						[m.di[(0,0,0,0)]],
						[]])

	unc_coeff = np.array([
						 [-1],
						 [1],
						 []])

	unc_rhs = np.array([
					   [3],
					   [3],
					   []])

	di0 = 0

	m.dvar_index = Set(dimen = 4)							#index for dual variables. i - deterministic constraint, j - uncertainty set constraint, k - time period, l - recourse period
	m.dvar = Var(m.dvar_index,domain = NonNegativeReals)		#dual variable

	m.t = Var([1])



	
	ctrl_hrzn = ctrl
	pred_hrzn = pred

	Q = 10
	W = 0
	r = 0

	UBy = 'NA'
	LBy = 'NA'
	UBu = 2.9
	LBu = -2.9
	UBE = 0.1
	LBE = -0.1

	def _constructCon(m,T):
		# print T
		t = T 			#assign current time step




		# #Constraints are written as sum(decision variables*coefficients)<=right hand side
		# #decision variables for the constraints
		#pred hrzn is the number of rows or equality constraints
		#2*pred hrzn is the number of rows of inequality constraints (LB and UB on the outputs)
		#2*ctrl hrzn is the number of rows of inequality constraints (LB and UB on the inputs)
		# Con_count = pred_hrzn+2*ctrl_hrzn+pred_hrzn+ctrl_hrzn+pred_hrzn #pred_hrzn = MPC constraints, 2*pred_hrzn = UB/LB outputs, 2*ctrl_hrzn = UB/LB on inputs, pred_hrzn = define for obj, ctrl_hrzn = define for obj
		Con_count = 0
		Con_count += pred_hrzn
		Con_count += 2*ctrl_hrzn
		Con_count += pred_hrzn
		Con_count += ctrl_hrzn
		Con_count += 2*pred_hrzn
		# Con_count += pred_hrzn

		decvars = [[] for _ in xrange(Con_count+1)]
		decvars_idx = [[] for _ in xrange(Con_count+1)] 	#1 - y,2 - u,3 - d,4 - t
		coeff = [[] for _ in xrange(Con_count+1)]
		rhs = [[] for _ in xrange(Con_count+1)]
		con_cert = [[] for _ in xrange(Con_count+1)]
		con_eq = [[] for _ in xrange(Con_count+1)]

		# print decvars
		R = 0
		for i in range(R,pred_hrzn):
			m.var_index.add((0,0,i,0))
			m.di_index.add((0,0,i,0))
			decvars[i].append((m.y[0,0,i,0],'adj'))
			decvars_idx[i].append((1,0,0,i,0))
			coeff[i].append(1)
			rhs[i].append(0)
			con_cert[i].append('unc')
			con_eq[i].append('eq')

			for j in range(ctrl_hrzn-1,-1,-1):
				if (j <= i):
					decvars[i].append((m.u[0,0,j,0],'adj'))
					decvars_idx[i].append((2,0,0,j,0))
			for j in range(0,ctrl_hrzn):
				if (j <= i):
					if (i < ctrl_hrzn):
						coeff[i].append(-H[j])
					elif (i >= ctrl_hrzn):
						coeff[i].append(-H[j+i-(ctrl_hrzn-1)])
			for j in range(pred_hrzn,-1,-1):
				if (j <= i):
					decvars[i].append((m.di[0,0,j,0],'unc'))
					decvars_idx[i].append((3,0,0,j,0))
			for j in range(0,pred_hrzn):
				if (j <= i):
					coeff[i].append(-Hd[j])

		# R += pred_hrzn			
		# for i in range(R,R+pred_hrzn):
		# 	decvars[i].append((m.y[0,0,i-R,0],'adj'))
		# 	decvars_idx[i].append((1,0,0,i-R,0))
		# 	coeff[i].append(1)
		# 	rhs[i].append(UBy)
		# 	con_cert[i].append('unc')
		# 	con_eq[i].append('ineq')

		# R += pred_hrzn
		# for i in range(R,R+pred_hrzn):
		# 	decvars[i].append((m.y[0,0,i-R,0],'adj'))
		# 	decvars_idx[i].append((1,0,0,i-R,0))
		# 	coeff[i].append(-1)
		# 	rhs[i].append(-LBy)
		# 	con_cert[i].append('unc')
		# 	con_eq[i].append('ineq')

		R += pred_hrzn
		for i in range(R,R+ctrl_hrzn):
			decvars[i].append((m.u[0,0,i-R,0],'adj'))
			decvars_idx[i].append((2,0,0,i-R,0))
			coeff[i].append(1)
			rhs[i].append(UBu)
			con_cert[i].append('unc')
			con_eq[i].append('ineq')

		R += ctrl_hrzn
		for i in range(R,R+ctrl_hrzn):
			decvars[i].append((m.u[0,0,i-R,0],'adj'))
			decvars_idx[i].append((2,0,0,i-R,0))
			coeff[i].append(-1)
			rhs[i].append(-LBu)
			con_cert[i].append('unc')
			con_eq[i].append('ineq')

		R += ctrl_hrzn
		for i in range(R,R+pred_hrzn):
			decvars[i].append(m.y[0,0,i-R,0])
			decvars_idx[i].append((1,0,0,i-R,0))
			con_cert[i].append('def')

		# R += pred_hrzn
		# for i in range(R,R+pred_hrzn):
		# 	decvars[i].append(m.y1[0,0,i-R,0])
		# 	decvars_idx[i].append((1,0,0,i-R,0))
		# 	con_cert[i].append('def')

		R += pred_hrzn
		for i in range(R,R+ctrl_hrzn):
			decvars[i].append(m.u[0,0,i-R,0])
			decvars_idx[i].append((2,0,0,i-R,0))
			con_cert[i].append('def')

		R += ctrl_hrzn
		for i in range(R,R+pred_hrzn):
			decvars[i].append((m.y[0,0,i-R,0],'adj'))
			decvars_idx[i].append((1,0,0,i-R,0))
			coeff[i].append(1)
			rhs[i].append(r-LBE)
			con_cert[i].append('unc')
			con_eq[i].append('ineq')


		R += pred_hrzn
		for i in range(R,R+pred_hrzn):
			decvars[i].append((m.y[0,0,i-R,0],'adj'))
			decvars_idx[i].append((1,0,0,i-R,0))
			coeff[i].append(-1)
			rhs[i].append(UBE-r)
			con_cert[i].append('unc')
			con_eq[i].append('ineq')

		# R += ctrl_hrzn
		# decvars[R].append((m.y[0,0,pred_hrzn-1,0],'adj'))
		# decvars_idx[R].append((1,0,0,pred_hrzn-1,0))
		# coeff[R].append(1)
		# rhs[R].append(r-LBE)
		# con_cert[R].append('unc')
		# con_eq[R].append('ineq')

		# R += 1
		# decvars[R].append((m.y[0,0,pred_hrzn-1,0],'adj'))
		# decvars_idx[R].append((1,0,0,pred_hrzn-1,0))
		# coeff[R].append(-1)
		# rhs[R].append(UBE-r)
		# con_cert[R].append('unc')
		# con_eq[R].append('ineq')

		# R += ctrl_hrzn
		# decvars[R].append((m.t[1],'cert'))
		# decvars_idx[R].append((4,0,0,0,0))
		# coeff[R].append(1)
		# rhs[R].append(pred_hrzn*r)
		# con_cert[R].append('unc')
		# con_eq[R].append('ineq')
		# for i in range(R,R+pred_hrzn):
		# 	decvars[R].append((m.y[0,0,i-R,0],'adj'))
		# 	decvars_idx[R].append((1,0,0,i-R,0))
		# 	coeff[R].append(-1)

		decvars = np.array(decvars)
		decvars_idx = np.array(decvars_idx)
		coeff = np.array(coeff)
		rhs = np.array(rhs)
		con_cert = np.array(con_cert)
		con_eq = np.array(con_eq)
		# print H
		# print coeff
		# print rhs
		# print con_cert
		# print con_eq
	########################################################################MODIFY DECISION VARIABLE AND INDEX LISTS FOR THE GIVEN CONSTRAINT###############################################################################################
		for i in range(0,len(decvars)-1):																		#select the constraint from the array of constraints	
			if (con_cert[i][0] == 'unc'):
	################################################################################################INEQUALITY CONSTRAINTS##################################################################################################################
				if (con_eq[i][0] == 'ineq'):																		#is constraint uncertain?
					for j in range(0,len(decvars[i])):																#select the decision variable from constraint i
						if (decvars[i][j][1] == 'adj'):																#is decision variable adjustable?
							
							var_time = decvars_idx[i][j][3]

							for k in range(0,var_time+1):																	#begin with t=0 and add 1 slope, adding one more slope than the previous iteration each time period
								coeff[i].extend([coeff[i][j]])														#extend coefficient for slope(s)
							coeff[i].extend([coeff[i][j]]) 															#extend coefficient for intercept
							coeff[i][j] = 0																			#original coefficient to 0

							a = decvars_idx[i][j][0]
							b = decvars_idx[i][j][1]																#find the leading index of the decision variable being treated
							for k in range(var_time,-1,-1):																#begin with t=0 and add 1 slope, adding one more slope than the previous iteration each time period
								if (b,1,var_time,k) not in m.var_index:													#has this slope already been used? has this decision variable appeared in a previous constraint?
										m.var_index.add((b,1,var_time,k))													#if not, add the index to the decision variable index set
										decvars[i].extend([(decvars[i][j][0].parent_component()[b,1,var_time,k],'slp')])	#add the newly created slope to the list of decision variables for the constraint
										decvars_idx[i].extend([(a,b,1,var_time,k)])
								else:																				#if the slope not added above, add here
									decvars[i].extend([(decvars[i][j][0].parent_component()[b,1,var_time,k],'slp')])
									decvars_idx[i].extend([(a,b,1,var_time,k)])

							if (b,2,var_time,0) not in m.var_index:														#has this intercept already been used?
								m.var_index.add((b,2,var_time,0))															#if not, add the index to the decision variable index set
								decvars[i].extend([(decvars[i][j][0].parent_component()[b,2,var_time,0],'int')])			#add the newly create slope to the list of decision variables for the constraint
								decvars_idx[i].extend([(a,b,2,var_time,0)])
							else:																					#if the intercept not added above, add here
								decvars[i].extend([(decvars[i][j][0].parent_component()[b,2,var_time,0],'int')])			
								decvars_idx[i].extend([(a,b,2,var_time,0)])
	#############################################################################PAIR ARC DECISION VARIABLES WITH APPROPRIATE CONSTRAINTS###################################################################################################
					D = []																							#empty list to store the deterministic decision variables for the ARC problem
					for k in range(0,len(decvars[i])):																#loop through the decision variables of the constraint														
						if (decvars[i][k][1] == 'cert') or (decvars[i][k][1] == 'int'):								#is the decision variable deterministic - certain or an intercept?
							D.append(k)																				#append to list for constraints of type 1 - one per time period

					F = [[] for _ in xrange(var_time+1)]																	#empty array to store the uncertain decision variables for the ARC problem
					for k in range(0,var_time+1):																			#loop through the periods of recourse - one recourse period corresponds to one row in array F
						for l in range(0,len(decvars[i])):															#loop through the decision variables of the constraint
							if (decvars_idx[i][l][4] == k): 														#does the decision variable belong to recourse period k AND is it uncertain OR a slope?
								if ((decvars[i][l][1] == 'unc') or (decvars[i][l][1] == 'slp')):	
									F[k].append(l)																	#append to row k 
					F = np.array(F)																					#convert F into array
					# print F
	##########################################################################################INTRODUCE DUAL VARIABLES######################################################################################################################
					for j in range(0,len(unc_vars)-1):
						for tt in range(0,var_time+1):
							m.dvar_index.add((i,j,t,tt))															#add the dual variable. indexing: i - constraint, j - uncertainty set constraint, k - time period, l - recourse period
	##########################################################################################DUALIZING THE OBJECTIVE FUNCTION##############################################################################################################
					# if i == 0:
					# 	if t == 0:
					# 		m.con.add((i,t,0),sum(decvars[i][D[j]][0]*coeff[i][D[j]] for j in range(0,len(D)))-rhs[i][0]>=sum(unc_rhs[j][0]*m.dvar[(i,j,t,tt)] for j in range(0,len(unc_vars)-1) for tt in range(0,t+1)))

					# 		m.con.add((i,t,t+1),sum(m.dvar[(i,j,t,t)]*unc_rhs[j][0] for j in range(0,len(unc_vars)-1))>=sum(-1*decvars[i][F[t][j]][0]*coeff[i][F[t][j]] for j in range(0,len(F[t]))))
						
					# 	else:
					# 		m.con[i,0,0].set_value(m.con[i,0,0].body - sum(decvars[i][D[j]][0]*coeff[i][D[j]] for j in range(0,len(D)))+sum(unc_rhs[j][0]*m.dvar[(i,j,t,t)] for j in range(0,len(unc_vars)-1))<=m.con[i,0,0].upper)

					# 		m.con.add((i,t,t+1),sum(m.dvar[(i,j,t,t)]*unc_rhs[j][0] for j in range(0,len(unc_vars)-1))>=sum(-1*decvars[i][F[t][j]][0]*coeff[i][F[t][j]] for j in range(0,len(F[t]))))
	###################################################################################################BUILD CONSTRAINTS####################################################################################################################
					m.con.add((i,t,0),rhs[i][0]-sum(decvars[i][D[j]][0]*coeff[i][D[j]] for j in range(0,len(D)))>=sum(unc_rhs[j][0]*m.dvar[(i,j,t,tt)] for j in range(0,len(unc_vars)-1) for tt in range(0,var_time+1)))

					for tt in range(var_time+1,0,-1):
						m.con.add((i,t,tt),sum(m.dvar[(i,j,t,tt-1)]*unc_coeff[j][0] for j in range(0,len(unc_vars)-1))==sum(decvars[i][F[tt-1][j]][0]*coeff[i][F[tt-1][j]] for j in range(0,len(F[tt-1]))))
	##################################################################################################EQUALITY CONSTRAINTS##################################################################################################################
				elif (con_eq[i][0] == 'eq'):
					for j in range(0,len(decvars[i])):																#select the decision variable from constraint i
						if (decvars[i][j][1] == 'adj'):																#is decision variable adjustable?
							
							var_time = decvars_idx[i][j][3]

							for k in range(0,var_time+1):																	#begin with t=0 and add 1 slope, adding one more slope than the previous iteration each time period
								coeff[i].extend([coeff[i][j]])														#extend coefficient for slope(s)
							coeff[i].extend([coeff[i][j]]) 															#extend coefficient for intercept
							coeff[i][j] = 0																			#original coefficient to 0

							a = decvars_idx[i][j][0]
							b = decvars_idx[i][j][1]																#find the leading index of the decision variable being treated
							for k in range(var_time,-1,-1):																#begin with t=0 and add 1 slope, adding one more slope than the previous iteration each time period
								if (b,1,var_time,k) not in m.var_index:													#has this slope already been used? has this decision variable appeared in a previous constraint?
										m.var_index.add((b,1,var_time,k))													#if not, add the index to the decision variable index set
										decvars[i].extend([(decvars[i][j][0].parent_component()[b,1,var_time,k],'slp')])	#add the newly created slope to the list of decision variables for the constraint
										decvars_idx[i].extend([(a,b,1,var_time,k)])
								else:																				#if the slope not added above, add here
									decvars[i].extend([(decvars[i][j][0].parent_component()[b,1,var_time,k],'slp')])
									decvars_idx[i].extend([(a,b,1,var_time,k)])

							if (b,2,var_time,0) not in m.var_index:														#has this intercept already been used?
								m.var_index.add((b,2,var_time,0))															#if not, add the index to the decision variable index set
								decvars[i].extend([(decvars[i][j][0].parent_component()[b,2,var_time,0],'int')])			#add the newly create slope to the list of decision variables for the constraint
								decvars_idx[i].extend([(a,b,2,var_time,0)])
							else:																					#if the intercept not added above, add here
								decvars[i].extend([(decvars[i][j][0].parent_component()[b,2,var_time,0],'int')])			
								decvars_idx[i].extend([(a,b,2,var_time,0)])
	#############################################################################PAIR ARC DECISION VARIABLES WITH APPROPRIATE CONSTRAINTS###################################################################################################
					D = []																							#empty list to store the deterministic decision variables for the ARC problem
					for k in range(0,len(decvars[i])):																#loop through the decision variables of the constraint														
						if (decvars[i][k][1] == 'cert') or (decvars[i][k][1] == 'int'):								#is the decision variable deterministic - certain or an intercept?
							D.append(k)																				#append to list for constraints of type 1 - one per time period
					con_time = decvars_idx[i][0][3]
					# print D

					F = [[] for _ in xrange(con_time+1)]																	#empty array to store the uncertain decision variables for the ARC problem
					for k in range(0,con_time+1):																			#loop through the periods of recourse - one recourse period corresponds to one row in array F
						for l in range(0,len(decvars[i])):															#loop through the decision variables of the constraint
							if (decvars_idx[i][l][4] == k): 														#does the decision variable belong to recourse period k AND is it uncertain OR a slope?
								if (decvars[i][l][1] == 'slp'):	
									F[k].append(l)
					F = np.array(F)																					#convert F into array

					G = [[] for _ in xrange(con_time+1)]																	#empty array to store the uncertain decision variables for the ARC problem
					for k in range(0,con_time+1):																			#loop through the periods of recourse - one recourse period corresponds to one row in array F
						for l in range(0,len(decvars[i])):															#loop through the decision variables of the constraint
							if (decvars_idx[i][l][3] == k): 														#does the decision variable belong to recourse period k AND is it uncertain OR a slope?
								if (decvars[i][l][1] == 'unc'):	
									G[k].append(l)
					G = np.array(G)																					#convert F into array
	###################################################################################################BUILD CONSTRAINTS####################################################################################################################
					m.con.add((i,0,0),sum(decvars[i][D[j]][0]*coeff[i][D[j]] for j in range(0,len(D))) == rhs[i][0])

					for tt in range(con_time+1,0,-1):
						m.con.add((i,0,tt),sum(decvars[i][F[tt-1][j]][0]*coeff[i][F[tt-1][j]] for j in range(0,len(F[tt-1])))+coeff[i][G[tt-1][0]] == 0)
	###################################################################################################DEFINITION CONSTRAINTS###############################################################################################################
			elif (con_cert[i][0] == 'def'):
				# if (decvars[i][0].parent_component() == y):
					m.con.add((i,0,0),decvars[i][0] == decvars[i][0].parent_component()[0,2,decvars_idx[i][0][3],0]+sum(decvars[i][0].parent_component()[0,1,decvars_idx[i][0][3],j]*di0 for j in range(0,decvars_idx[i][0][3]+1)))
					# if (decvars[i][0].parent_component() == m.u):
					# 	tt = decvars_idx[i][0][3]
					# 	for j in range(tt):
					# 		m.con.add((i,0,j),abs(m.u[(0,1,tt,j)]) >= 0.1*abs(m.u[0,1,tt,j+1]))
				# else:
					# m.con.add((i,0,0),decvars[i][0] == decvars[i][0].parent_component()[0,2,decvars_idx[i][0][3],0]+sum(decvars[i][0].parent_component()[0,1,decvars_idx[i][0][3],j]*(-1)*unc_rhs[0][0] for j in range(0,decvars_idx[i][0][3]+1)))
	################################################################################################DETERMINISTIC CONSTRAINTS###############################################################################################################
			else:
				m.con.add((i,0,0),sum(decvars[i][j][0]*coeff[i][j] for j in range(0,len(decvars[i])))>=rhs[i][0])
	##########################################################################################################SOLVE#########################################################################################################################

	m.con_idx = Set(dimen=3)
	m.con = Constraint(m.con_idx,noruleinit=True)
	m.constructCon = BuildAction(T,rule=_constructCon)

	m.cost = Objective(expr = m.t[1])

	def obj_rule(m):
		return (m.t[1]>=Q*sum((r-m.y[0,0,i,0])**2 for i in range(0,pred_hrzn))+W*sum(m.u[0,0,i,0]**2 for i in range(0,ctrl_hrzn)))

	m.cost_fcn = Constraint(rule = obj_rule)

	# m.pprint()
	opt = SolverFactory("ipopt")
	results = opt.solve(m)
	# results.write()
	# print(results)

	# m.solutions.store_to(results)
	# results.write()

	result = [[] for _ in xrange(ctrl)]

	for i in range(ctrl):
		result[i].append(m.u[0,2,i,0].value)
		for j in range(i+1):
			result[i].append(m.u[0,1,i,j].value)

	# results = np.array(result)
	# print result
	return result
# print(time.time()-start_time)


######################################################################################################WRITE TO EXCEL####################################################################################################################
# workbook = xlsxwriter.Workbook('EMPC All Error Bounded.xlsx')
# worksheet = workbook.add_worksheet()


# #Controller Specifications
# R = 1
# worksheet.write('B{}'.format(R),'Control Horizon')
# worksheet.write('C{}'.format(R),ctrl_hrzn)
# worksheet.write('E{}'.format(R),'Weighting on Output (Q)')
# worksheet.write('F{}'.format(R),Q)


# R += 1 #2

# worksheet.write('B{}'.format(R),'Prediction Horizon')
# worksheet.write('C{}'.format(R),pred_hrzn)
# worksheet.write('E{}'.format(R),'Weighting on Input (W)')
# worksheet.write('F{}'.format(R),W)

# R += 1
# worksheet.write('B{}'.format(R),'Gain')
# worksheet.write('C{}'.format(R),1/z)

# R += 1
# worksheet.write('B{}'.format(R),'Time Constant')
# worksheet.write('C{}'.format(R),1/z)

# R += 2 #3
# worksheet.write('B{}'.format(R),'Lower Bound')
# worksheet.write('C{}'.format(R),'Upper Bound')

# R += 1 #4
# worksheet.write('A{}'.format(R),'U')
# worksheet.write('B{}'.format(R),LBu)
# worksheet.write('C{}'.format(R),UBu)
# worksheet.write('E{}'.format(R),'Solution:')
# if (results.solver.status == SolverStatus.ok) and (results.solver.termination_condition == TerminationCondition.optimal):
#     worksheet.write('F{}'.format(R),'Feasible and Optimal')
# elif (results.solver.termination_condition == TerminationCondition.infeasible):
#     worksheet.write('F{}'.format(R),'Infeasible')


# R += 1 #5
# worksheet.write('A{}'.format(R),'Y')
# worksheet.write('B{}'.format(R),LBy)
# worksheet.write('C{}'.format(R),UBy)

# R += 1 #6
# worksheet.write('A{}'.format(R),'d')
# worksheet.write('B{}'.format(R),-unc_rhs[0][0])
# worksheet.write('C{}'.format(R),unc_rhs[1][0])

# R += 1
# worksheet.write('A{}'.format(R),'Error')
# worksheet.write('B{}'.format(R),LBE)
# worksheet.write('C{}'.format(R),UBE)

# #Controller Results
# R +=2 #8
# worksheet.write('B{}'.format(R),'Nominal Disturbance')
# for i in range(2,pred_hrzn+2):
# 	worksheet.write(string.uppercase[i]+'{}'.format(R),di0)
# R +=1 #9
# worksheet.write('B{}'.format(R),'Disturbance Reading')
# for i in range(2,pred_hrzn+2):
# 	worksheet.write(string.uppercase[i]+'{}'.format(R),di0)
# R +=1 #8
# worksheet.write('B{}'.format(R),'Delta Disturbance')
# for i in range(2,pred_hrzn+2):
# 	worksheet.write_formula(string.uppercase[i]+'{}'.format(R),'='+string.uppercase[i]+'{}-'.format(R-1)+string.uppercase[i]+'{}'.format(R-2))



# R += 1 #9
# worksheet.write('B{}'.format(R),'Setpoint')
# worksheet.write_formula(string.uppercase[pred_hrzn+3]+'{}'.format(R),'='+str(Q)+'*SUMXMY2(C{}:'.format(R)+string.uppercase[pred_hrzn+1]+'{},'.format(R)+string.uppercase[pred_hrzn+3]+'{}:'.format(ctrl_hrzn+R+5)+string.uppercase[pred_hrzn+3]+'{})+'.format(ctrl_hrzn+pred_hrzn+R+4)+str(W)+'*SUMSQ('+string.uppercase[pred_hrzn+3]+'{}:'.format(R+2)+string.uppercase[pred_hrzn+3]+'{})'.format(R+ctrl_hrzn+2))
# for i in range(2,pred_hrzn+2):
# 	worksheet.write(string.uppercase[i]+'{}'.format(R),r)
# R += 1 #10
# worksheet.write('A{}'.format(R),'INPUT LABEL')
# worksheet.write('B{}'.format(R),'INTERCEPT')
# worksheet.write(string.uppercase[pred_hrzn+3]+'{}'.format(R),'CONTROL ACTION')
# for i in range(2,pred_hrzn+2):
# 	worksheet.write(string.uppercase[i]+'{}'.format(R),'Slope for t = {}'.format(i-2))



# #Inputs
# R += 1 #11
# ctrl = 0
# for i in range(R,ctrl_hrzn+R):
# 	worksheet.write('A{}'.format(i),ctrl)
# 	ctrl += 1
# 	worksheet.write('B{}'.format(i),m.u[0,2,i-R,0].value)
# 	worksheet.write_formula(string.uppercase[pred_hrzn+3]+'{}'.format(i),'=SUMPRODUCT(C{}:'.format(i)+string.uppercase[pred_hrzn+1]+'{},C{}:'.format(i,R-3)+string.uppercase[pred_hrzn+1]+'{})+B{}'.format(R-3,i))
# 	for j in range(2,pred_hrzn+2):
# 		if (j <= i-(R-2)):
# 			worksheet.write(string.uppercase[j]+'{}'.format(i),m.u[0,1,i-R,j-2].value)

# chart = workbook.add_chart({'type':'scatter'})
# chart.add_series({'name':'=Sheet1!'+string.uppercase[1]+'{}'.format(R-3),'categories':'=Sheet1!A{}:A{}'.format(R,R+ctrl_hrzn),'values':'=Sheet1!'+string.uppercase[2]+'{}:'.format(R-3)+string.uppercase[pred_hrzn+1]+'{}'.format(R-3)})
# worksheet.insert_chart(string.uppercase[pred_hrzn+5]+'1',chart)

# chart = workbook.add_chart({'type':'scatter'})
# chart.add_series({'name':'=Sheet1!'+string.uppercase[pred_hrzn+3]+'{}'.format(R-1),'categories':'=Sheet1!A{}:A{}'.format(R,R+ctrl_hrzn),'values':'=Sheet1!'+string.uppercase[pred_hrzn+3]+'{}:'.format(R)+string.uppercase[pred_hrzn+3]+'{}'.format(R+ctrl_hrzn)})
# worksheet.insert_chart(string.uppercase[pred_hrzn+5]+'16',chart)

# R += ctrl_hrzn+2

# #Outputs
# worksheet.write('A{}'.format(R),'OUTPUT LABEL')
# worksheet.write('B{}'.format(R),'INTERCEPT')
# for i in range(2,pred_hrzn+2):
# 	worksheet.write(string.uppercase[i]+'{}'.format(R),'Slope for t = {}'.format(i-2))
# worksheet.write(string.uppercase[pred_hrzn+3]+'{}'.format(R),'OUTPUT')


# R += 1
# output = 1
# for i in range(R,R+pred_hrzn):
# 	worksheet.write('A{}'.format(i),output)
# 	output += 1
# 	worksheet.write('B{}'.format(i),m.y[0,2,i-R,0].value)
# 	worksheet.write_formula(string.uppercase[pred_hrzn+3]+'{}'.format(i),'=SUMPRODUCT(C{}:'.format(i)+string.uppercase[pred_hrzn+1]+'{},C{}:'.format(i,R-ctrl_hrzn-6)+string.uppercase[pred_hrzn+1]+'{})+B{}'.format(R-ctrl_hrzn-6,i))	
# 	for j in range(2,pred_hrzn+2):
# 		if (j <= i-(R-2)):
# 			worksheet.write(string.uppercase[j]+'{}'.format(i),m.y[0,1,i-R,j-2].value)

# chart = workbook.add_chart({'type':'scatter'})
# chart.add_series({'name':'=Sheet1!'+string.uppercase[pred_hrzn+3]+'{}'.format(R-1),'categories':'=Sheet1!A{}:A{}'.format(R,R+pred_hrzn),'values':'=Sheet1!'+string.uppercase[pred_hrzn+3]+'{}:'.format(R)+string.uppercase[pred_hrzn+3]+'{}'.format(R+pred_hrzn)})
# worksheet.insert_chart(string.uppercase[pred_hrzn+5]+'31',chart)

# R += pred_hrzn+2


###########################################################################################PLANT MODEL MISMATCHRIATE CONSTRAINTS#######################################################################################################
# from random import randint
# dist=[randint(-1,1) for p in range(0,pred_hrzn+1)]
# alph = 0.99
# dist = [x / 10 for x in dist]

# Y = []
# for i in range(1,pred_hrzn+1):
# 	y_real = (m.u[0,2,i-1,0].value+sum(m.u[0,1,i-1,j].value*dist[j] for j in range(0,i)))*(1-alph*math.exp(-i)+1-dist[i]*math.exp(-i))
# 	Y.append(y_real)

# obj_real = Q*sum((r-Y[i])**2 for i in range(0,pred_hrzn))+W*sum(m.u[0,2,i,0].value+m.u[0,1,i,j].value*dist[j] for i in range(0,ctrl_hrzn) for j in range(0,i))

# worksheet.write('A{}'.format(ctrl_hrzn+pred_hrzn+16),'Disturbance')
# worksheet.write('B{}'.format(ctrl_hrzn+pred_hrzn+16),'Input')
# worksheet.write('C{}'.format(ctrl_hrzn+pred_hrzn+16),'Output')
# worksheet.write('D{}'.format(ctrl_hrzn+pred_hrzn+16),'Objective')
# worksheet.write('D{}'.format(ctrl_hrzn+pred_hrzn+17),obj_real)

# for i in range(ctrl_hrzn+pred_hrzn+17,ctrl_hrzn+pred_hrzn+17+ctrl_hrzn):
# 	worksheet.write('B{}'.format(i),sum(m.u[0,2,i-(ctrl_hrzn+pred_hrzn+17),0].value+dist[j]*m.u[0,1,i-(ctrl_hrzn+pred_hrzn+17),j].value for j in range(0,i-(ctrl_hrzn+pred_hrzn+16))))

# for i in range(ctrl_hrzn+pred_hrzn+17,ctrl_hrzn+pred_hrzn+17+pred_hrzn-1):
# 	worksheet.write('A{}'.format(i),dist[i-(ctrl_hrzn+pred_hrzn+17)])
# 	worksheet.write('C{}'.format(i),Y[i-(ctrl_hrzn+pred_hrzn+17)])
	

# chart = workbook.add_chart({'type':'scatter'})
# chart.add_series({'name':'=Sheet1!'+string.uppercase[pred_hrzn+3]+'10','categories':'=Sheet1!A11:A{}'.format(ctrl_hrzn+10),'values':'=Sheet1!'+string.uppercase[pred_hrzn+3]+'11:'+string.uppercase[pred_hrzn+3]+'{}'.format(ctrl_hrzn+10)})
# chart.add_series({'name':'=Sheet1!'+string.uppercase[pred_hrzn+3]+'{}'.format(ctrl_hrzn+13),'categories':'=Sheet1!A{}:A{}'.format(ctrl_hrzn+14,ctrl_hrzn+pred_hrzn+13),'values':'=Sheet1!'+string.uppercase[pred_hrzn+3]+'{}:'.format(ctrl_hrzn+14)+string.uppercase[pred_hrzn+3]+'{}'.format(ctrl_hrzn+pred_hrzn+13)})
# worksheet.insert_chart('Z1',chart)