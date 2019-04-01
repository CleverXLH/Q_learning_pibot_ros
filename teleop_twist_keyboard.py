#!/usr/bin/env python
import rospy
from driver import navigation_demo
import numpy as np
import pandas as pd
import random


np.random.seed(2)  # reproducible

N_STATES = 21   # the length of the 1 dimensional world
ACTIONS = ['up','down','left', 'right']     # available actions
EPSILON = 0.9   # greedy police
ALPHA = 0.1     # learning rate
GAMMA = 0.9    # discount factor
MAX_EPISODES = 100   # maximum episodes


def step(action,x,y):
        if action == 0:   # up
            if  x==0 : 
		x_ = x
		y_ = y
		r = 180
		reward = -1
	    else:
		x_ = x-0.5
		y_ = y
		r = 180
		reward = 0
        elif action == 1:   # down
            if  x==1 : 
		x_ = x
		y_ = y
		r = 0
		reward = -1
	    else:
		x_ = x+0.5
		y_ = y
		r = 0
		reward = 0
        elif action == 2:   # right
            if  y==3 : 
		x_ = x
		y_ = y
		r = 90
		reward = -1
	    else:
		x_ = x
		y_ = y+0.5
		r = 90
		reward = 0
        elif action == 3:   # left
            if  y==0 : 
		x_ = x
		y_ = y
		r = -90
		reward = -1
	    else:
		x_ = x
		y_ = y-0.5
		r = -90 
		reward = 0
	s_ = int( x/0.5 + 3*y/0.5)
	
	if (x_==1 and y_==3):
		reward = 1
		done = True
	else:
		
		done = False	
	return x_,y_,r,s_,reward,done



def build_q_table(n_states, actions):
    table = pd.DataFrame(
        np.zeros((n_states, len(actions))),     # q_table initial values
        columns=actions,    # actions's name
    )
    # print(table)    # show table
    return table


def choose_action(state, q_table):
    # This is how to choose an action
    state_action = q_table.iloc[state, :]
    if np.random.uniform() < EPSILON:
            # choose best action

            # some actions may have the same value, randomly choose on in these actions
            action_name = random.choice(state_action[state_action == np.max(state_action)].index)
    else:
            # choose random action
            action_name = random.choice(ACTIONS)
    return action_name

def Translate(A):
	if A == 'up':
		a = 0
	elif A == 'down':
		a = 1
	elif A == 'right':
		a = 2
	else:
		a = 3
	return a 

if __name__ == "__main__":
    	rospy.init_node('navigation_demo',anonymous=True)
    	navi = navigation_demo()
    	r = rospy.Rate(1)
    	r.sleep()	

    # main part of RL loop
    	q_table = build_q_table(N_STATES, ACTIONS)
	#print q_table
    	for episode in range(MAX_EPISODES):
        	navi.set_pose([0,0,0]) 
		navi.goto([0,0,0])        
		x,y = navi.get_pose()
		S =int( x/0.5 + 3*y/0.5)
        	is_terminated = False
        	while not is_terminated:

            		A = choose_action(S, q_table)
			print A
			a = Translate(A)
            		x_,y_,r,S_,R,flag = step(a,x,y)  # take action & get next state and reward
            		q_predict = q_table.loc[S, A]
            		if  (flag!=True):
                		q_target = R + GAMMA * q_table.iloc[S_, :].max()   # next state is not terminal
            		else:
                		q_target = R     # next state is terminal
                		is_terminated = True    # terminate this episode

            		q_table.loc[S, A] += ALPHA * (q_target - q_predict)  # update
            		S = S_  # move to next state
			x = x_
			y = y_
			navi.goto([x,y,r]) 
			
		print is_terminated
    		

