#!/usr/bin/env python

import rospy, rosbag, rosparam
import math, sys, random, datetime
import numpy as np
from geometry_msgs.msg import Twist
from std_srvs.srv import Trigger, TriggerResponse
from raspimouse_ros.msg import LightSensorValues
from sklearn.decomposition import PCA
from sklearn.preprocessing import StandardScaler
from numpy import linalg as la
from scipy.special import digamma
from scipy.stats import multivariate_normal
import math
import random


class Logger():
    def __init__(self):
        self.sensor_values = LightSensorValues()
        print("start")
        #self._decision = rospy.Publisher('/cluster',LightSensorValues,queue_size=100)
        rospy.Subscriber('/event', LightSensorValues, self.sensor_callback)
        
        self.k = 8
        self.reset()
        self.g_sigma = [[[0.1,0],[0,0.1]],
                        [[0.1,0],[0,0.1]],
                        [[0.1,0],[0,0.1]],
                        [[0.1,0],[0,0.1]],
                        [[0.1,0],[0,0.1]],
                        [[0.1,0],[0,0.1]],
                        [[0.1,0],[0,0.1]],
                        [[0.1,0],[0,0.1]]]
        self.pai = [1.0/self.k for i in range(self.k)]

	self.on = True
	self.bag_open = False
        self.output_decision()
    
    def sensor_callback(self,messages):
        self.sensor_values = messages

    def reset(self):
        self.label = []
        self.count = np.zeros((self.k,1))
        self.sum = np.zeros((self.k,2))
        self.t = np.zeros((self.k,1))

    def cluster_center(self,samples):
        clus_center = []
        for i in range(self.k):
            clus_center.append(random.choice(samples))
            #clus_center.append([random.uniform(min(samples[:,0])+1.5,max(samples[:,0])-1.5),random.uniform(min(samples[:,1])+1.5,max(samples[:,1])-1.5)])
        self.mu = np.array(clus_center)

    def EM_step(self,sample,mu):
        self.reset()
        
        for n in range(self.N):
            distance = []
            distance = [np.linalg.norm(self.mu[i] - sample[n]) for i in range(self.k)]
            
            self.label.append(distance.index(min(distance)))
            self.count[distance.index(min(distance))] += 1
            self.sum[distance.index(min(distance))] += sample[n]
        
        for i in range(0,self.k):
            self.mu[i] = self.sum[i]/self.count[i]
        
    def make_gauss_model(self,mu,sigma):
        gaussian = []
        for i in range(self.k):
            gaussian.append(multivariate_normal(mean=mu[i],cov=sigma[i]))
        return gaussian
    
    def ganma_init(self,samples,gauss):
        ganma = []
        for i in range(self.N):
            sum_p = 0.0
            p=[]
            for j in range(self.k):
                sum_p += self.pai[j]*gauss[j].pdf([samples[i][0],samples[i][1]])
            for j in range(self.k):
                p.append(self.pai[j]*gauss[j].pdf([samples[i][0],samples[i][1]])/sum_p)
            ganma.append(p)
        return ganma

    def cal(self,ganma,samples):
        mu = np.zeros((self.k,2))
        S = np.zeros((self.k,2,2))
        N_k = []
    
        for k in range(self.k):
            sum_r = 0.0
            for n in range(self.N):
                sum_r += ganma[n][k]
            N_k.append(sum_r)
        
        for k in range(self.k):
            sum_r = 0.0
            for n in range(self.N):
                sum_r += ganma[n][k]*samples[n]
            mu[k] = sum_r/N_k[k]
            sigma = []
            sigma = [samples[j]-mu[k] for j in range(self.N)]
            tmp = np.zeros((1,2,2))
            for n in range(self.N):
                tmp += (ganma[n][k]*sigma[n])*sigma[n][:, np.newaxis]
            tmp2 = tmp/N_k[k]
            S[k] = tmp2
        return N_k,mu,S
    
    def M_step(self,N_k,mu,S):
        alpha_0 = 0.01
        beta_0 = 0.01
        nu_0 = 1.0
        m_0 = np.zeros((self.k,2))
        m = np.zeros((self.k,2))
        W_0 = np.identity(2)
        alpha=[]
        beta=[]
        nu=[]
        W = np.zeros((self.k,2,2))
        for k in range(self.k):
            alpha.append(alpha_0 + N_k[k])
            beta.append(beta_0 + N_k[k])
            nu.append(nu_0 + N_k[k])
            m[k] = (beta_0*m_0[k] + N_k[k]*mu[k])/beta[k]
        sigma = []
        sigma = [mu[j]-m_0[j] for j in range(self.k)]
        for k in range(self.k):
            tmp = beta_0*N_k[k]*sigma[k]*sigma[k][:, np.newaxis]/(beta_0 + N_k[k])
            tmp2 = la.inv(W_0)+N_k[k]*S[k]+tmp
            W[k] = la.inv(tmp2)
        return alpha,beta,nu,m,W,alpha_0,beta_0,nu_0,m_0,W_0
    
    def E_step(self,alpha,beta,nu,m,W,samples):
        E_ln_A = []
        E_ln_pi = []
        E_mu_A = []
        r = []
        for k in range(self.k):
            tmp = sum([digamma((nu[k]+1-i)/2)for i in range(1,2+1)])
            E = tmp +2*math.log(2)+math.log(la.norm(W[k]))
            E_ln_A.append(E)
        
            E =  digamma(alpha[k])-digamma(sum(alpha))
            E_ln_pi.append(E)
        for n in range(self.N):
            tmp = [(2/beta[k])+nu[k]*np.dot((samples[n] - m[k]),np.dot(W[k],(samples[n] - m[k]).T)) for k in range(self.k)]
            E_mu_A.append(tmp)
        for n in range(self.N):
            tmp = [np.exp(np.array(E_ln_pi[k]) + np.array(E_ln_A[k])/2 - 2*math.log(2*math.pi)/2-np.array(E_mu_A[n][k])/2) for k in range(self.k)]
            for k in range(self.k):
                if tmp[k] < 1e-10:
                    tmp[k] = 1e-10
            tmp2 = np.array(tmp)/sum(tmp)
            
            r.append(tmp2)
        return E_ln_A,E_ln_pi,E_mu_A,r

    def convergence(self,pai2,n):
        z = np.array(pai2[n-1])-np.array(pai2[n])
        for i in range(self.k):
            if math.fabs(z[i])<0.001:
                self.fin[i] = 1 
        return sum(self.fin)


    def output_decision(self):
	if not self.on:
	    if self.bag_open:
                print("---------------------")
                print("Stop")
		self.bag.close()
		self.bag_open = False
	    return
	else:
	    if not self.bag_open:
                #print("---------------------")
                #print("Start")

                try:
		    #filename = datetime.datetime.today().strftime("sensor"
                    filename = "sensor_values.bag"
                    #self.bagfile_path = "~/catkin_ws/src/variational_bayes_gaussian_mixture_models/scripts/"+ filename
                    self.bag = rosbag.Bag(filename)
		    #rosparam.set_param("/current_bag_file", filename)
		    #self.bag = rosbag.Bag(filename, 'w')
                    print('--------------------------')
                    print("reading bagfile : %s" % filename)
                    self.bag_open = True
                    
                    self.right_forward = []
                    self.right_side = []
                    self.left_side = []
                    self.left_forward = []

                    for topic, msg, t in self.bag.read_messages(topics=['/event']):
                        #print("pan")
                        self.right_forward.append(float(msg.right_forward))
                        self.right_side.append(float(msg.right_side))
                        self.left_side.append(float(msg.left_side))
                        self.left_forward.append(float(msg.left_forward))
                        #print(float(msg.left_forward))
                    sensor_values = np.c_[self.left_forward, self.left_side, self.right_side, self.right_forward]
                    
                    ss = StandardScaler()
                    ss.fit(sensor_values)
                    sensor_ss = ss.transform(sensor_values)
                    pca = PCA(n_components=2)
                    self.samples = pca.fit_transform(sensor_ss)
                    
                    self.N = len(self.samples)
                    print("x:min")
                    print(min(self.samples[:,0]))
                    print("x:max")
                    print(max(self.samples[:,0]))
                    print("y:min")
                    print(min(self.samples[:,1]))
                    print("y:max")
                    print(max(self.samples[:,1]))
                    print("start clustering")
                except:
                    print('--------------')
                    print('Error!!!')

    def run(self):
        rate = rospy.Rate(10)
        data = Twist()
        count = 100
        pai2 = []
        self.fin = [0 for i in range(self.k)]
        samples = self.samples

        for j in range(200):
            self.cluster_center(samples)
            for i in range(10):
                self.EM_step(samples,self.mu)
            val = [math.isnan(self.mu[x][0]) for x in range(self.k)]
            print(val)
            check = True in val
            if check == False:
                print(j)
                break;
            #self.mu = []
        clus_center = self.mu
        print(j)
        print("ans:")
        print(clus_center)
        gauss = self.make_gauss_model(clus_center,self.g_sigma)
        ganma = self.ganma_init(samples,gauss)

        N_k,mu,S = self.cal(ganma,samples)
        gauss = self.make_gauss_model(mu,S)
        alpha,beta,nu,m,W,alpha_0,beta_0,nu_0,m_0,W_0 = self.M_step(N_k,mu,S)
        E_ln_A,E_ln_pi,E_mu_A,ganma = self.E_step(alpha,beta,nu,m,W,samples)

        for i in range(count):
            #N_k,mu,S = self.cal(ganma)

            alpha,beta,nu,m,W,alpha_0,beta_0,nu_0,m_0,W_0 = self.M_step(N_k,mu,S)
            E_ln_A,E_ln_pi,E_mu_A,ganma = self.E_step(alpha,beta,nu,m,W,samples)
            pai2.append(self.pai)
            if i > 1:
                juge = self.convergence(pai2,i)
                if juge == self.k:
                    print("finish!!!!")
                    num = 0
                    for j in range(self.k):
                        if self.pai[j] > 0.01:
                            num += 1
                    print(num)
                    fin_count = i
                    break
            self.pai = np.exp(E_ln_pi)
            N_k,mu,S = self.cal(ganma,samples)
            gauss = self.make_gauss_model(mu,S)
            print("count:{}".format(i))
            print(self.pai)

        while not rospy.is_shutdown():
            #count = 100
            #fin = [0 for i in range (self.k)]
            #for i in range(count):
                #alpha,beta,nu,m,W,alpha_0,beta_0,nu_0,m_0,W_0 = self.M_step(N_k,mu,S)
                #E_ln_A,E_ln_pi,E_mu_A,ganma = self.E_step(alpha,beta,nu,m,W,samples)
                #if i > 1:
                #juge = convergence(pai2,i)
                #if juge == K:
                    #print("finish!!!!")
                    #fin_count = i
                    #break
                #self.pai = np.exp(E_ln_pi)
                #N_k,mu,S = cal(ganma)
                #self.gauss = self.make_gauss_model(mu,S)
                #print("count:{}".format(i))
                #print(pai)
            #self.output_decision()
            rate.sleep()
            

if __name__ == '__main__':
    rospy.init_node('logger')
    Logger().run()
