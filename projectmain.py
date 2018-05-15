import gym
import time
from multiprocessing import Process, Pipe
import numpy as np
from gym.envs.registration import registry, register, make, spec


 
class EnvWorker(Process):
 
    def __init__(self, env_name, pipe, name=None,NotDisplay=False,delay=0):
        Process.__init__(self, name=name)
        self.env = gym.make(env_name)
        #self.env.tags['wrapper_config.TimeLimit.max_episode_steps'] = 500
        self.pipe = pipe
        self.name = name
        self.Display= not NotDisplay
        self.delay = delay
        print ("Environment initialized ->", self.name)
 
    def run(self):
        observation=self.env.reset()
        param=self.pipe.recv()
        episode_return=0
        time_steps=0
        trials=0
        while True:
            time.sleep(self.delay)
            decision=np.matmul(observation,param)
            action=1 if decision>0 else 0
            observation, reward, done, _ = self.env.step(action)
            episode_return+=reward
            time_steps=time_steps+1
            self.env.render(close=self.Display)
            if done:
                trials=trials+1
                #print("No of trials:",trials)
                #print(" time steps:",time_steps)
                self.pipe.send(episode_return)
                episode_return=0
                param=self.pipe.recv()
                if param=="  IT":
                    break
                self.env.reset()
 
class ParallelEnvironment(object):
 
    def __init__(self, env_name, num_env,NotDisplay):
        assert num_env > 0, "Number of environments must be postive."
        self.num_env = num_env
        self.workers = []
        self.pipes = []
        for env_idx in range(num_env):
            p_start, p_end = Pipe()
            env_worker = EnvWorker(env_name, p_end, name="Worker"+str(env_idx),NotDisplay=NotDisplay)
            env_worker.start()
            self.workers.append(env_worker)
            self.pipes.append(p_start)
 
    def episode(self, params):
        returns = []
        for idx in range(self.num_env):
            self.pipes[idx].send(params[idx])
        for idx in range(self.num_env):
            return_ = self.pipes[idx].recv()
            returns.append(return_)
        return returns
 
    def __del__(self):
        for idx in range(self.num_env):
            self.pipes[idx].send("EXIT")
 
        for worker in self.workers:
            worker.terminate()
temp=[]
j=0
while j<=19:     
    num_envs = 7
    NotDisplay=False
    velocity_alpha=0.5
    No_of_iter=100
    env_name="CartPole-v0"
    p_env = ParallelEnvironment(env_name, num_envs,NotDisplay)
 
    params=[]
    best_params=[]
    best_returns=[]
    best_pos=0
    velocity=[]
    iter_cnt=0
    #iter_cnt[0]=0
    inertia=1
    cognitive=1
    social=1
 
    for i in range(num_envs):
        params.append(np.random.uniform(-1,1,4))
        best_returns.append(0)
        best_params.append(np.array([0 for i in range(4)]))
        velocity.append(np.array([0.0 for i in range(4)]))
    while True:
        returns=p_env.episode(params)
        iter_cnt+=1
        if iter_cnt>=No_of_iter:
             print("Run failed to reach, we will re-run")
             #j=j-1
             #if j==9:
                #print("Average number of trials")
                #print(np.sum(temp)/(j+1))
             break
        print ("Number of batch episodes ran -> ",iter_cnt)
        #print ("Parameter for the batch for last episode ->")
        #print (np.around(params,3))
        #print ("Best Parameters for the batch for all episodes ->")
        #print (np.around(best_params,3))
        print ("Returns for the batch for last episode -> ",returns)
        print ("Returns for the batch for all episodes -> ",best_returns)
        #print ("Rate of change of parameters for the batch -> ")
        #print (np.around(velocity,3))
        #time_steps=0
 
        if returns==best_returns:
            print ("Batch converged after {} iterations.".format(iter_cnt))
            p_env.__del__()
            p_start, p_end = Pipe()
            env_worker = EnvWorker("CartPole-v0", p_end, name="Worker",NotDisplay=True,delay=0.05)
            print("No. of trails for run ",j+1,"is",iter_cnt)
            j=j+1
            temp.append(iter_cnt)
            if j==20:
                print("No. of trails for run ",j,"is",iter_cnt)
                temp.append(iter_cnt)
                print("Average number of trials")
                print(np.sum(temp)/(j))
                env_worker.start()
                p_start.send(np.sum(best_params,axis=0)/num_envs)
                episode_return = p_start.recv()
                print ("Reward for the final episode ->", episode_return)
                p_start.send("EXIT")
                env_worker.terminate()
                break
            else:
                break

       
        for i in range(num_envs):
            if(returns[i]>=best_returns[i]):
                best_returns[i]=returns[i]
                best_params[i]=params[i]
   
        best_pos=returns.index(max(returns))
 
        for i in range(num_envs):
            velocity[i]=(inertia*velocity[i]
            +cognitive*np.random.uniform(0,velocity_alpha)*(best_params[i]-params[i])
            +social*np.random.uniform(0,velocity_alpha)*(best_params[best_pos]-params[i]))
            params[i]+=velocity[i]
        pass
