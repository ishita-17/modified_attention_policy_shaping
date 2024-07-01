import numpy as np
import robot
import task
import torch
import torch.nn.functional as F
import math

np.random.seed(0)

class Environment:
    
    def __init__(self, arm, task):
        self.arm = arm
        self.task = task
        self.states = self.task.get_states()
        self.actions_length = self.task.get_actions_length()
        
    # returns num_steps, stop, episode_reward
    def step(self, num_steps, policy, episode_reward, episode_num):
        num_steps += 1
        current_state = self.task.get_current_state()
        print("Current state: ", current_state.to_string())
        new_state, action_index = policy.get_action(current_state, self.task, self.arm, episode_num)
        print("Action taken:", action_index)
        #print("New state: ", new_state.to_string())
        current_reward = new_state.get_reward()
        episode_reward += current_reward
        #print("Current reward: ", current_reward)
        if (episode_num > (0.5*policy.total_episodes)) or (episode_num < (0.9*policy.total_episodes)):
            policy.update_q_table(current_state, new_state, current_reward, action_index)
            policy.update_f_table(current_state, action_index)
        stop = False
        if new_state in self.task.terminal_states:
            self.terminal_state_cb()
            stop = True
        self.task.set_current_state(new_state)
        return num_steps, stop, episode_reward

    def terminal_state_cb(self):
        self.arm.open_gripper()
        self.arm.home_arm()
        pass

    def reset(self):
        self.current_state = self.task.reset(self.arm)

class QTablePolicy:

    def __init__(self, states, actions_length, total_episodes = 20, steps_per_episode = 10, exploration_rate = 0.5, exploration_decay = 0.01, discount_factor = 0.9, learning_rate = 0.1):
        self.q_table = {}
        self.feedback = {}
        self.old_reward = 0
        self.total_episodes = total_episodes
        self.steps_per_episode = steps_per_episode
        self.exploration_rate = exploration_rate
        self.exploration_decay = exploration_decay
        self.discount_factor = discount_factor
        self.learning_rate = learning_rate
        self.end_episode = False
        self.states = states
        for state in states.values():
            self.q_table[state] = np.zeros(actions_length)
            self.feedback[state] = np.zeros(actions_length)

    def learn_task(self, env):
        for episode in range(self.total_episodes):
            print("Episode: ", episode+1)   
            episode_reward = 0
            num_steps = 0
            env.reset()
            for step in range(self.steps_per_episode):
                num_steps, stop, episode_reward = env.step(num_steps, self, episode_reward, episode)
                if stop:
                    print("BREAK")
                    break
            print("Episode reward: ", episode_reward)
            print("Num steps: ", num_steps)
            print("===================================")
        print("||||||||||||||||||||||||||||||||||")
        self.exploration_rate *= (1 - self.exploration_decay)
        print("Q table")
        for i in range(len(self.states)):
            if (list(self.states.values())[i] in self.q_table.keys()):
                print("State ", str(i),": ", self.q_table[list(self.states.values())[i]])
        print("Feedback table")
        for i in range(len(self.states)):
            if (list(self.states.values())[i] in self.q_table.keys()):
                print("State ", str(i),": ", self.feedback[list(self.states.values())[i]])
    
    
    def get_action(self, state, task, arm, episode_num):
        # returns the state and action index
        check_explore = np.random.uniform(0,1)
        # assumes that possible_actions is a list of action indices corresponding to the 
        # actions list in task
        possible_actions = state.get_action_indices()
        if (check_explore < self.exploration_rate):
            q_prob = np.array([1/len(possible_actions) for i in range(len(possible_actions))])
            #print("Possible actions", possible_actions)
        else:
            q_prob = F.softmax(torch.tensor([self.q_table[state][action]/self.exploration_rate for action in possible_actions]),dim = 0).detach().numpy()
            # action_index = np.random.choice(np.flatnonzero(q_prob == q_prob.max()))
        p_prob = self.get_p_action(state, possible_actions)
        #print(q_prob)
        prob = q_prob * p_prob
        print("Old probability: ", prob)
        if (episode_num <= (0.5*self.total_episodes)) or (episode_num >= (0.9*self.total_episodes)):
            print("Getting attention now.")
            random_number = np.random.choice([0, 1], p=[0.5, 0.5])
            #print("Random number ", random_number)
            for i in range(len(possible_actions)):
                if (len(state.get_actions_seen()) != 0) and (possible_actions[i] in state.get_actions_seen()) and (random_number == 0):
                    prob[i] = 0
                elif (len(state.get_actions_good()) != 0)  and (possible_actions[i] not in state.get_actions_good()) and (random_number == 1):
                    prob[i] = 0
        else:
            print("Checking for good actions without supervision.")
            similarity_index = self.state_similarity_check(state)
            org_prob = prob[similarity_index]
            for i in range(len(possible_actions)):
                if (len(state.get_actions_good()) != 0)  and (possible_actions[i] not in state.get_actions_good()):
                   prob[i]= 0
            if similarity_index != -1:
                print("Found a good action using similarity metric.")
                #prob[similarity_index] = org_prob * 1.5      # increase by 50%
        print("New Probability", prob)
        action_index = possible_actions[np.random.choice(np.flatnonzero(prob == prob.max()))]
        print("Action index: ", action_index)
        new_state = task.take_action(self.states, action_index, arm)
        task.set_current_state(new_state)
        # print("New state: ", str(new_state))
        return new_state, action_index
    
    def state_similarity_check(self, state):
        possible_actions = state.action_indices
        for i in self.feedback.values():
            for j in range(len(possible_actions)):
                if i[possible_actions[j]] > 0: 
                    return j
        return -1

    def get_p_action(self, state, possible_actions):
        prob = []
        trust = 0.9
        if all(value == 0 for value in self.feedback[state]):
            return np.array([1/len(possible_actions) for i in range(len(possible_actions))])
        for i in range(len(possible_actions)):
            prob.append(math.pow(trust,self.feedback[state][i])/
                        (math.pow(trust,self.feedback[state][i]) + 
                         math.pow(trust,self.feedback[state][i])) )
        prob = np.array(prob)
        return prob
    
    def update_q_table(self, state, new_state, reward, action_index):
        if new_state not in self.q_table.keys(): 
            self.q_table[new_state] = np.zeros(len(self.q_table[0]))
        self.old_reward = self.q_table[state][action_index]
        self.q_table[state][action_index] = self.q_table[state][action_index] * \
            (1 - self.learning_rate) + self.learning_rate * (reward + self.discount_factor * \
            np.max(self.q_table[new_state]))


    def update_f_table(self, state, action_index): 
        if action_index in state.get_actions_good():
            self.feedback[state][action_index] += 1
        if action_index not in state.get_actions_seen():
            state.set_actions_seen(action_index)
        else:
            #print(self.q_table[state][action_index])
            #print(np.max(self.q_table[state]))
            if self.q_table[state][action_index] > self.old_reward:
                #n = int(input("Enter feedback: "))
                self.feedback[state][action_index] += 1
                state.set_actions_good(action_index)
            else:
                self.feedback[state][action_index] -= 1
            
        

if __name__=="__main__":
    #task = task.Task_Stack()
    task = task.Task_Move(3)
    env = Environment(robot.Robot(), task)
    current_policy = QTablePolicy(env.states, env.actions_length)
    current_policy.learn_task(env)
    
    
        

