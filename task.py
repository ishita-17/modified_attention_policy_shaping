from abc import ABC, abstractmethod
import action
import state

class Task(ABC):

    def __init__(self):
        self.current_state = None 
        self.states = None
        self.initial_state = None
        self.actions = []

    def get_current_state(self):
        return self.current_state
    
    def set_current_state(self, new_state):
        self.current_state = new_state
    
    def get_current_reward(self):
        return self.current_state.get_reward()
    
    def get_states(self):
        return self.states
    
    def get_actions_length(self):
        return len(self.actions)
    
    @abstractmethod
    def take_action(self, states, action_index, arm):
        pass

    def reset(self, arm):
        self.current_state = self.initial_state
        return self.current_state
        

class Task_Stack(Task):
    def __init__(self):
        # CHANGE THIS TO THE SPECIFIC ACTION
        self.actions = action.TaskStackActions.ALL
        self.states = {((0, 0, 0), (0, 0, 0)) : state.State(actions = [0, 1], reward = -1, location=((0, 0, 0), (0, 0, 0))),
                   ((1, 0, 0), (0, 0, 0)) : state.State(actions = [2, 4], reward = -1, location=((0, 0, 0), (0, 0, 0))),
                   ((2, 0, 0), (0, 0, 0)) : state.State(actions =[], reward = 10, location=((0, 0, 0), (0, 0, 0))), 
                   ((3, 0, 0), (0, 0, 0)) : state.State(actions = [3, 4], reward = -1, location=((0, 0, 0), (0, 0, 0))),
                   ((4, 0, 0), (0, 0, 0)) : state.State(actions = [], reward = 10, location=((0, 0, 0), (0, 0, 0)))}
        self.terminal_states = self.states[((2, 0, 0), (0, 0, 0))], self.states[((4, 0, 0), (0, 0, 0))]
        self.initial_state = self.current_state = self.states[((0, 0, 0), (0, 0, 0))]


    def take_action(self, states, action_index, arm):
        action = self.actions[action_index]
        print("Action in task: ", action_index)
        return action.execute(states, arm)

    def reset(self, arm):
        super().reset(arm)
        

    
class Task_Move(Task):
    def __init__(self, grid_dim):
        self.actions = action.TaskMoveActions.ALL
        print("Task Actions List: ",self.actions)
        self.grid_dim = grid_dim # can change for any grid dimension
        self.states = {}
        for i in range(grid_dim):
            for j in range(grid_dim):
                possible_actions = [0, 1, 2, 3]
                if (i == 0):
                    # we can't move up
                    possible_actions.remove(0)
                if (i == grid_dim - 1):
                    # we can't move down
                    possible_actions.remove(1)
                if (j == 0):
                    # we can't move left
                    possible_actions.remove(2)
                if (j == grid_dim - 1):
                    # we can't move right
                    possible_actions.remove(3)
                self.states[(i, j)] = state.State(possible_actions, -1, i, j)
        self.states[(grid_dim - 1, grid_dim - 1)].reward = 10
        self.distance_delta = 0.1
        self.terminal_states = [self.states[(grid_dim - 1, grid_dim - 1)]]
        self.x_start = 0.06
        self.y_start = -0.27
        self.z_start = -0.44
        self.initial_state = self.current_state = self.states[(0,0)]
        #self.arm = arm

    def take_action(self, states, action_index, arm):
        action = self.actions[action_index]
        print("Action in task: ", action)
        return action.execute(states, self.current_state, self.distance_delta, arm)

    def reset(self, arm):
        arm.home_arm()
        self.x_start = 0.06
        self.y_start = -0.27
        self.z_start = -0.44
        arm.goto_cartesian_pose(self.x_start, self.y_start, self.z_start)
        arm.close_gripper()
        super().reset(arm)
        
'''
class Task_Sort(Task):

'''