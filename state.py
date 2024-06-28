
class State:
    def __init__(self, actions = [], reward = 0, *args, **kwargs):
        self.desc = args if args else kwargs
        # contains the indices of possible actions
        #self.description = location
        self.action_indices = actions
        self.actions_seen = []
        self.actions_good = []
        self.reward = reward

    def get_reward(self):
        return self.reward
    
    def get_action_indices(self):
        return self.action_indices
    
    def to_string(self):
        return str(self.desc) 
    
    def get_actions_seen(self):
        return self.actions_seen
    
    def get_actions_good(self):
        return self.actions_good
    
    def set_actions_seen(self, action_index):
        self.actions_seen.append(action_index)
        
    def set_actions_good(self, action_index):
        self.actions_good.append(action_index)
