from enum import Enum

class AgentState(Enum):
    IDLE = 0
    SEARCH = 1
    FOUND_FOOD = 2
    RETURN = 3
    STOP = 4


class FSMController:
    def __init__(self, agent,search_policy,homing_policy,logger=None):
        """
        agent: Agent_Model instance
        navigation_policy: e.g. CorrelatedRandomWalk
        """
        self.agent = agent
        self.search_policy = search_policy
        self.homing_policy = homing_policy
        self.nav = None
        self.logger = logger
        self.state = AgentState.IDLE


    def log(self, message):
        """Helper to send logs to the UI if available, else print to terminal."""
        if self.logger:
            self.logger(message)  # Call the UI function
        else:
            print(f"[FSM] {message}")  # Fallback for debugging

    def set_state(self, new_state):
        if new_state == self.state:
            return

        self.log(f"FSM Switching: {self.state.name} -> {new_state.name}")

        self.state = new_state

        if self.state == AgentState.SEARCH:
            self.nav = self.search_policy
            self.search_policy.reset()

        elif self.state == AgentState.RETURN:
            self.nav = self.homing_policy
            self.homing_policy.reset()

        elif self.state == AgentState.STOP:
            self.nav = None

        

    def update(self):
        
        if self.state in (AgentState.IDLE, AgentState.STOP):
            return

        elif self.state == AgentState.SEARCH:
            dx, dy = self.search_policy.step()
            self.agent.move(dx, dy)

        elif self.state == AgentState.RETURN:
            sim_pos = self.agent.get_sim_pos()[:2]
            dx, dy, arrived = self.homing_policy.step(sim_pos)
            self.agent.move(dx, dy)

            if arrived:
                self.log("Agent believes it is home.")
                self.set_state(AgentState.STOP)

        elif self.state == AgentState.STOP:
            return

