import numpy as np
from gymnasium_env import AntNavigationEnv
from agent import Agent_Model
from environment import Environment
from config import AgentConfig, EnvironmentConfig

def test_random_policy():
    # 1. Initialize configuration and world
    agent_cfg = AgentConfig()
    env_cfg = EnvironmentConfig()
    world = Environment(env_cfg)
    world.build()
    
    # 2. Initialize agent (headless mode)
    agent = Agent_Model(stl_path="CAD_Model_ant/ant_model.stl", agent_cfg=agent_cfg)
    
    # 3. Create Gymnasium environment
    # Pass fsm_instance=None if you don't have FSM in RL mode
    env = AntNavigationEnv(agent_instance=agent, env_instance=world)
    
    # 4. Run episodes
    for episode in range(10):
        print(f"\n{'='*50}")
        print(f"Episode {episode+1}")
        print(f"{'='*50}")
        
        obs, info = env.reset()
        done = False
        step_count = 0
        total_reward = 0
        
        while not done:
            # Sample random action
            action = env.action_space.sample()
            
            # Execute step
            obs, reward, terminated, truncated, info = env.step(action)
            total_reward += reward
            
            # Print progress every 10 steps
            if step_count % 10 == 0:
                print(f"Step {step_count:3d} | "
                      f"Dist: {obs[0]:6.2f} | "
                      f"Bearing: {np.degrees(obs[1]):6.1f}° | "
                      f"Uncertainty: {obs[2]:6.2f} | "
                      f"Reward: {reward:7.3f}")
            
            step_count += 1
            done = terminated or truncated or step_count > 500
        
        print(f"\nEpisode finished:")
        print(f"  Steps: {step_count}")
        print(f"  Total reward: {total_reward:.2f}")
        print(f"  Success: {info['is_success']}")
        print(f"  Final distance (true): {info['true_dist']:.2f}")
        print(f"  Final distance (belief): {info['belief_dist']:.2f}")
    
    print("\n[SUCCESS] Environment verification complete!")
    env.close()

if __name__ == "__main__":
    test_random_policy()