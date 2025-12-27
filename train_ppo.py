"""
PPO Training Script for Ant Navigation
Run: python train_ppo.py
"""

import numpy as np
import torch
from stable_baselines3 import PPO
from stable_baselines3.common.env_util import make_vec_env
from stable_baselines3.common.callbacks import CheckpointCallback, EvalCallback
from stable_baselines3.common.monitor import Monitor
import matplotlib.pyplot as plt
import os

from gymnasium_env import AntNavigationEnv
from agent import Agent_Model
from environment import Environment
from config import AgentConfig, EnvironmentConfig


def make_env(seed=0):
    """
    Create a single environment instance.
    """
    def _init():
        agent_cfg = AgentConfig()
        env_cfg = EnvironmentConfig()
        world = Environment(env_cfg)
        world.build()
        agent = Agent_Model(stl_path="CAD_Model_ant/ant_model.stl", agent_cfg=agent_cfg)
        env = AntNavigationEnv(agent_instance=agent, env_instance=world, max_steps=1000)
        env = Monitor(env)  # Wrap with Monitor for logging
        env.reset(seed=seed)
        return env
    return _init


def plot_training_results(log_dir):
    """Plot training metrics from tensorboard logs."""
    try:
        from stable_baselines3.common.results_plotter import load_results, ts2xy
        
        results = load_results(log_dir)
        x, y = ts2xy(results, 'timesteps')
        
        fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(12, 4))
        
        # Plot episode rewards
        ax1.plot(x, y)
        ax1.set_xlabel('Timesteps')
        ax1.set_ylabel('Episode Reward')
        ax1.set_title('Training Progress')
        ax1.grid(True)
        
        # Plot success rate (moving average)
        window = 100
        if len(results['r']) >= window:
            success_rate = (results['r'] > 500).rolling(window=window).mean()
            ax2.plot(success_rate)
            ax2.set_xlabel('Episode')
            ax2.set_ylabel('Success Rate')
            ax2.set_title(f'Success Rate (Moving Avg, window={window})')
            ax2.grid(True)
        
        plt.tight_layout()
        plt.savefig(os.path.join(log_dir, 'training_results.png'))
        print(f"📊 Training plot saved to {log_dir}/training_results.png")
        plt.show()
    except Exception as e:
        print(f"Could not plot results: {e}")


def evaluate_policy(model, n_eval_episodes=20):
    """Evaluate the trained policy."""
    print("\n" + "="*60)
    print("EVALUATING TRAINED POLICY")
    print("="*60)
    
    agent_cfg = AgentConfig()
    env_cfg = EnvironmentConfig()
    world = Environment(env_cfg)
    world.build()
    agent = Agent_Model(stl_path="CAD_Model_ant/ant_model.stl", agent_cfg=agent_cfg)
    eval_env = AntNavigationEnv(agent_instance=agent, env_instance=world, max_steps=1000)
    
    successes = 0
    total_rewards = []
    episode_lengths = []
    
    for episode in range(n_eval_episodes):
        obs, info = eval_env.reset()
        done = False
        episode_reward = 0
        steps = 0
        
        while not done:
            action, _ = model.predict(obs, deterministic=True)
            obs, reward, terminated, truncated, info = eval_env.step(action)
            episode_reward += reward
            steps += 1
            done = terminated or truncated
        
        if info['is_success']:
            successes += 1
        
        total_rewards.append(episode_reward)
        episode_lengths.append(steps)
        
        print(f"Episode {episode+1:2d}: Reward={episode_reward:7.2f}, "
              f"Steps={steps:3d}, Success={info['is_success']}")
    
    print("\n" + "="*60)
    print("EVALUATION SUMMARY")
    print("="*60)
    print(f"Success Rate:     {100*successes/n_eval_episodes:.1f}%")
    print(f"Average Reward:   {np.mean(total_rewards):.2f} ± {np.std(total_rewards):.2f}")
    print(f"Average Steps:    {np.mean(episode_lengths):.1f} ± {np.std(episode_lengths):.1f}")
    print("="*60)


def main():
    # ========== CONFIGURATION ==========
    TOTAL_TIMESTEPS = 500_000      # Total training steps
    N_ENVS = 4                      # Number of parallel environments
    SAVE_FREQ = 50_000             # Save model every N steps
    EVAL_FREQ = 25_000             # Evaluate every N steps
    N_EVAL_EPISODES = 10           # Episodes per evaluation
    
    # Create directories
    log_dir = "./ppo_ant_logs/"
    models_dir = "./ppo_ant_models/"
    os.makedirs(log_dir, exist_ok=True)
    os.makedirs(models_dir, exist_ok=True)
    
    print("="*60)
    print("🐜 ANT NAVIGATION - PPO TRAINING")
    print("="*60)
    print(f"Total timesteps:  {TOTAL_TIMESTEPS:,}")
    print(f"Parallel envs:    {N_ENVS}")
    print(f"Device:           {torch.cuda.get_device_name(0) if torch.cuda.is_available() else 'CPU'}")
    print(f"Log directory:    {log_dir}")
    print(f"Models directory: {models_dir}")
    print("="*60 + "\n")
    
    # ========== CREATE ENVIRONMENTS ==========
    print("🔧 Creating environments...")
    
    # Create training environments with different seeds
    env = make_vec_env(
        lambda: make_env(seed=42)(),
        n_envs=N_ENVS,
        vec_env_cls=None  # Use default (DummyVecEnv)
    )
    
    # Create evaluation environment
    eval_env = make_vec_env(
        lambda: make_env(seed=123)(),
        n_envs=1
    )
    
    # ========== SETUP CALLBACKS ==========
    # Save checkpoints
    checkpoint_callback = CheckpointCallback(
        save_freq=SAVE_FREQ // N_ENVS,
        save_path=models_dir,
        name_prefix="ppo_ant"
    )
    
    # Evaluate periodically
    eval_callback = EvalCallback(
        eval_env,
        best_model_save_path=models_dir,
        log_path=log_dir,
        eval_freq=EVAL_FREQ // N_ENVS,
        n_eval_episodes=N_EVAL_EPISODES,
        deterministic=True,
        render=False
    )
    
    # ========== CREATE PPO MODEL ==========
    print("🤖 Initializing PPO model...")
    model = PPO(
        "MlpPolicy",
        env,
        learning_rate=3e-4,
        n_steps=2048 // N_ENVS,      # Steps per environment per update
        batch_size=64,
        n_epochs=10,
        gamma=0.99,                   # Discount factor
        gae_lambda=0.95,
        clip_range=0.2,
        ent_coef=0.01,                # Entropy coefficient (exploration)
        vf_coef=0.5,
        max_grad_norm=0.5,
        verbose=1,
        tensorboard_log=log_dir,
        device="auto"
    )
    
    print(f"Policy network: {model.policy}\n")
    
    # ========== TRAIN ==========
    print("🚀 Starting training...\n")
    try:
        model.learn(
            total_timesteps=TOTAL_TIMESTEPS,
            callback=[checkpoint_callback, eval_callback],
            progress_bar=True
        )
        print("\n✅ Training completed!")
        
    except KeyboardInterrupt:
        print("\n⚠️ Training interrupted by user")
    
    # ========== SAVE FINAL MODEL ==========
    final_model_path = os.path.join(models_dir, "ppo_ant_final")
    model.save(final_model_path)
    print(f"💾 Final model saved to: {final_model_path}")
    
    # ========== EVALUATE ==========
    evaluate_policy(model, n_eval_episodes=20)
    
    # ========== PLOT RESULTS ==========
    plot_training_results(log_dir)
    
    print("\n🎉 All done! Check the logs and models directories.")
    print(f"   - Logs: {log_dir}")
    print(f"   - Models: {models_dir}")
    print("\nTo visualize training in real-time, run:")
    print(f"   tensorboard --logdir {log_dir}")


if __name__ == "__main__":
    main()