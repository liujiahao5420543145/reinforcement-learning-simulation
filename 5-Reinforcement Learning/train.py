from stable_baselines3 import PPO  # 导入近端策略优化算法 PPO，用于训练、加载模型及预测动作

from stable_baselines3.common.monitor import Monitor  # 导入监视包装器，统计回合累计奖励和回合长度等信息
from stable_baselines3.common.env_checker import check_env  # 导入环境检查工具，验证自定义环境符合 Gymnasium 接口

from car_env import CarEnv  # 从本工程的 car_env.py 导入自定义小车环境

from pathlib import Path  # 导入路径类，便于定位 XML 文件和创建输出目录


# =====================================
# 1. 创建文件夹
# =====================================

Path("models").mkdir(exist_ok=True)  # 在当前工作目录下创建模型目录，已存在时不报错
Path("logs").mkdir(exist_ok=True)  # 创建训练日志目录，已存在时不报错


# =====================================
# 2. 创建POMDP环境
# =====================================

env = CarEnv()  # 实例化小车环境，加载 XML 并初始化仿真状态

# 检查环境接口
check_env(env)  # 检查观测、动作及 reset/step 返回值是否符合接口要求

# 记录训练数据
env = Monitor(env)  # 用 Monitor 包装环境，供算法统计训练过程中的回合表现


# =====================================
# 3. 创建PPO模型
# =====================================

model = PPO(  # 创建 PPO 模型，并在后续行配置策略与训练超参数

    policy="MlpPolicy",  # 使用多层感知机策略；它根据当前观测决策，不包含循环记忆

    env=env,  # 将小车环境交给算法，用于采样交互数据

    learning_rate=0.0003,  # 设置神经网络优化器的学习率为 0.0003

    n_steps=2048,  # 每个环境采样 2048 个控制步后进行一轮策略更新

    batch_size=64,  # 将采样数据拆成大小为 64 的小批次进行梯度更新

    n_epochs=10,  # 每轮采样的数据重复训练 10 遍

    gamma=0.99,  # 设置奖励折扣因子，未来奖励每延后一步乘以 0.99

    verbose=1,  # 启用训练进度和统计信息输出

    tensorboard_log="./logs/"  # 将 TensorBoard 训练指标写入当前工作目录下的 logs 文件夹
)  # 结束当前跨行的函数调用或括号表达式


# =====================================
# 4. 开始训练
# =====================================

print("开始训练POMDP小车控制器")  # 向终端输出训练开始提示；观测含噪声，策略使用的是部分状态信息

model.learn(  # 开始与环境交互并优化策略网络
    total_timesteps=200000,  # 指定训练目标为 20 万步；按完整采样批次执行，实际步数可能略多
    tb_log_name="POMDP_Car"  # 指定本次 TensorBoard 运行的名称前缀
)  # 结束当前跨行的函数调用或括号表达式


# =====================================
# 5. 保存模型
# =====================================

model.save(  # 保存训练好的模型参数和算法配置
    "models/ppo_car"  # 指定保存路径，Stable-Baselines3 自动添加 .zip 扩展名
)  # 结束当前跨行的函数调用或括号表达式

print("训练完成，模型已保存")  # 向终端输出模型保存完成提示

env.close()  # 调用环境关闭接口，结束环境使用
