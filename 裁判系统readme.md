# 裁判系统说明

## 自动冒烟测试

编译后可使用独立 ROS 域和端口测试裁判，不影响正在运行的正式比赛：

```bash
cd ~/github/RM_simulation_test
./test_referee.sh
```

该脚本覆盖 11 个规则单元测试、错误 token 拒绝，以及 `start`、`pause`、`resume`、`reset` 网络命令。日常启动以 [快速启动.md](./快速启动.md) 为准。

## 历史开发记录

/home/wangxiaotao/文档/ChatGPT/仿真/RM_SELECTION_PROJECT_HANDOFF.md
请先完整读取 /home/wangxiaotao/文档/ChatGPT/仿真/RM_SELECTION_PROJECT_HANDOFF.md，
再继续 RM 新生选拔裁判系统项目。不要从头重新设计，先核对当前仓库状态和文档中的下一步。
更新今天的交接记录

环境初始化

cd ~/github/RM_simulation_test
set +u
source /opt/ros/humble/setup.bash
source /home/wangxiaotao/ws_livox/install/setup.bash 2>/dev/null || true
source install/setup.bash
裁判端启动端口
使用 run_rmul_match.sh 时已自动启动裁判端，无需重复执行本节。

cd ~/github/RM_simulation_test
set +u
source /opt/ros/humble/setup.bash
source /home/wangxiaotao/ws_livox/install/setup.bash 2>/dev/null || true
source install/setup.bash

ros2 launch rm_referee lan_referee_demo.launch.py \
  bind_host:=0.0.0.0 \
  port:=8765 \
  red_token:=red-test-2026 \
  blue_token:=blue-test-2026 \
  referee_token:=referee-test-2026 \
  state_broadcast_hz:=2.0
红方选手端连接
cd ~/github/RM_simulation_test
set +u
source /opt/ros/humble/setup.bash
source /home/wangxiaotao/ws_livox/install/setup.bash 2>/dev/null || true
source install/setup.bash

./install/rm_referee/lib/rm_referee/referee_lan_client \
  --host 127.0.0.1 \
  --port 8765 \
  --role red \
  --name red-player \
  --token red-test-2026 \
  --watch-seconds 300
蓝方选手端连接
cd ~/github/RM_simulation_test
set +u
source /opt/ros/humble/setup.bash
source /home/wangxiaotao/ws_livox/install/setup.bash 2>/dev/null || true
source install/setup.bash

./install/rm_referee/lib/rm_referee/referee_lan_client \
  --host 127.0.0.1 \
  --port 8765 \
  --role blue \
  --name blue-player \
  --token blue-test-2026 \
  --watch-seconds 300
连接交互式裁判 CLI
cd ~/github/RM_simulation_test
set +u
source /opt/ros/humble/setup.bash
source /home/wangxiaotao/ws_livox/install/setup.bash 2>/dev/null || true
source install/setup.bash

./install/rm_referee/lib/rm_referee/referee_lan_client \
  --host 127.0.0.1 \
  --port 8765 \
  --role referee \
  --name main-referee \
  --token referee-test-2026
裁判直接开始比赛
cd ~/github/RM_simulation_test
set +u
source /opt/ros/humble/setup.bash
source /home/wangxiaotao/ws_livox/install/setup.bash 2>/dev/null || true
source install/setup.bash

./install/rm_referee/lib/rm_referee/referee_lan_client \
  --host 127.0.0.1 \
  --port 8765 \
  --role referee \
  --name main-referee \
  --token referee-test-2026 \
  --command start \
  --watch-seconds 3
可将 start 替换为：
pause
resume
reset
下次主机启动
终端 1：启动 RMUL 仿真、导航和裁判服务
cd ~/github/RM_simulation_test
./run_rmul_match.sh
终端 2：连接裁判控制终端
cd ~/github/RM_simulation_test
set +u
source /opt/ros/humble/setup.bash
source /home/wangxiaotao/ws_livox/install/setup.bash 2>/dev/null || true
source install/setup.bash

./install/rm_referee/lib/rm_referee/referee_lan_client \
  --host 127.0.0.1 \
  --port 8765 \
  --role referee \
  --name main-referee \
  --token referee-test-2026
终端 3：连接红方选手端
cd ~/github/RM_simulation_test
set +u
source /opt/ros/humble/setup.bash
source /home/wangxiaotao/ws_livox/install/setup.bash 2>/dev/null || true
source install/setup.bash

./install/rm_referee/lib/rm_referee/referee_lan_client \
  --host 127.0.0.1 \
  --port 8765 \
  --role red \
  --name red-player \
  --token red-test-2026 \
  --watch-seconds 300
  
 ---------------------------------------------------------------------------
  可以用“独立 Linux 普通用户 + 独立工作区 + 独立开发 ROS_DOMAIN_ID”模拟选手 SSH 进入宿主机。
需要先说明：这种方案能隔离文件、sudo、Git 和开发仿真，但正式比赛时选手与裁判共用 ROS_DOMAIN_ID，理论上仍能绕过 teleop 直接发布裁判内部 ROS 话题。适合新生选拔和过程审计，不属于恶意对抗级安全。正式强隔离还需要 SROS2 权限或只暴露受控网络驾驶网关。
以下以红方选手 rm_red 为例。
一、主办方一次性创建隔离账号
1. 创建无 sudo 的选手账号
sudo adduser \
  --disabled-password \
  --gecos "RM red contestant" \
  rm_red

sudo deluser rm_red sudo 2>/dev/null || true
sudo deluser rm_red docker 2>/dev/null || true
sudo deluser rm_red lxd 2>/dev/null || true

id rm_red
输出中不应出现 sudo、docker 或 lxd。
2. 配置选手 SSH 公钥
先把选手发来的公钥保存为：
/tmp/rm_red_authorized_key
然后执行：
sudo install -d \
  -o rm_red \
  -g rm_red \
  -m 700 \
  /home/rm_red/.ssh

sudo install \
  -o rm_red \
  -g rm_red \
  -m 600 \
  /tmp/rm_red_authorized_key \
  /home/rm_red/.ssh/authorized_keys
3. 限制 SSH 能力
sudoedit /etc/ssh/sshd_config.d/rm-selection.conf
写入：
Match User rm_red
    PasswordAuthentication no
    PubkeyAuthentication yes
    AllowAgentForwarding no
    AllowTcpForwarding no
    PermitTunnel no
    PermitUserEnvironment no
    X11Forwarding no
检查配置并重载：
sudo sshd -t
sudo systemctl reload ssh
二、生成选手独立题目副本
当前项目还有未提交开发改动，因此模拟阶段先使用文件快照。正式选拔应从冻结后的 challenge commit 生成。
sudo install -d \
  -o rm_red \
  -g rm_red \
  -m 700 \
  /home/rm_red/pb_rm_simulation

sudo rsync -a \
  --exclude='.git' \
  --exclude='build/' \
  --exclude='install/' \
  --exclude='log/' \
  --exclude='teacher_tests/' \
  --exclude='hidden_tests/' \
  --exclude='golden/' \
  /home/wangxiaotao/github/pb_rm_simulation/ \
  /home/rm_red/pb_rm_simulation/

sudo chown -R rm_red:rm_red \
  /home/rm_red/pb_rm_simulation
为选手快照建立独立 Git 仓库：
sudo -u rm_red git -C /home/rm_red/pb_rm_simulation init \
  --initial-branch=challenge

sudo -u rm_red git -C /home/rm_red/pb_rm_simulation config \
  user.name "rm-red-contestant"

sudo -u rm_red git -C /home/rm_red/pb_rm_simulation config \
  user.email "rm-red@selection.local"

sudo -u rm_red git -C /home/rm_red/pb_rm_simulation add -A

sudo -u rm_red git -C /home/rm_red/pb_rm_simulation commit \
  -m "challenge baseline"
主办方的黄金版本、隐藏测试和评分器不要放在选手家目录内，并限制为：
sudo chmod 700 /home/wangxiaotao
执行前先确认这不会影响你需要共享给其他人的文件。
三、选手第一次 SSH 登录并编译
选手电脑执行：
ssh rm_red@你的宿主机IP
进入后：
cd ~/pb_rm_simulation

set +u
source /opt/ros/humble/setup.bash

export CMAKE_POLICY_VERSION_MINIMUM=3.5

colcon build \
  --symlink-install \
  --parallel-workers 4 \
  --executor parallel
编译后：
set +u
source /opt/ros/humble/setup.bash
source ~/pb_rm_simulation/install/setup.bash

ros2 pkg prefix rm_referee
ros2 pkg prefix rm_combat_gazebo
如果只修改裁判、URDF 或战斗插件，可定向编译：
cd ~/pb_rm_simulation

set +u
source /opt/ros/humble/setup.bash
source install/setup.bash

export CMAKE_POLICY_VERSION_MINIMUM=3.5

colcon build \
  --symlink-install \
  --packages-up-to rm_combat_gazebo rm_referee \
  --parallel-workers 4 \
  --executor parallel
四、选手自己的隔离开发仿真
选手调试 URDF、裁判或碰撞插件时，不接入正式比赛 ROS 域。
选手 SSH 终端执行：
cd ~/pb_rm_simulation

set +u
source /opt/ros/humble/setup.bash
source install/setup.bash

export ROS_DOMAIN_ID=142
export ROS_LOCALHOST_ONLY=1
export GAZEBO_MASTER_URI=http://127.0.0.1:11346

./run_rmul_duel.sh \
  gui:=false \
  lan_gateway:=false
另开一个 SSH 终端，使用相同环境：
ssh rm_red@你的宿主机IP
cd ~/pb_rm_simulation

set +u
source /opt/ros/humble/setup.bash
source install/setup.bash

export ROS_DOMAIN_ID=142
export ROS_LOCALHOST_ONLY=1
export GAZEBO_MASTER_URI=http://127.0.0.1:11346
启动裁判监视器：
./install/rm_referee/lib/rm_referee/combat_monitor \
  --red-robot red_robot \
  --blue-robot blue_robot
启动红车手动控制：
./install/rm_referee/lib/rm_referee/combat_teleop \
  --robot red_robot
运行自动验收：
./install/rm_combat_gazebo/lib/rm_combat_gazebo/duel_integration_probe.py
选手自己的开发域与正式比赛域不同，因此不会控制正式场地。
五、主办方启动正式比赛
主办方终端：
cd ~/github/RM_simulation_test

set +u
source /opt/ros/humble/setup.bash
source /home/wangxiaotao/ws_livox/install/setup.bash 2>/dev/null || true
source install/setup.bash

export ROS_DOMAIN_ID=42
export ROS_LOCALHOST_ONLY=1

./run_rmul_duel.sh
该终端保持运行。
六、主办方连接裁判 CLI
另开主办方终端：
cd ~/github/RM_simulation_test

set +u
source /opt/ros/humble/setup.bash
source /home/wangxiaotao/ws_livox/install/setup.bash 2>/dev/null || true
source install/setup.bash

export ROS_DOMAIN_ID=42
export ROS_LOCALHOST_ONLY=1

./install/rm_referee/lib/rm_referee/referee_lan_client \
  --host 127.0.0.1 \
  --port 8765 \
  --role referee \
  --name main-referee \
  --token referee-test-2026
先执行：
roster
status
选手准备好后执行：
start
比赛中可使用：
pause
resume
status
reset
七、选手进入正式比赛
选手 SSH 登录：
ssh rm_red@你的宿主机IP
加载正式比赛 ROS 域：
cd ~/pb_rm_simulation

set +u
source /opt/ros/humble/setup.bash
source install/setup.bash

export ROS_DOMAIN_ID=42
export ROS_LOCALHOST_ONLY=1
1. 连接红方裁判席位
终端一：
./install/rm_referee/lib/rm_referee/referee_lan_client \
  --host 127.0.0.1 \
  --port 8765 \
  --role red \
  --name red-player \
  --token red-test-2026 \
  --watch-seconds 7200
2. 启动红车手动控制
选手再开一个 SSH 终端，重新加载环境：
cd ~/pb_rm_simulation

set +u
source /opt/ros/humble/setup.bash
source install/setup.bash

export ROS_DOMAIN_ID=42
export ROS_LOCALHOST_ONLY=1
启动控制：
./install/rm_referee/lib/rm_referee/combat_teleop \
  --robot red_robot
按键：
W/S：前进、后退
A/D：左移、右移
Q/E：旋转
方向键：调整云台
F：射击
空格：停车
X：停车并退出
主办方在裁判 CLI 执行 start 前，选手无法移动或射击。
八、蓝方选手
创建账号时将 rm_red 换成 rm_blue。正式控制命令为：
./install/rm_referee/lib/rm_referee/referee_lan_client \
  --host 127.0.0.1 \
  --port 8765 \
  --role blue \
  --name blue-player \
  --token blue-test-2026 \
  --watch-seconds 7200
./install/rm_referee/lib/rm_referee/combat_teleop \
  --robot blue_robot
蓝方独立开发环境建议使用：
export ROS_DOMAIN_ID=143
export ROS_LOCALHOST_ONLY=1
export GAZEBO_MASTER_URI=http://127.0.0.1:11347
九、选手提交代码
选手完成后：
cd ~/pb_rm_simulation

git status --short
git add -A
git commit -m "contestant solution"

git rev-parse HEAD
git bundle create ~/rm_red_submission.bundle HEAD
sha256sum ~/rm_red_submission.bundle
选手把 commit ID 和 SHA256 交给主办方。
主办方取出提交：
sudo install -d \
  -o wangxiaotao \
  -g wangxiaotao \
  -m 700 \
  /home/wangxiaotao/rm_evaluation/red

sudo cp \
  /home/rm_red/rm_red_submission.bundle \
  /home/wangxiaotao/rm_evaluation/red/

sudo chown -R wangxiaotao:wangxiaotao \
  /home/wangxiaotao/rm_evaluation/red
在干净目录重新克隆：
cd /home/wangxiaotao/rm_evaluation/red

git clone \
  rm_red_submission.bundle \
  workspace

cd workspace
git rev-parse HEAD
git status --short
重新编译：
set +u
source /opt/ros/humble/setup.bash
source /home/wangxiaotao/ws_livox/install/setup.bash 2>/dev/null || true

export CMAKE_POLICY_VERSION_MINIMUM=3.5

colcon build \
  --symlink-install \
  --parallel-workers 4 \
  --executor parallel
然后由主办方挂载隐藏测试进行评分，不直接在选手正在编辑的目录里评分。
最关键的边界是：ROS_DOMAIN_ID=142/143 用于选手开发，ROS_DOMAIN_ID=42 只在正式比赛窗口使用。正式比赛阶段选手仍可看到整个 ROS graph，因此必须配合进程审计；如果要求选手从技术上绝对无法控制蓝车或伪造裁判事件，下一步需要给项目增加 SROS2 权限策略或受控驾驶网关。
