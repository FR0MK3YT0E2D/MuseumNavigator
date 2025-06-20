# 博物館導覽機器人 (ROS 1 套件)

本專案是一個以 ROS 1 為基礎的模擬導覽機器人系統，適用於博物館場景。機器人能夠接收前端指令，根據指定展區自動導航至對應區域，並提供導覽資訊。整合了 Gazebo 模擬環境，RViz 視覺化工具，自主導航（AMCL + move\_base）與前端控制面板。

## 系統功能說明
* 提供網頁式前端介面，使用者可透過按鈕選擇導覽展區
* 每個展區對應機器人地圖上的特定區域 (3x3 九宮格)
* 機器人會根據目標自動導航，途中避開障礙物並使用 AMCL 進行定位
* RViz 提供即時可視化，包括機器人位置與導航路徑
* Gazebo 模擬現實環境與機器人運動
* 系統啟動後自動載入模型，地圖，AMCL，Move Base 與前端橋接程式

## 安裝與執行說明
### 系統需求
* Ubuntu 20.04
* ROS Noetic
* Python 3
* Gazebo 11
* 已安裝 roslibjs 與 rosbridge\_server 套件

### 依賴套件安裝
使用以下指令安裝必要套件：

```bash
sudo apt install ros-noetic-amcl ros-noetic-map-server ros-noetic-move-base \
                 ros-noetic-robot-state-publisher ros-noetic-rosbridge-server
```

### 建立與建置工作區
```bash
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/src
git clone https://github.com/your_name/museum_nav.git
cd ~/catkin_ws
catkin_make
```

### 啟動系統
```bash
source ~/catkin_ws/devel/setup.bash
roslaunch museum_nav bringup.launch
```

此指令會同時啟動 Gazebo 模擬器，RViz，AMCL 定位，地圖伺服器，Move Base 導航模組與前後端橋接節點。

## 控制面板操作

1. 開啟 `web/index.html`，使用瀏覽器開啟網頁
2. 根據你的網路狀況，請將 `index.html` 中的 rosbridge 地址 `ws://xxx.xxx.xxx.xxx:9090` 修改為實際執行 rosbridge 的主機 IP
3. 點選任一展區按鈕後，機器人將自動前往該區域的中心點
4. 當機器人到達指定展區後，頁面會顯示對應的導覽資訊

## 專案結構說明

```
museum_nav/
├── launch/                  # 啟動檔案
│   └── bringup.launch
├── maps/                    # 地圖與 YAML 描述檔
│   └── museum_map.yaml
├── models/                  # 機器人模型檔案（包含 .xacro 與 STL）
│   └── museum_robot/
├── scripts/                 # Python 腳本（已轉為 Python 3）
│   ├── goal_bridge.py       # 接收前端目標並轉送給 move_base
│   └── region_detector.py   # 根據 AMCL 位置發佈當前展區
├── urdf/                    # 機器人描述檔（Xacro 格式）
│   └── museum_robot.urdf.xacro
├── rviz/                    # RViz 配置檔
│   └── default.rviz
├── web/                     # 前端 HTML 控制面板
│   └── index.html
```

## 注意事項
* 使用 WSL 啟動 ROS 時，需手動查詢 IP 位址並將其填入前端介面的 rosbridge URL
* 展區以地圖中心 (0,0) 為原點，格點間距為 1 公尺，採用九宮格分區
* 若導航失敗可檢查是否被障礙物擋住，或 costmap 尚未正確建立
* 若模型異常或高度過低，可修改 `urdf/museum_robot.urdf.xacro` 中參數
