##ROS-Navigation
[ROS-Navigation](http://wiki.ros.org/navigation)的內容相當多，要簡單了解並應用可以先至[Navigation Stack Setup](http://wiki.ros.org/navigation/Tutorials/RobotSetup)閱讀相關資料。程式的架構圖如下
![程式架構圖](./overview_tf_small.png)
- **白色區域: **
為主要的程式，不可拆開使用，包含了global與local的costmap，藉由地圖資訊與感測器資訊算出全域與區域性的grid map
- **灰色區域: **
為選擇性提供，其中map_server藉由提供png圖片產生grid map，而AMCL為slam的一種方法，提供/map至/odom的tf關係，使用者可以進而算出/map至/base_link的關係，了解機器人在地圖的實際位置
- **藍色區域: **
為使用者提供，包含了odom資訊、sensor資訊、並提供odom、sensor_link、base_link、base_footprint等相關的tf


####程式主體
- **robot_setup.py: **
提供機器人相關tf，可以在此處決定機器人的起始位置，由於還未使用slam，所以/map至/odom的轉換關係由使用者自行提供
- **fake_laser.launch: **
利用xtionpro，將深度資訊轉換成假的雷射資訊，並以/laserScan publish，使用ROS上depthimage_to_laserscan package
- **laser_filter.launch: **
接收由fake_laser.launch提供的laserScan，濾掉錯誤訊息，例如Nan, Inf 或是超過最大與最小值的量測值，將值轉換為 (max_range_value - small_value) 如此costmap才可正確的更新障礙物資訊

- **minimal.launch: **
為機器人本體的控制程式，提供odom資訊，並publish相關tf資訊，並接收由move_base.launch所算出的控制指令/cmd_vel
- **keyop.launch: **
開啟或關閉馬達電源，與直接以鍵盤控制機器人的遙控程式
- **move_base.launch: **
導航程式主體，在此開啟map_server，提供/map資訊，並開始rviz，使用圖形化介面控制

####相關參數
參數主要放置於yaml的folder，大致分為以下六種
- **move_base_params.yaml: **
決定control frequency 與 planner frequency 和 base_local_planner 與 base_global_planner 的使用

- **base_local_planner.yaml: **
分為TrajectoryPlannerROS與DWAPlannerROS，可以在move_base_param.yaml底下選擇，在此可以決定移動速度、目標點容忍度、軌跡模擬時間等
- **costmap_common_params.yaml: **
costmap分成global與local在此決定兩者所共同享有的設定，例如讀取的sensor種類與資訊，機器人本體描述footprint、膨脹半徑等...設定sensor資訊的marker與clear可以簡單作為sensor的開關，例如若只想要單純導航可以將marking設成false
- **global_costmap_params.yaml: **
設定global costmap的tf與update frequence
- **local_costmap_params.yaml: **
此處由於local costmap是與機器人本體移動，因此global frame應該設於odom，在有sensor的情況下，local costmap應與區域障礙相關，因此將static map設成false將rolling_window為true，決定resolution，此處resolution單位為m/pixel，應與map的resolution設成一致
- **map.yaml: **
提供地圖圖片路徑與解析度，此處pixel的多寡，決定了矩陣大小，也因此決定了計算量的大小

####實際應用
使用者在遠端連線至機器人端，開啟robot.launch，並在本地端開啟rviz以滑鼠傳送/move_base_simple/goal傳送指令，若遇到緊急狀況可以在robot.launch的terminal以鍵盤遙控機器人暫停

以目前來說control frequency: 5Hz 與 base_local_planner : DWAPlannerROS 是較為穩定的選擇

####待解決問題
- 在global costmap與local costmap中，障礙物的clear不夠即時，造成障礙物生成後移開無法避障

- control frequency 在超過5Hz時，dwa_local_planner會有無法轉彎的現象

- 使用TrajectoryPlannerROS 會有在目標點不停旋轉的問題
