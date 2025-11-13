# filters

## Overview
フィルタをかますだけのパッケージ

## Usage
## Prerequisites
- ROS2 Humble
- C++ 17

### Download and Install
1. ワークスペースにこのパッケージをクローン
    ``` cmd
    cd ~/ros2_ws/src/
    git clone https://github.com/yazawakenichi/filters
    ```
2. パッケージのビルド
    ``` cmd
    cd ~/ros2_ws/
    colcon build --packages-select filters
    ```
3. 実行ファイルのオーバーレイ
    ``` cmd
    source install/local_setup.bash
    ```

### Launching Node
`ros2 launch` を用いることでノードを起動することができます。

新しくターミナルを開いた場合、ノードを起動する前に `source install/local_setup.bash` することを忘れないでください。

``` cmd
ros2 launch filters filters.launch.py
```

## Package Description

#### Topic
|Name|Type|Description
|---|---|---
|`/pub_topic`|`std_msgs::msg::String`| Publish するトピック
|`/sub_topic`|`std_msgs::msg::String`| Subscrbe するトピック

### Parameters
|Name|Type|Default|Description
|---|---|---|---
|`/pub_topic_name`|`std::string`|`"pub"`| パブリッシュするトピック
|`/sub_topic_name`|`std::string`|`"sub"`| サブスクライブするトピック

## License
このソフトウェアは、MIT License の下、再頒布および使用が許可されます。

(C) 2025 YAZAWA Kenichi

